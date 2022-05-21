/**
 * @file freq.cpp
 * @author thgcMtdh
 * @brief 
 */

#include "freq.h"

#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "soc/rtc.h"
#include "driver/mcpwm.h"

// minimum pulse count
// if the measured interval is less than PULSE_COUNT_MIN,
// the result is ignored because it should be a noise capture
static const uint32_t PULSE_COUNT_MIN = 80000000 / 70;  // (APB freq) / (70Hz)

typedef struct {
  uint32_t capture_signal;
  mcpwm_capture_signal_t sel_cap_signal;
} capture;

static QueueHandle_t cap_queue;

static volatile uint32_t cap_val = 0;
static volatile uint32_t cap_val_prev = 0;

static volatile bool is_run = false;

float* freq_buffer = NULL;
float* freq_buffer_to_read = NULL;
static size_t freq_buffer_len = 0;
static size_t freq_buffer_write_index = 0;
static size_t freq_buffer_read_index = 0;

static portMUX_TYPE mutex_freq_buffer = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t captured_task_handle = NULL;


// private functions

/**
 * @brief this is an ISR callback, we take action according to the captured edge
 */
static bool IRAM_ATTR isr_handler(mcpwm_unit_t mcpwm, mcpwm_capture_channel_id_t cap_sig, const cap_event_data_t *edata, void *arg) {
    BaseType_t high_task_wakeup = pdFALSE;

    // calculate the interval in the ISR,
    // so that the interval will be always correct even when cap_queue is not handled in time and overflow.
    cap_val_prev = cap_val;
    cap_val = edata->cap_value;
    uint32_t pulse_count = cap_val - cap_val_prev;
    // send measurement back through queue
    if (is_run && pulse_count > PULSE_COUNT_MIN)
    {
      xQueueSendFromISR(cap_queue, &pulse_count, &high_task_wakeup);
    }

    return high_task_wakeup == pdTRUE;
}

/**
 * @brief a task running on each capture event. calculate frequency and store it to the buffer
 */
static void captured_task(void* pvParameters) {
  while (true)
  {
    uint32_t pulse_count;
    // block and wait for new measurement
    xQueueReceive(cap_queue, &pulse_count, portMAX_DELAY);
    // calculate frequency
    float freq_measured = (float)rtc_clk_apb_freq_get() / pulse_count;
    // push into buffer
    portENTER_CRITICAL(&mutex_freq_buffer);
    freq_buffer[freq_buffer_write_index++] = freq_measured;
    if (freq_buffer_write_index >= freq_buffer_len)
    {
      freq_buffer_write_index = 0;
    }
    portEXIT_CRITICAL(&mutex_freq_buffer);
  }
}


// API functions

/**
 * @brief 周波数計測機能を初期化し、実行可能な状態にする
 * @param gpio_cap_in 周波数パルスを入力するピン番号
 * @param buffer_len 計測した周波数を保存しておくバッファの個数
 *                   あふれた場合は後ろから順に上書きされる。
 */
void freq_init(uint8_t gpio_cap_in, size_t buffer_len)
{
  ESP_LOGI(TAG, "initializing mcpwm gpio...");

  // allocate frequency data buffer
  freq_buffer_len = buffer_len;
  freq_buffer = (float*)calloc(buffer_len, sizeof(float));
  freq_buffer_to_read = (float*)calloc(buffer_len, sizeof(float));
  if (!freq_buffer || !freq_buffer_to_read)
  {
    while (true);
  }

  // the queue where we read data
  cap_queue = xQueueCreate(1, sizeof(uint32_t));

  // set GPIO
  mcpwm_pin_config_t mcpwm_gpio_config = {
    .mcpwm0a_out_num = -1,
    .mcpwm0b_out_num = -1,
    .mcpwm1a_out_num = -1,
    .mcpwm1b_out_num = -1,
    .mcpwm2a_out_num = -1,
    .mcpwm2b_out_num = -1,
    .mcpwm_sync0_in_num  = -1,  
    .mcpwm_sync1_in_num  = -1,  
    .mcpwm_sync2_in_num  = -1,  
    .mcpwm_fault0_in_num = -1,
    .mcpwm_fault1_in_num = -1,  
    .mcpwm_fault2_in_num = -1,
    .mcpwm_cap0_in_num = gpio_cap_in,
    .mcpwm_cap1_in_num = -1,
    .mcpwm_cap2_in_num = -1
  };
  mcpwm_set_pin(MCPWM_UNIT_0, &mcpwm_gpio_config);
  gpio_pullup_en((gpio_num_t)gpio_cap_in);

  // enable rising edge capture on CAP0
  mcpwm_capture_config_t cap_config = {
    .cap_edge = MCPWM_POS_EDGE,
    .cap_prescale = 1,
    .capture_cb = isr_handler,
    .user_data = NULL
  };
  mcpwm_capture_enable_channel(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, &cap_config);

  // create a task that handles capture events
  xTaskCreatePinnedToCore(captured_task, "captured_task", 2048, NULL, 2, &captured_task_handle, APP_CPU_NUM); 

  ESP_LOGI(TAG, "initializiation complete");
}

/**
 * @brief 周波数計測を開始する
 */
void freq_start_measurement()
{
  is_run = true;
}

/**
 * @brief バッファに溜まっている計測データを全て読み出す. 個数は毎回変わるので必ず返り値を確認すること
 * @param[out] data 読み出したデータ配列の先頭アドレスを格納する、float*型へのポインタ
 * @return 読み出したデータの個数
 */
size_t freq_read_buffer(float** data)
{
  size_t i = 0;
  portENTER_CRITICAL(&mutex_freq_buffer);
  while (freq_buffer_read_index != freq_buffer_write_index)
  {
    freq_buffer_to_read[i++] = freq_buffer[freq_buffer_read_index++];
    if (freq_buffer_read_index >= freq_buffer_len)
    {
      freq_buffer_read_index = 0;
    }
  }
  portEXIT_CRITICAL(&mutex_freq_buffer);

  *data = freq_buffer_to_read;
  return i;
}

/**
 * @brief 周波数計測を終了する
 */
void freq_stop_measurement()
{
  is_run = false;
}

/**
 * @brief 周波数計測モジュールからリソースを開放する
 */
void freq_end() {
  mcpwm_capture_disable_channel(MCPWM_UNIT_0, MCPWM_SELECT_CAP0);
  vTaskDelete(captured_task_handle);
  captured_task_handle = NULL;
  cap_val = 0;
  cap_val_prev = 0;
  freq_buffer_len = 0;
  freq_buffer_write_index = 0;
  freq_buffer_read_index = 0;
  free(freq_buffer);
  free(freq_buffer_to_read);
}



