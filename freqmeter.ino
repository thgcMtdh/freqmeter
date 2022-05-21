#include "src/freq.h"

const uint8_t PIN_F_SENSE = 25;

float freqs[10] = {0,0,0,0,0,0,0,0,0,0};
size_t idx = 0;

void setup() {
  Serial.begin(115200);

  freq_init(PIN_F_SENSE, 512);
  delay(1000);
  freq_start_measurement();
  delay(1000);
}

void loop() {
  float* buf;
  size_t buf_size = freq_read_buffer(&buf);
  for (size_t i=0; i<buf_size; i++)
  {
    freqs[idx++] = buf[i];
    if (idx >= 10) { idx = 0; }

    float average = 0;
    for (size_t j=0; j<10; j++) {
      average += freqs[j];
    }
    average /= 10;
    
    Serial.printf("%f, 50000, 49900, 50100\n", average*1000.0);
  }
  delay(1);
}
