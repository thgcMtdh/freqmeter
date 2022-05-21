#ifndef INC_FREQ_H
#define INC_FREQ_H

#include <Arduino.h>

void freq_init(uint8_t, size_t);
void freq_start_measurement();
size_t freq_read_buffer(float**);
void freq_stop_measurement();
void freq_end();

#endif
