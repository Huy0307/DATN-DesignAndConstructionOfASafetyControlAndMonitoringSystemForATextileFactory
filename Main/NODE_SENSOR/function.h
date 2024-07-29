#ifndef FUNCTION_H
#define FUNCTION_H
uint16_t checksumCalculator(uint8_t* data, uint16_t length);
uint16_t crc_cal(void* data, size_t size);
boolean runEvery(unsigned long interval);
void Receive_Config(void* pvParameters);
void rtos_delay(uint16_t delay_in_ms);
void print_wakeup_reason();
void deep_sleep();
void setup_node();
void loop_node();
#endif