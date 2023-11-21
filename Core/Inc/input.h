
#ifndef INPUT_H
#define INPUT_H 100

/* C++ detection */
#ifdef __cplusplus
extern C {
#endif

#include <stdbool.h>
#include <stdint.h>


bool isExti();// return the value of exti, it's value is true(1)  when an ext interupt happened

void resetExti(); // reset exti = 0, reset ext interupt time.

void input_init(void);

void exti_callback(uint16_t GPIO_Pin);  // ext interupt call back for all digital input pins

char* getExtiSrc(); // return ext interupt pin name

uint16_t encoderGetTick();  //get encoder count value

void encoderSetTick(uint16_t tick); // set encoder count value

#ifdef __cplusplus
}
#endif

#endif
