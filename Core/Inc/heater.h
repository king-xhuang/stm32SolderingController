
#ifndef HEATER_H
#define HEATER_H 102

/* C++ detection */
#ifdef __cplusplus
extern C {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "common.h"


void heaterInit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);  //uint16_t onCountMax);
void heaterSetTemp(uint16_t temp);
void heaterOn();
void heaterOff();
bool heaterIsDisabled();
//uint16_t heaterOnCount();
/**
 * Disable heater - false
 * Enable heater -  true
 */
void heaterEnable(bool val);
void heaterCheckTemp(uint32_t adcVal, uint16_t tick);
//void heaterCheckOnTime();
uint32_t heaterTargetTemp();
uint32_t heaterWarmTargetTemp();
uint32_t heaterHighTargetTemp();
uint32_t heaterTargetAdcV();
uint32_t heaterWarmTargetAdcV();
uint32_t heaterHighTargetAdcV();
uint16_t v2temp(uint32_t adcValue);
uint32_t temp2V(uint16_t temp);


#ifdef __cplusplus
}
#endif

#endif
