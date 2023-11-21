#ifndef EEPROM_H
#define  EEPROM_H 200

/* C++ detection */
#ifdef __cplusplus
extern C {
#endif
#include "stm32f1xx_hal_i2c.h"

#define EEPROM_ADDR 0xA0

void  cfgSaveEncTick(  uint16_t encoderCt );
uint16_t cfgGetEncTick( );
void cfgInit(I2C_HandleTypeDef *hi2c1, UART_HandleTypeDef *huart1);

void testCfg();
//void testPages();
//void  testEEPROM( );

#ifdef __cplusplus
}
#endif

#endif
