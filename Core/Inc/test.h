#ifndef TEST_H
#define TEST_H 404

/* C++ detection */
#ifdef __cplusplus
extern C {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "common.h"
#include "heater.h"

void testInit(I2C_HandleTypeDef *hi2c1, UART_HandleTypeDef *huart1);
void testTemp2V();

#ifdef __cplusplus
}
#endif

#endif
