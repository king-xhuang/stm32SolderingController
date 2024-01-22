/*
 * buzzerTone.h
 *
 *  Created on: Jan 21, 2024
 *      Author: Administrator
 */

#ifndef INC_BUZZERTONE_H_
#define INC_BUZZERTONE_H_
#include "main.h"
#include <stdbool.h>
#include <stdint.h>

enum ToneType{
	HeatTone,
	WarmTone,
	PowerHeatTone,
	StopTone
};
void buzzToneInit(TIM_HandleTypeDef* pTimer);
void playHeatTone();
void playTone(enum ToneType t);

void toneTickHandler();

#endif /* INC_BUZZERTONE_H_ */
