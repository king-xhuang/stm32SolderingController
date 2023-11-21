#include "main.h"
//#include "stm32f1xx_hal_i2c.h"
#include "heater.h"
uint32_t target = 300;  // target heater Temperature  in Celsius
uint32_t targetTempAdcVal ;  // target heater Temperature in ADC value (0-4095) acquired from the tc voltage amplified by OP


uint32_t adcVCalMax = 2560; // adv calibration high value at tip temp of 360 C. TODO need adjusted by experiment, determined by OP gain and tip tc property
uint32_t adcVCalMin = 770; // adv calibration low  value  at tip temp of 180 C.
float  vCalDelta = 0.0;
uint32_t tempCalMax = 440;  // measured tip temp when adc read = adcVCalMax
uint32_t tempCalMin = 240;  // measured tip temp when adc read = adcVCalMin
float  tempCalDelta = 0.0;
bool   disabled = true; // when is set to ture, heater is turned off and cannot be turn on
uint16_t onCountMax = 100;      // the max value of onCount ,in ms,
uint16_t onCountStepSize = 10; // onCount ,in ms, is set to be one of the n = onCountStepSize values
uint16_t onCount = 0;           // heater on time in ms

uint16_t onCountMap[10]; // array size should be the same as the onCountStepSize. store onCount value
						//for different vd = targetVal - adcVal. smaller index store smaller onCount value for smaller vd
uint32_t heaterStartTime = 0;//
uint32_t adcVCalStepCount = 20; // TODO passed in from heaterInit()

GPIO_TypeDef *GPIOxx;
uint16_t heaterPin;

//uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max) // need to declare this function before the calling function when not declared in .h file
//{
//  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;  // TODO need float type????
//}

uint32_t map(float x, float in_min, float in_max, float out_min, float out_max) // need to declare this function before the calling function when not declared in .h file
{
	if (x < in_min){
		return  (uint32_t)(out_min - (in_min - x) * (out_max - out_min) / (in_max - in_min) );
	}else{
		return (uint32_t)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;  // TODO need float type????
	}
}
/*
 * get heater on time in ms
 */
uint16_t heaterOnCount(){
	return onCount;
}
bool heaterIsDisabled(){
    return disabled;
}
bool isOn(){
	if ( HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin) == GPIO_PIN_SET) return true;
	else return false;
}
void heaterEnable(bool val){
	if ( val == false){
		heaterOff();
		disabled = true;
	}else{
		disabled = false;
	}

}
void heaterInit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint16_t onMax){
	heaterEnable(true);

	// calculate onCountMap
	onCountMax = onMax;
	short arryLen = sizeof(onCountMap)/sizeof(onCountMap[0]);
	if ( arryLen!= onCountStepSize){
		onCountStepSize = arryLen;
	}
	for (short i = 0; i < onCountStepSize; i++){
		onCountMap[i] =  onCountMax*(i+1)/onCountStepSize;
	}
	onCountMap[onCountStepSize-1] = onCountMax;

	heaterPin = GPIO_Pin;
	GPIOxx = GPIOx;

	tempCalDelta = (float)(tempCalMax -tempCalMin);
	vCalDelta    = (float)(adcVCalMax - adcVCalMin);
}
void heaterCheckOnTime(){
	if ( isOn() && ((HAL_GetTick() - heaterStartTime) > onCount)){
		 heaterOff();
	}
}
/*
 * based on v diff
 */

uint16_t calcOnCount(int32_t vd){  // TODO  need a map
	uint16_t c = 0;
   if (vd < 0)  c = 0;
   else{
	   short index=  (short)vd/adcVCalStepCount;
       if (index >= onCountStepSize) index = onCountStepSize - 1;
	   c = onCountMap[index];
   }
   return c;
}
/**'
 *
 */
void heaterCheckTemp(uint32_t adcVal, uint16_t tick){
	if (tick != target){
		heaterSetTemp(tick);
	}

	int32_t vd = (int32_t)targetTempAdcVal - (int32_t)adcVal;
	//onCount = calcOnCount(vd);
	//if (onCount >  0){ //TODO
	if (vd  >  0){
		heaterStartTime = HAL_GetTick();
		heaterOn();
	}else{
		heaterOff();
	}
}

uint32_t heaterTargetAdcV(){
	return targetTempAdcVal;
}

void heaterSetTemp(uint16_t temp){
	target = temp;
	targetTempAdcVal = temp2V(temp);
}

void heaterOn(){
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(GPIOxx, heaterPin, GPIO_PIN_SET);
 }

void heaterOff(){

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(GPIOxx, heaterPin, GPIO_PIN_RESET); TODO system hang after calling this function. Why??? So direct call HAL_GPIO_WritePin as walk around
}
/**
 * convert temp (in C) to adv val
 */
uint32_t temp2V(uint16_t temp){

	return map(temp, tempCalMin, tempCalMax, adcVCalMin, adcVCalMax);
}

uint16_t v2temp(uint32_t adcValue){
	return map(adcValue, adcVCalMin, adcVCalMax, tempCalMin, tempCalMax);
}
