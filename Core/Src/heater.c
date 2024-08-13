#include "main.h"
//#include "stm32f1xx_hal_i2c.h"
#include "heater.h"

static const int HighTempDelta = 30;
uint32_t targetTemp = 300;  // target heater Temperature  in Celsius
uint32_t targetWarmTemp = 150;
uint32_t targetHighTemp  = 380;

uint32_t targetTempAdcVal;  // target heater Temperature in ADC value (0-4095) acquired from the tc voltage amplified by OP
uint32_t targetWarmTempAdcVal;
uint32_t targetHighTempAdcVal;

#ifdef C210
uint32_t adcVCalMax = 1575; // adv calibration high value at tip temp of 330 C.   need adjust by experiment, determined by OP gain and tip tc property
uint32_t adcVCalMin = 241; // adv calibration low  value  at tip temp of 150 C.

uint32_t tempCalMax = 330;  // measured tip temp when adc read = adcVCalMax
uint32_t tempCalMin = 150;  // measured tip temp when adc read = adcVCalMin
#endif
#ifdef C245
uint32_t adcVCalMax = 2593; // adv calibration high value at tip temp of tempCalMax. TODO need adjusted by experiment, determined by OP gain and tip tc property
uint32_t adcVCalMin = 1341; // adv calibration low  value  at tip temp of tempCalMin.

uint32_t tempCalMax = 360;  // measured tip temp when adc read = adcVCalMax
uint32_t tempCalMin = 200;  // measured tip temp when adc read = adcVCalMin
#endif

float  vCalDelta = 0.0;
float  tempCalDelta = 0.0;
bool   disabled = true; // when is set to ture, heater is turned off and cannot be turn on

GPIO_TypeDef *heaterPinPort;
uint16_t heaterPin;

struct State*  pSt;

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
//uint16_t heaterOnCount(){
//	return onCount;
//}
bool heaterIsDisabled(){
    return disabled;
}
//bool isOn(){
//	//if ( HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin) == GPIO_PIN_SET) return true;
//	if ( HAL_GPIO_ReadPin(HEATER_GPIO_Port, HEATER_Pin) == GPIO_PIN_SET) return true;
//	else return false;
//}
void heaterEnable(bool val){
	if ( val == false){
		heaterOff();
		disabled = true;
	}else{
		disabled = false;
	}

}
void heaterInit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin  ){ //uint16_t onMax){
	heaterEnable(true);
	//sta = st;
	pSt  = getState();
	// calculate onCountMap
//	onCountMax = onMax;
//	short arryLen = sizeof(onCountMap)/sizeof(onCountMap[0]);
//	if ( arryLen!= onCountStepSize){
//		onCountStepSize = arryLen;
//	}
//	for (short i = 0; i < onCountStepSize; i++){
//		onCountMap[i] =  onCountMax*(i+1)/onCountStepSize;
//	}
//	onCountMap[onCountStepSize-1] = onCountMax;

	heaterPin = GPIO_Pin; //TODO
	heaterPinPort = GPIOx;  //TODO

	tempCalDelta = (float)(tempCalMax -tempCalMin);
	vCalDelta    = (float)(adcVCalMax - adcVCalMin);

	targetWarmTempAdcVal = temp2V(targetWarmTemp);


}
//void heaterCheckOnTime(){
//	if ( isOn() && ((HAL_GetTick() - heaterStartTime) > onCount)){
//		 heaterOff();
//	}
//}
/*
 * based on v diff
 */

//uint16_t calcOnCount(int32_t vd){  // TODO  need a map
//	uint16_t c = 0;
//   if (vd < 0)  c = 0;
//   else{
//	   short index=  (short)vd/adcVCalStepCount;
//       if (index >= onCountStepSize) index = onCountStepSize - 1;
//	   c = onCountMap[index];
//   }
//   return c;
//}
/**'
 *
 */
void heaterCheckTemp(uint32_t adcVal, uint16_t tick){
	if (tick != targetTemp){
		heaterSetTemp(tick);
	}
	uint32_t tempSetAdcVal = targetTempAdcVal;
	if(stateModeIs(WARM)){
		tempSetAdcVal = targetWarmTempAdcVal;
	}else if (stateModeIs(HEATING)){
		if (pSt->highPower){
			tempSetAdcVal = targetHighTempAdcVal;
		}
	}

	int32_t vd = (int32_t)tempSetAdcVal - (int32_t)adcVal;

	if (vd  >  0){// temp lower than temp set
		if (HCAgetType() == HCAPID){
			//TODO calculate/set PowerLevel based on PID rule
			setPowerLevel(getHCTickMax()/2);//TODO
		}else{//HCAOnOff
			setPowerLevel(getHCTickMax());
		}

	}else{ // temp higher than temp set

		if (HCAgetType() == HCAOnOff){
			setPowerLevel(0);
			HCAset(HCAPID);//at the first time when temp passed the target, change HCA to PID
		}
		if (HCAgetType() == HCAPID){
			//TODO calculate/set PowerLevel based on PID rule
			setPowerLevel(0);//TODO
		}
	}
	if (getPowerLevel() > 0) heaterOn();
	else heaterOff();
}

uint32_t heaterTargetTemp(){
	return targetTemp;
}
uint32_t heaterWarmTargetTemp(){
	return targetWarmTemp;
}
uint32_t heaterHighTargetTemp(){
	return targetHighTemp;
}
uint32_t heaterTargetAdcV(){
	return targetTempAdcVal;
}
uint32_t heaterWarmTargetAdcV(){
	return targetWarmTempAdcVal;
}
uint32_t heaterHighTargetAdcV(){
	return targetHighTempAdcVal;
}

void heaterSetTemp(uint16_t temp){
	targetTemp = temp;
	targetTempAdcVal = temp2V(temp);
	targetHighTemp = targetTemp + HighTempDelta;
	targetHighTempAdcVal = temp2V(targetHighTemp);
}

void heaterOn(){
	HAL_GPIO_WritePin(HEATER_GPIO_Port, HEATER_Pin, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
 }

void heaterOff(){
	HAL_GPIO_WritePin(HEATER_GPIO_Port, HEATER_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	// HAL_GPIO_WritePin(heaterPinPort, heaterPin, GPIO_PIN_RESET);// cannot use port, pin passed by guiInit() function
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
