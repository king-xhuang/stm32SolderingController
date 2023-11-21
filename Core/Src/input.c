
//#include "stm32f1xx_hal.h"
#include "main.h"

#include "input.h"
#include "heater.h"
#include <string.h>


 uint32_t previousMillis = 0;
 uint32_t currentMillis = 0;
 const uint32_t fast_inc_time = 300;
 const uint8_t fast_inc = 5;
 const uint8_t norm_inc = 1;
 uint8_t inc = 1;
// uint32_t counterOutside = 0; //For testing only
// uint32_t counterInside = 0; //For testing only
 volatile uint16_t encoderCt = 300;
 const uint16_t encoderCtMax = 500;
 const uint16_t encoderCtMin = 150;
 uint8_t exti = 0;

 char pinName[5] ="";
 void resetEncoder(){

 }
 bool isExti(){
	 return exti;
 }
 void  resetExti(){
	 exti = 0;
//	 previousMillis = currentMillis;
 }
 uint16_t encoderGetTick(){
	 return encoderCt;
 }
 void encoderSetTick(uint16_t tick){
 	   encoderCt = tick;
  }
 char* getExtiSrc(){
	 return pinName;
 }
void exti_callback(uint16_t GPIO_Pin){

	currentMillis = HAL_GetTick();
	 if(GPIO_Pin == ENC_A_Pin ){
		 bool plus = HAL_GPIO_ReadPin(ENC_A_GPIO_Port, ENC_A_Pin) == HAL_GPIO_ReadPin(ENC_B_GPIO_Port, ENC_B_Pin);

		if (currentMillis - previousMillis <= fast_inc_time) {
			inc   = fast_inc;
		}else{
			inc = norm_inc;
		}

	    if(plus){
			 encoderCt = encoderCt + inc;

		 }else{
			 encoderCt = encoderCt - inc;

		 }
	    if (encoderCt > encoderCtMax) encoderCt = encoderCtMax;
	    else if (encoderCt < encoderCtMin) encoderCt = encoderCtMin;
		 strcpy(pinName, "ENC_A");
	 }
	 else if (GPIO_Pin == ENC_SW_Pin && (currentMillis - previousMillis > 20)){
		 strcpy(pinName, "ENCSW");

	 }
	 else if  (GPIO_Pin == Han_On_Pin && (currentMillis - previousMillis > 20)){
		 strcpy(pinName, "HA_SW");
	 }
	 else if  (GPIO_Pin == Han_Dock_Pin && (currentMillis - previousMillis > 20)){
			 strcpy(pinName, "HA_SP");
	 }
	 previousMillis = currentMillis;
	 exti = 1;
}
/**
 * Check values of input pins
 */
void checkPins(){

	if (HAL_GPIO_ReadPin(Han_On_GPIO_Port, Han_On_Pin) == GPIO_PIN_RESET){ //handle button is pressed down

//		if (!heaterIsDisabled()){
//			heaterOn();  // force heaterOn regardless adcVal
//		}
	}
}
