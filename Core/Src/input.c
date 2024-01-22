
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
 const uint16_t  longPress = 2000;;
 const uint16_t  shortPress = 500;         // If the button was pressed less that this timeout, we assume the short button press
 volatile uint32_t pt;                       // Time in ms when the button was pressed (press time)

 char pinName[5] ="";
 struct State* pStat;

 void input_init( ){
	 pStat = getState();
 }
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
	 else if (GPIO_Pin == ENC_SW_Pin){
		 strcpy(pinName, "ENCSW");
		 bool keyUp = HAL_GPIO_ReadPin(ENC_SW_GPIO_Port, ENC_SW_Pin);
		 unsigned long now_t = HAL_GetTick();
		  if (!keyUp) {                                 // The button has been pressed
		    if ((pt == 0) || (now_t - pt > longPress)) pt = now_t;
		  } else {
		    if (pt > 0) {
		      if ((now_t - pt) < shortPress){// short press
		    	 switch(pStat->mode)
		    	 {
					 case HEATING: stateSetMode(WARM); break; // reset WARM to OFF count
					 case WARM:  stateSetMode(OFF); break;
					 case OFF: stateSetMode(HEATING); break; //  reset HEATING to WARM count
		    	 }
		      }
		      else{   // long press
		    	  if (pStat->mode !=  SETTING ){
		    		  pStat->mode =  SETTING;
		    	  }else{
		    		  pStat->mode =  OFF;
		    	  }
 		      }
		      pt = 0;
		    }
		  }

	 }
	 else if  (GPIO_Pin == PH_Pin ){ //Power heating button/paddle down/up
		if (  isPaddleDown()){
			stateSetMode(HEATING);
			pStat->highPower = true;
		}
		else{
			pStat->highPower = false;
		}
	 }
	 else if  (GPIO_Pin == Dock_Pin){
		if (isHandleDocked()){
			if (stateModeIs(HEATING)){
				stateSetMode(WARM);
			}
		}
		else{ //if(!isHandleDocked()){ //stateModeIs(WARM) &&
			if (stateModeIs(WARM)){
				stateSetMode(HEATING);
			}
		}
	}
	 previousMillis = currentMillis;
	 exti = 1;
}


bool isPaddleDown(){
	return HAL_GPIO_ReadPin(PH_GPIO_Port, PH_Pin) == 0;// TODO use PH_pin
}

bool isHandleDocked(){
	return (HAL_GPIO_ReadPin(Dock_GPIO_Port, Dock_Pin) == 0);
}

void checkHighPowerPaddle(){// a work aroung for failed exi int on dock pin. call it from main loop
	 if (  isPaddleDown()){
				 stateSetMode(HEATING);
				 pStat->highPower = true;
			//	 HAL_GPIO_WritePin(OUT_19_GPIO_Port, OUT_19_Pin, 1);
			}
	 else{
		 pStat->highPower = false;
	 }
//	if (isHandleDocked()){
//		if (stateModeIs(HEATING)){
//			stateSetMode(WARM);
//		}
//	// HAL_GPIO_WritePin(OUT_19_GPIO_Port, OUT_19_Pin, 1);
//	}
//	else{ //if(!isHandleDocked()){ //stateModeIs(WARM) &&
//		if (stateModeIs(WARM)){
//			stateSetMode(HEATING);
//		}
//	// HAL_GPIO_WritePin(OUT_19_GPIO_Port, OUT_19_Pin, 0);
//	}
}


/**
 * Check values of input pins
 */
//void checkPins(){
//
//	if (HAL_GPIO_ReadPin(Han_On_GPIO_Port, Han_On_Pin) == GPIO_PIN_RESET){ // handle button/foot paddle is pressed down
//		// forced heating
//		if (pState->mode == HEATING || pState->mode == WARM ){
//			pState->mode = FORCED_HEATING;
//		}
//	}else{
//		//pState->mode == HEATING;  // when foot paddle release, go to normal heating mode
//	}
//
//	if (HAL_GPIO_ReadPin(Han_Dock_GPIO_Port, Han_Dock_Pin) == GPIO_PIN_RESET){ // iron handle on holder, heater go to warm mode
//		if (pState->mode == HEATING) pState->mode = WARM;
//	}else{
//		//if (pState->mode == WARM) pState->mode = HEATING;
//	}
//}

