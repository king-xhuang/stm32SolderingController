#include "common.h"

const uint32_t TimeToOff = 10*60*1000; // time from WARM to OFF mode in millis/adcCheckPeriod       minutes*60^1000
const uint32_t TimeToWarm = 2*60*1000; // time from HEATING to WARM mode in millis/adcCheckPeriod   minutes*60^1000

volatile uint32_t warmEndTime;
volatile uint32_t heatingEndTime; //  in millis

//extern volatile uint32_t warmEndTime;
//extern volatile uint32_t heatingEndTime; //  in millis
volatile struct State state;
volatile bool beep = false;
volatile bool save2eeprom = false;
volatile enum ToneType toneType = StopTone;
//check beep flag. Stop an existing beeper and start a new one when flag is set.
void checkBeepFlag(){
	if (beep){
		beep = false;
		toneRepeatTickEnd();
		playTone(toneType);
	}
}
// set a beep flag and tone type
void setBeepFlag(bool b, enum ToneType t){
	beep = b;
	toneType = t;
}
bool save2Eeprom(){
	if(save2eeprom){
		save2eeprom = false;
		return true;
	}else return false;
}
void stateSetMode(enum Mode mode){
	switch(mode){

	case HEATING:
		if(state.mode == HEATING) break;
		state.mode = mode;
		heatingEndTime = HAL_GetTick()  + TimeToWarm;

		if(state.highPower){
			setBeepFlag(true, PowerHeatTone);
		}
		else{
			setBeepFlag(true, HeatTone);
		}
		break;
	case WARM:
		if(state.mode == WARM) break;
		setBeepFlag(true,WarmTone );
		state.mode = mode;
		warmEndTime = HAL_GetTick()  + TimeToOff;
		break;
	case OFF:
		if (state.mode == WARM){
			setBeepFlag(true, StopTone);
			save2eeprom = true;
		}
		state.mode = mode;
		break;
	}
}

bool stateModeIs(enum Mode mode){
	return state.mode == mode;
}


void checkStateTimeout(uint32_t currentTick ){
	if (stateModeIs(WARM) && warmEndTime <= currentTick){
		 stateSetMode(OFF);
	}
	else if (stateModeIs(HEATING) && heatingEndTime <= currentTick){
		 stateSetMode(WARM);
	}
}
struct State* getState(){
	return &state;
}

/**
 * convert a float to char[], the size of char[] should be int part len + decimal part len + 1
 */
//void float2Char(float f, char *sval){
//	int tmpInt1 = f;                  // Get the integer (678).
//	float tmpFrac = f - tmpInt1;      // Get fraction (0.0123).
//	int tmpInt2 = trunc(tmpFrac * 100);  // Turn into integer (123).
//
//	// Print as parts, note that you need 0-padding for fractional bit.
//
//	sprintf (sval, "%d.%03d\n",   tmpInt1, tmpInt2);
//
//}
