

#include "buzzerTone.h"
#include "stm32f1xx_hal.h"
TIM_HandleTypeDef* hTimer;

void buzzToneInit(TIM_HandleTypeDef* pTimer){
	hTimer = pTimer;
}
// parameters for tone ticks
const uint16_t toneTickPeriod = 125; // ms. Tone tick period is the time unit for playing a tone
const uint16_t toneRepeatPeriod =  toneTickPeriod*8*5; // for circular tones only
uint32_t nextToneTickTime = 0;  // ms. Time to check tone playing status.
uint32_t nextTonePlayTime = 0;  // ms. for circular tone, repeat every toneRepeatPeriod
bool isToneTick = false;        // true - a ToneTick is created
bool isRepeatToneTick = false;  // true - a RepeatToneTick is created
bool genToneTick = false;       // true - be able to generate ToneTick
bool genRepeatToneTick = false; // true - be able to generate RepeatToneTick// notes

// notes
const uint16_t tC5 = 1912;
const uint16_t tD5 = 1703;
const uint16_t tE5 = 1517;
const uint16_t tHi = 1200;
const uint16_t tLow = 2400;

// tones

const uint8_t heatToneSize = 3;
const uint16_t heatToneARR[3] = {tC5, tD5, tE5}; // c5, d5, e5  // tone array -- tone represented by ARR count
const uint8_t heatToneDur[3] = {2, 1, 2};   // array of play duration for each tone, in unit of tone tick,


const  uint8_t powerHeatToneSize = 2;
const  uint16_t powerHeatToneARR[2] = { tLow, tHi}; //
const  uint8_t powerHeatToneDur[2] = {1, 2};   // array of play duration for each tone, in unit of tone tick,


const  uint8_t warmToneSize = 3;
const  uint16_t warmToneARR[3] = {tE5, tD5, tC5}; // c5, d5, e5  // tone array -- tone represented by ARR count
const  uint8_t warmToneDur[3] = {2, 1, 6};   // array of play duration for each tone, in unit of tone tick,


const  uint8_t stopToneSize = 5;
const  uint16_t stopToneARR[5] = {tE5, tD5, tC5, tD5, tC5}; // c5, d5, e5  // tone array -- tone represented by ARR count
const  uint8_t stopToneDur[5] = {2, 1, 2, 1, 6};   // array of play duration for each tone, in unit of tone tick,


// tone player
uint16_t*  arr; // tone array -- tone represented by ARR count
uint8_t*  dur;  // array of play duration for each tone, in unit of tone tick,
uint8_t toneSize;

bool isPlayTone = false;
bool isRepeatTone = false;

uint16_t on_peroide = 100; //in us. Pulse on time
uint8_t curToneIndex = 0;  // the index of the tone being played currently
uint8_t curToneDur = 0;    // the tone duration, in unit of tone tick, being played currently

void setTone(uint16_t* pArr,  const uint8_t* pDur, const uint8_t sz, bool rep){// to pass tone config parameters
//	 for(int i = 0; i < sz; i++){
//		 arr[i] = pArr[i];
//	 }
//	 for(int i = 0; i < sz; i++){
//	 		 dur[i] = pDur[i];
//	 	 }

	arr = &pArr[0];
	dur = &pDur[0];
	toneSize = sz;  // size of tone array
	isRepeatTone = rep;
}
void playHeatTone(){
//	setTone(heatToneARR, heatToneDur, heatToneSize, false);
//	startTonePlay();
	playTone(WarmTone);
}
void playTone(enum ToneType t){
	switch(t){
		case PowerHeatTone:	setTone(powerHeatToneARR, powerHeatToneDur, powerHeatToneSize, true); break;
		case HeatTone: setTone(heatToneARR, heatToneDur, heatToneSize, true);break;
		case WarmTone: setTone(warmToneARR, warmToneDur, warmToneSize, false);break;
		case StopTone: setTone(stopToneARR, stopToneDur, stopToneSize, false);break;
	}

	startTonePlay();
}


void startTonePlay(){
	isPlayTone = 1; // indicating a new tones play is started
	curToneIndex = 0; // start with 1st note
    toneTickBegin(false);
	__HAL_TIM_SET_COMPARE(hTimer, TIM_CHANNEL_1, on_peroide);
	HAL_TIM_OC_Start(hTimer, TIM_CHANNEL_1);
	TIM3->ARR = arr[curToneIndex];
}
void checkTonePlay(){
	if (isPlayTone == 0) return; // no tone is played
	curToneDur++; // tone duration increased by one
	if (dur[curToneIndex] > curToneDur){
		return; // continue play the tone
	}
	else{
		curToneIndex++;
		if(curToneIndex >= toneSize){ // no more tone to play
			endTonePlay(false);
			return;
		}else{ // play next tone
			curToneDur = 0;
			TIM3->ARR = arr[curToneIndex]; // reset ARR -- set new tone
		}
	}
}
/*
 *
 * forceStop, false -  schedule next play for repeatable tone, true -- stop tone play
 */
void endTonePlay(bool forceStop){
	isPlayTone = 0;
	curToneIndex = 0;
	curToneDur = 0;
	HAL_TIM_OC_Stop(hTimer, TIM_CHANNEL_1);

    toneTickEnd(); // stop tone tick gen
    if (forceStop) return;
    else if (isRepeatTone  ){//  if circular tones
		toneRepeatTickBegin(false);
	}
}


// tone tick generator

void setNextToneTick(){
	nextToneTickTime  = HAL_GetTick() + toneTickPeriod;
}
void setNextTonePlayTime(){
	nextTonePlayTime = HAL_GetTick() + toneRepeatPeriod;
}

void toneTickBegin(bool tickNow){
	if (tickNow) isToneTick = true;
	genToneTick = true;
	setNextToneTick();
}

void toneTickEnd(){
	isToneTick = false;;
	genToneTick = false;
}
void toneRepeatTickBegin(bool now){
	if(now) isRepeatToneTick = true;
	genRepeatToneTick = true;
	setNextTonePlayTime();
}
void toneRepeatTickEnd(){
	isRepeatToneTick = false;
	genRepeatToneTick = false;
}
void toneTickGen(){// tone tick generator, called in while loop
	if (genToneTick){
		if (HAL_GetTick() >= nextToneTickTime){
			isToneTick = true; // set tone tick flag
			setNextToneTick(); // set next tone tick check time
		}
	}
	if (genRepeatToneTick){
		if (HAL_GetTick() >= nextTonePlayTime){
			isRepeatToneTick = true; // set repeat tones tick flag
			setNextTonePlayTime(); // set next repeat tones tick check time
		}
	}

}

void toneTickHandler(){ // process tone tick, called in while loop

	toneTickGen();
	if (isRepeatToneTick){
		isRepeatToneTick = false;
		toneTickBegin(false);
		startTonePlay();
	}
	else if (isToneTick){
		isToneTick = false;
	    checkTonePlay();
	}
}
