#include "gui.h"
#include "fonts.h"
#include "ssd1306.h"

char stemp[5];
char sAdcVal[4];
char sAdcVal2[4];
char sMode[1];
char sPower[3];
char sTip[4];


void guiInit( ){
	strcpy(sTip,"????");
	#ifdef C210
	strcpy(sTip,"C210");
	#endif
	#ifdef C245
	strcpy(sTip,"C245");
	#endif
	SSD1306_Init();
}

void guiUpdate(uint32_t adcValues[] ){
	uint8_t x = 0;
#ifdef C245
    x = 4;
#endif

	SSD1306_Fill (0);
    uint16_t target = heaterTargetTemp();
    uint32_t targetav = heaterTargetAdcV();
    if (stateModeIs( WARM)){
    	target = heaterWarmTargetTemp();
    	targetav =  heaterWarmTargetAdcV();
    }
    else if(stateModeIs(HEATING) && getState()->highPower){
    	target = heaterHighTargetTemp();
    	targetav = heaterHighTargetAdcV();
    }
	// show target temp at 1st line
    utoa(target, stemp, 10);
	SSD1306_GotoXY (x, 0);
	SSD1306_Puts (stemp, &Font_11x18, 1);

	utoa(targetav, sAdcVal, 10);
	SSD1306_GotoXY (x+40, 0);
	SSD1306_Puts (sAdcVal, &Font_11x18, 1);

	switch(getState()->mode){
		case HEATING: sMode[0] = 'H'; break;
		case FORCED_HEATING: sMode[0] = 'F'; break;
		case OFF: sMode[0] = 'O'; break;
		case WARM: sMode[0] = 'W'; break;
		case SETTING: sMode[0] = 'S'; break;
	}

	SSD1306_GotoXY (x+112, 0);
	SSD1306_Puts (sMode, &Font_11x18, 1);

	// show current temp on 2nd line
	utoa(getState()->currentTemp, stemp, 10);
	SSD1306_GotoXY (x, 20);
	SSD1306_Puts (stemp, &Font_11x18, 1);

	// show tip type on 2nd line right
	SSD1306_GotoXY (x+80, 20);
	SSD1306_Puts (sTip, &Font_11x18, 1);

	// show current currentAdcVal on 3rd line
	utoa(getState()->currentAdcVal, sAdcVal, 10);
	SSD1306_GotoXY (x, 40);
	SSD1306_Puts (sAdcVal, &Font_11x18, 1);

    itoa(getPowerLevel(), sPower, 10);
    SSD1306_GotoXY (x+ 90, 40);
	SSD1306_Puts (sPower, &Font_11x18, 1);
	//  most near target acd value
//	utoa(mcreadValue, sAdcVal2, 10);
//	SSD1306_GotoXY (x+60, 40);
//	SSD1306_Puts (sAdcVal2, &Font_11x18, 1);

	SSD1306_UpdateScreen();
}


//void guiUpdate(struct State s){
//	SSD1306_Clear();
//    uint16_t target = s.encoderTick;
//    uint32_t targetav = heaterTargetAdcV();
//    if (s.mode == WARM){
//    	target = 200;
//    	targetav = heaterWarmTargetAdcV();
//    }
//    else if(s.mode == HEATING && s.highPower){
//    	target = s.encoderTick + 30;
//    	targetav = heaterHighTargetAdcV();
//    }
//	// show target temp at 1st line
//    utoa(target, stemp, 10);
//	SSD1306_GotoXY (0, 0);
//	SSD1306_Puts (stemp, &Font_11x18, 1);
//
//	utoa(targetav, sAdcVal, 10);
//	SSD1306_GotoXY (40, 0);
//	SSD1306_Puts (sAdcVal, &Font_11x18, 1);
//
//	switch(s.mode){
//		case HEATING: sMode[0] = 'H'; break;
//		case FORCED_HEATING: sMode[0] = 'F'; break;
//		case OFF: sMode[0] = 'O'; break;
//		case WARM: sMode[0] = 'W'; break;
//		case SETTING: sMode[0] = 'S'; break;
//	}
//
//	SSD1306_GotoXY (111, 0);
//	SSD1306_Puts (sMode, &Font_11x18, 1);
//
//	// show current temp on 2nd line
//	utoa(s.currentTemp, stemp, 10);
//	SSD1306_GotoXY (0, 20);
//	SSD1306_Puts (stemp, &Font_11x18, 1);
//
//	// show current currentAdcVal on 3rd line
//	utoa(s.currentAdcVal, sAdcVal, 10);
//	SSD1306_GotoXY (0, 40);
//	SSD1306_Puts (sAdcVal, &Font_11x18, 1);
//
//	SSD1306_UpdateScreen();
//}

//	SSD1306_GotoXY (0,0);
//	SSD1306_Puts ("NIZAR", &Font_11x18, 1);
//	SSD1306_GotoXY (0, 30);
//	SSD1306_Puts ("MOHIDEEN", &Font_11x18, 1);
//	SSD1306_UpdateScreen();
//	HAL_Delay (1000);
//
//	SSD1306_ScrollRight(0,7);
//	HAL_Delay(3000);
//	SSD1306_ScrollLeft(0,7);
//	HAL_Delay(3000);
//	SSD1306_Stopscroll();
//	SSD1306_Clear();
//	SSD1306_GotoXY (35,0);
//	SSD1306_Puts ("SCORE", &Font_11x18, 1);
