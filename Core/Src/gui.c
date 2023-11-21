#include "gui.h"
#include "fonts.h"
#include "ssd1306.h"

char stemp[5];
char sAdcVal[4];

void guiInit( ){
	 SSD1306_Init();
}
void guiUpdate(struct State s){
	SSD1306_Clear();

	// show target temp at 1st line
    utoa(s.encoderTick, stemp, 10);
	SSD1306_GotoXY (0, 0);
	SSD1306_Puts (stemp, &Font_11x18, 1);

	utoa(heaterTargetAdcV(), sAdcVal, 10);
	SSD1306_GotoXY (48, 0);
	SSD1306_Puts (sAdcVal, &Font_11x18, 1);

	// show current temp on 2nd line
	utoa(s.currentTemp, stemp, 10);
	SSD1306_GotoXY (0, 20);
	SSD1306_Puts (stemp, &Font_11x18, 1);

	// show current currentAdcVal on 3rd line
	utoa(s.currentAdcVal, sAdcVal, 10);
	SSD1306_GotoXY (0, 40);
	SSD1306_Puts (sAdcVal, &Font_11x18, 1);

	SSD1306_UpdateScreen();
}

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
