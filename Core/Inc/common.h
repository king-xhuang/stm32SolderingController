#ifndef COMMON_H
#define COMMON_H 103

/* C++ detection */
#ifdef __cplusplus
extern C {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "buzzerTone.h"
//#include "stm32f1xx_hal.h"
// common variable, constances shared by all modulea

// choose one of the following soldering tips
#define	C245  = 0

//#define	C210
//end of choose soldering tips
enum Mode {OFF = 0,
			WARM,
			HEATING,
			FORCED_HEATING,
			SETTING};

struct State{
	uint16_t preSetTemp;  // preset temp loaded from eeprom
	uint16_t currentTemp; // calculated value from currentAdcVal
	volatile 	uint16_t encoderTick; // latest encoder tick

	//uint32_t preSetAdcVal;
	volatile  uint32_t currentAdcVal; // latest acd value
	volatile bool highPower;
    volatile uint16_t highPowerTemp;


	enum Mode mode;

};
//extern volatile uint32_t warmEndTime;
//extern volatile uint32_t heatingEndTime; //  in millis
//extern volatile struct State state;

void stateSetMode(enum Mode mode);
bool stateModeIs(enum Mode mode);
void checkStateTimeout( uint32_t currentTick );
struct State* getState();
void checkBeepFlag();
bool save2Eeprom();
#ifdef __cplusplus
}
#endif

#endif
