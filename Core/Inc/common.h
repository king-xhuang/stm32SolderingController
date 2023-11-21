#ifndef COMMON_H
#define COMMON_H 103

/* C++ detection */
#ifdef __cplusplus
extern C {
#endif

#include <stdbool.h>
#include <stdint.h>
// common variable, constances shared by all modulea
struct State{
	uint16_t preSetTemp;  // preset temp loaded from eeprom
	uint16_t currentTemp; // calculated value from currentAdcVal
	uint16_t encoderTick; // latest encoder tick

	//uint32_t preSetAdcVal;
	uint32_t currentAdcVal; // latest acd value

	enum Mode {HEATING,FORCED_HEATING,WARM,SETTING,IDEL,OFF} mode;

};

#ifdef __cplusplus
}
#endif

#endif
