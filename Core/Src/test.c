#include "test.h"
#include "heater.h"

I2C_HandleTypeDef *hi2c12;
UART_HandleTypeDef *huart12;

void testInit(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart){
	hi2c12 = hi2c;
	huart12 = huart;
}

void sendMessage(char* msg, uint16_t len){
	HAL_UART_Transmit(huart12, msg , len, 1000);
}
// test heater v2temp() temp2V()
void testTemp2V(){
	char message[15] = {' '};
	uint16_t tempbase = 150;
	uint32_t adcValBase = 0;
	uint32_t adcVal = 150;
	uint8_t count = 7;
	uint32_t inc = 50;

	//char message[20] = {' '};sprintf((char*)message, "Wp=%d Er=%d\r\n", i, rc);

	for (int i = 0; i < 11; i++){
		uint16_t temp = tempbase + i*30;
		adcVal = temp2V(temp);

		sprintf((char*)message, "\r\nt=%d v=%d", temp, adcVal);
		sendMessage(message, 14);
		//HAL_Delay(100);
	}
	inc = 200;
	adcVal = 500;
	for (int i = -1; i < 20; i++){
		adcVal = adcValBase +  i*inc;
		uint16_t t = v2temp(adcVal);
		sprintf((char*)message, "\r\nv=%d t=%d", adcVal, t);
		sendMessage(message, 14);
		//HAL_Delay(100);
	}



}

