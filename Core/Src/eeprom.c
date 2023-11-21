#include "main.h"
//#include "stm32f1xx_hal_i2c.h"
#include "eeprom.h"


I2C_HandleTypeDef *hi2c11;
UART_HandleTypeDef *huart11;

uint8_t toWrite1[] = "English is a WestGermanic language of the IndoEuropean language";
uint8_t toWrite4[4] = "ABCD";
uint8_t toWrite8[8] = "ABCDefgh";
uint8_t toRead2[64];
uint8_t toWriteB[] = "H";
uint8_t toWriteO[] = "Ok";
uint8_t toRead1[1];
uint8_t toRead4[4];
uint8_t toRead8[8];


uint8_t message[15] = {'_'};
uint16_t encTickMemAddress = 0;
uint16_t MemAddSize = 1;
void cfgInit(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart){
	hi2c11 = hi2c;
	huart11 = huart;
}
void cfgSaveEncTick(  uint16_t encoderCt ){
	uint16_t data16[1];
	data16[0] = encoderCt;
//	memcpy(data, data16, 4);
	//data = (uint8_t*)encoderCt;
//	data[0] = (uint8_t)(encoderCt>>24);
//	data[1] = (uint8_t)(encoderCt>>16);
//	data[2] = (uint8_t)(encoderCt>>8);
//	data[3] = (uint8_t)(encoderCt);
	HAL_I2C_Mem_Write(hi2c11, EEPROM_ADDR, encTickMemAddress, 1, data16, 2, HAL_MAX_DELAY);
	HAL_Delay(100);
}
uint16_t cfgGetEncTick( ){
	uint16_t data16[1] = {0};
	HAL_I2C_Mem_Read(hi2c11, EEPROM_ADDR, encTickMemAddress, 1, data16, 2, HAL_MAX_DELAY);
	//memcpy(&rt, data, sizeof(data));
	return data16[0];

}
// pageNo - 0 based; pdata size <=8
HAL_StatusTypeDef writePage(uint16_t pageNo, uint8_t *pdata){
	return HAL_I2C_Mem_Write(hi2c11, EEPROM_ADDR, pageNo*8, 1, pdata, 8, HAL_MAX_DELAY);
}
// pageNo - 0 based; pdata size <=8
HAL_StatusTypeDef readPage(uint16_t pageNo, uint8_t *pdata){
	return HAL_I2C_Mem_Read(hi2c11, EEPROM_ADDR, pageNo*8, 1, pdata, 8, HAL_MAX_DELAY);
}
// test write/read all  pages, for 34w02, there are 32 pages and each page has 8 bytes. Writing all bytes with 0 or 255 and check read values
void testPages(){
	 HAL_UART_Transmit(huart11, "testing\r\n",10, 1000);

	 uint8_t zv = 0;
	 uint8_t fv = 255;
	 uint8_t wv = zv;
    uint8_t WA[8];
   for(int j = 0; j< 8; j++){
	   WA[j] = wv;
   }
   uint8_t r[8];
   HAL_StatusTypeDef rc ;
   for (int i = 0; i < 32; i++ ){
	  // HAL_UART_Transmit(huart11, "write\r\n",10, 1000);
	     rc = writePage(i, WA);
	   //  HAL_UART_Transmit(huart11, "Af write\r\n",10, 1000);
	   //HAL_UART_Transmit(huart11, message, sizeof(message), 1000);
	   HAL_Delay(100);
		sprintf((char*)message, "Wp=%d Er=%d\r\n", i, rc);
		HAL_UART_Transmit(huart11, message, sizeof(message), 1000);

   }
   int err = 0;
   for (int i = 0; i < 32; i++ ){
	  // HAL_UART_Transmit(huart11, "read\r\n",8, 1000);
	    rc = readPage(i, r);
		sprintf((char*)message, "\r\nRp=%d Er=%d\r\n", i, rc);
		HAL_UART_Transmit(huart11, message, sizeof(message), 1000);
		for(int j =0; j< 8; j++){
			sprintf((char*)message, " %00d ", r[j] );
			HAL_UART_Transmit(huart11, message, 5, 1000);

	 	 if (r[j] != wv){
				sprintf((char*)message, "ERRc=%d r=%d w=%d", j, r[j], WA[j]);
						HAL_UART_Transmit(huart11, message, sizeof(message), 1000);
            err++;
		  }
		}
   }
   if (err > 0){
	   sprintf((char*)message, "\r\nERR=%d  ", err);

	}else{
		sprintf((char*)message, "\r\nNo ERR");
	}
	HAL_UART_Transmit(huart11, message, 8, 1000); ///show total # of read error
	sprintf((char*)message, "\r\nwrite %d", wv);
	HAL_UART_Transmit(huart11, message, 12, 1000); ///show the byte value written to

}
/*
 * save data to eeprom and retrive it to set encoder tick
 */
void testCfg(){
	uint8_t message[15] = {'\0'};
	uint16_t data16[1] = {200};
//	cfgSaveEncTick( 155);

	uint16_t data16r[1] = {0};

	data16r[0] = cfgGetEncTick( );

	HAL_UART_Transmit(&huart11, "\r\n", 2, 1000);
	sprintf((char*)message, "u %u\r\n", data16r[0]);
	HAL_UART_Transmit(&huart11, message, 10, 1000);

	setEncoderTick(data16r[0]);
}
void testEEPROM( ){
	uint8_t message[10] = {'\0'};
	strcpy((char*)message, "testE\r\n");
	HAL_I2C_Mem_Write(hi2c11, EEPROM_ADDR, 0, 1, toWrite1, sizeof(toWrite1), HAL_MAX_DELAY);
	HAL_Delay(100);
	HAL_UART_Transmit(huart11, message, sizeof(message), 1000);
	uint8_t adr[1];
	uint8_t d[1];
	uint8_t i = 0;
	while(i < 8){
		uint8_t p = 8*i;
//		adr[0] = i;
	 	//readByte(EEPROM_ADDR, i, toRead8, sizeof(toRead8));

		//readBytes(i, toRead1);
		HAL_UART_Transmit(&huart11, "\r\npageAdr=", 2, 1000);
			HAL_UART_Transmit(&huart11, &p, sizeof(&p), 1000);

	HAL_I2C_Mem_Read(&hi2c11, EEPROM_ADDR, p, 1, toRead8, sizeof(toRead8), HAL_MAX_DELAY);

	HAL_UART_Transmit(&huart11, "\r\n", 2, 1000);
	HAL_UART_Transmit(&huart11, toRead8, sizeof(toRead8), 1000);

//	HAL_I2C_Mem_Read(&hi2c11, EEPROM_ADDR, 4, 1, toRead4, sizeof(toRead4), HAL_MAX_DELAY);
//
//	HAL_UART_Transmit(&huart11, "\r\n", 2, 1000);
//	HAL_UART_Transmit(&huart11, toRead4, sizeof(toRead4), 1000);


			i++;
	}
	strcpy((char*)message," end\r\n");
	HAL_UART_Transmit(&huart11, message, sizeof(message), 1000);

}
//
//void readByte(uint8_t devAddr, uint8_t byteAddr, uint8_t* read, uint8_t readSize){
//	uint8_t buf[20];
//	//uint8_t read[readSize];
//
//	buf[0] = byteAddr;
//	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c11, devAddr, buf, 1, HAL_MAX_DELAY);
//	if ( ret != HAL_OK ) {
//	      strcpy((char*)buf, "Error Tx\r\n");
//	} else {
//		// Read 2 bytes from the temperature register
//		ret = HAL_I2C_Master_Receive(&hi2c11, devAddr, read, readSize, HAL_MAX_DELAY);
//		if ( ret != HAL_OK ) {
//			sprintf((char*)buf, "Er%d Rx\r\n", ret);
//		} else {
//			uint8_t r = buf[0];
//			sprintf((char*)buf, "adr=%d,v=%s\r\n", byteAddr, read);
//		}
//	}
//
//	HAL_UART_Transmit(&huart11, buf, sizeof(buf), 1000);
//	return read;
//
//}
//
//void writeBytes(uint16_t MemAddress, uint8_t *data){
//	uint8_t message[10] = {'\0'};
//	HAL_StatusTypeDef ws = HAL_I2C_Mem_Write(&hi2c11, EEPROM_ADDR, MemAddress, 1, data, sizeof(data), 2000);
//	if (ws != HAL_OK){
//		sprintf(message, "ws err %d\n\r",  ws );
//		HAL_UART_Transmit(&huart11, message, sizeof(message), 1000);
//	}
//}
//void readBytes(uint16_t MemAddress, uint8_t *toRead4){
//	uint8_t message[10] = {'\0'};
//	HAL_StatusTypeDef ws = HAL_I2C_Mem_Read(&hi2c11, EEPROM_ADDR, MemAddress, 1, toRead4, sizeof(toRead4), 2000);
//	if (ws != HAL_OK){
//		sprintf(message, "rs err %d\n\r",  ws );
//		HAL_UART_Transmit(&huart11, message, sizeof(message), 1000);
//	}else{
//		sprintf((char*)message, "%d,  %d\r\n", MemAddress, toRead4);
//		HAL_UART_Transmit(&huart11, (char*)message, sizeof(message), 1000);
//	}
//}
