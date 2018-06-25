#include "stm32f4xx_hal.h"

#define DIRECTION_PIN GPIO_PIN_9
#define DIRECTION_PORT GPIOA
#define SEND GPIO_PIN_SET
#define RECEIVE GPIO_PIN_RESET

uint8_t checkSum(uint8_t *data, int size){
	uint8_t sum = 0x00;
	for (int i = 2; i<size; i++){
		sum += data[i];
	}
	return ~(sum);
}

//funkcja wysy³aj¹ca jeden bajt do pamiêci o podanym adresie w odpowiednim dynamixelu
//w trybie blokuj¹cym
void sendData(uint8_t byte, uint8_t mem_addr, uint8_t ID, UART_HandleTypeDef *huart){
	HAL_GPIO_WritePin(DIRECTION_PORT, DIRECTION_PIN, SEND);
	uint8_t data[8];
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = ID;
	data[3] = 0x04;
	data[4] = 0x03;
	data[5] = mem_addr;
	data[6] = byte;
	data[7] = checkSum(data, 7);
	HAL_UART_Transmit(huart, data, 8, 100);
	HAL_GPIO_WritePin(DIRECTION_PORT, DIRECTION_PIN, RECEIVE);
}

//funkcja wysy³aj¹ca dwa bajty do pamiêci o podanym adresie w odpowiednim dynamixelu
//w trybie blokuj¹cym
void sendData2(uint8_t byteLow, uint8_t byteHigh, uint8_t mem_addr, uint8_t ID, UART_HandleTypeDef *huart){
	HAL_GPIO_WritePin(DIRECTION_PORT, DIRECTION_PIN, SEND);
	uint8_t data[9];
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = ID;
	data[3] = 0x05;
	data[4] = 0x03;
	data[5] = mem_addr;
	data[6] = byteLow;
	data[7] = byteHigh;
	data[8] = checkSum(data, 8);
	HAL_UART_Transmit(huart, data, 8, 100);
	HAL_GPIO_WritePin(DIRECTION_PORT, DIRECTION_PIN, RECEIVE);
}



void ledOn(uint8_t ID, UART_HandleTypeDef *huart){
	sendData(0x01, 0x18, ID, huart);
}

void ledOff(uint8_t ID, UART_HandleTypeDef *huart){
	sendData(0x00, 0x18, ID, huart);
}

void setGoalPosition(uint16_t value, uint8_t ID, , UART_HandleTypeDef *huart){
	byteLow = value & 0xFF;
	value = value >> 8;
	byteHigh = value & 0xFF;
	sendData2(byteLow, byteHigh, 0x1E, ID, huart);
}

void setMovingSpeed(uint16_t value, uint8_t ID, , UART_HandleTypeDef *huart){
	byteLow = value & 0xFF;
	value = value >> 8;
	byteHigh = value & 0xFF;
	sendData2(byteLow, byteHigh, 0x20, ID, huart);
}

void setCWAngleLimit(uint16_t value, uint8_t ID, , UART_HandleTypeDef *huart){
	byteLow = value & 0xFF;
	value = value >> 8;
	byteHigh = value & 0xFF;
	sendData2(byteLow, byteHigh, 0x06, ID, huart);
}

void setCCWAngleLimit(uint16_t value, uint8_t ID, , UART_HandleTypeDef *huart){
	byteLow = value & 0xFF;
	value = value >> 8;
	byteHigh = value & 0xFF;
	sendData2(byteLow, byteHigh, 0x08, ID, huart);
}

//funckja zwracajaca zawartosc pamieci dostepnej pod podanym adresem w danym dynamixelu
int receiveData(uint8_t *read_value, uint8_t mem_addr, uint8_t ID){
	readValue[0] = 0x00;//inicjalizacja poczatkowego bajtu w celu sprawdzenia poprawnosci odczytu
						//pewnie mozna na podstawie wartosci zwracanej przez funkcje hal_uart_receive
						//ale trudno, dokumentacja jest za dluga :(
	HAL_GPIO_WritePin(DIRECTION_PORT, DIRECTION_PIN, SEND);
	uint8_t request[8];
	request[0] = 0xFF;
	request[1] = 0xFF;
	request[2] = ID;
	request[3] = 0x04;
	request[4] = 0x02;
	request[5] = mem_addr;
	request[6] = 0x01;
	request[7] = checkSum(request, 7);
	HAL_UART_Transmit(huart, request, 8, 100);
	HAL_GPIO_WritePin(DIRECTION_PORT, DIRECTION_PIN, RECEIVE);
	HAL_UART_Receive(huart, read_value, 7);
	if(read_value[0]==0xFF) return 0;
	else return -1;
}

//funckja zwracajaca zawartosc dwoch bajtow pod podanym adresem w danym dynamixelu
int receiveData2(uint8_t *read_value, uint8_t mem_addr, uint8_t ID){
	readValue[0] = 0x00;
	HAL_GPIO_WritePin(DIRECTION_PORT, DIRECTION_PIN, SEND);
	uint8_t request[8];
	request[0] = 0xFF;
	request[1] = 0xFF;
	request[2] = ID;
	request[3] = 0x04;
	request[4] = 0x02;
	request[5] = mem_addr;
	request[6] = 0x02;
	request[7] = checkSum(request, 7);
	HAL_UART_Transmit(huart, request, 8, 100);
	HAL_GPIO_WritePin(DIRECTION_PORT, DIRECTION_PIN, RECEIVE);
	HAL_UART_Receive(huart, read_value, 8);
	if(read_value[0]==0xFF) return 0;
	else return -1;
}
