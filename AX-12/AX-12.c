/*
 * AX-12.c
 *
 *  Created on: 13.05.2018
 *      Author: Pawel
 */
void WriteData(uint8_t ID, uint8_t length, uint8_t instruction, uint8_t *parameters){
	uint8_t *msg;
	msg[0]=0xFF;
	msg[1]=0xFF;
	msg[2]=ID;
	msg[3]=length;
	for (int i=0; i<length-2;i++){
		msg[i+4]=parameters[i];
	}
	msg[length+3]=~(msg[2]+msg[3]+msg[4]+msg[5]+msg[6]+msg[7]+msg[8]+msg[9]);
}


