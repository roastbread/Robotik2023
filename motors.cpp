#include <stdlib.h>
#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include "spi_com.h"

MotorDataType MotorData;

static const int SPI_Channel = 1;

void motorMain(void){

	wiringPiSetup(); 
	wiringPiSPISetup(SPI_Channel, 1000000);
	while(1){
		delay(50);
		Send_Read_Motor_Data(&MotorData);
	}
}