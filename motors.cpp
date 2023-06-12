#include <stdlib.h>
#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <chrono>
#include "spi_com.h"

MotorDataType MotorData;

static const int SPI_Channel = 1;

void motorMain(void){

	wiringPiSetup(); 
	wiringPiSPISetup(SPI_Channel, 1000000);
	while(1){
		delay(40);
		Send_Read_Motor_Data(&MotorData);
		//MotorData.Timestamp = std::chrono::duration_cast,std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	}
}