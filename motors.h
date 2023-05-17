/*
 * motors.h
 *
 *  Created on: May 4, 2023
 *      Author: Erik
 */

#ifndef MOTORS_H_
#define MOTORS_H_

#include <iostream>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include "spi_com.h"

extern MotorDataType MotorData;

void motorMain(void);




#endif /* MOTORS_H_ */

