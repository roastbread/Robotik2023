/*
 * coxRemade.h
 *
 *  Created on: May 5, 2023
 *      Author: Erik
 */
#ifndef COXREMADE_H_
#define COXREMADE_H_

#include <stdio.h>
#include <stdlib.h>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
//#include <math.h>
#include <string>
#include <time.h>
#include <unistd.h>
#include "lidar.h"
//#include "coxRemade.cpp"



extern Eigen::Vector3d posFix;
extern Eigen::MatrixXd covariance;
extern bool coxDone;

int coxMain(void);



#endif /* COXREMADE_H_ */
