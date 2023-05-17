/*
 * lidar.h
 *
 *  Created on: May 4, 2023
 *      Author: Erik
 */
#ifndef LIDAR_H_
#define LIDAR_H_

#include <Eigen/Dense>
//#include "lidar.cpp"

extern bool needData;
extern int Array_full_flag;
extern Eigen::Matrix<double, 1, 500> DIS;
extern Eigen::Matrix<double, 1, 500> ang;


void lidarRead(void);




#endif /* LIDAR_H_ */
