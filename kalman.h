/*
 * kalman.h
 *
 *  Created on: May 4, 2023
 *      Author: Erik
 */
#ifndef KALMAN_H_
#define KALMAN_H_

#include <Eigen/Dense>


extern Eigen::Vector3d position;
extern Eigen::Matrix3d covariance;

void kalmanMain(void);

double modulosAngle(double vinkel);


#endif 
