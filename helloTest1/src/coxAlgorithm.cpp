/*
 * coxAlgorithm.cpp
 *
 *  Created on: Apr 18, 2023
 *      Author: Erik
 */


#include <stdio.h>
#include <stdlib.h>
#include "Eigen/Dense"
#include <iostream>
#include <fstream>
#include <math.h>

class coxMap{
	public:
	float x;
	float y;
	double dist;
	double ang;
	double sensorPose[] = {1000000,20000000,3000000};


	void setup(){
		for (int i=0; i<line.rows(); i++){
				ui[i] << line(i,0) - line(i,2), line(i,1) - line(i,3);
				ui[i] = rotMat*ui[i]/ui[i].norm();
				Li[i] << line(i,0), line(i,1);
				ri[i] = ui[i].dot(Li[i]);
			}
	}

	Eigen::Matrix<float, 4, 4> line {
		  {559, -1140, 2856, 910},
		  {2856, 910, 1336, 2670},
		  {1336, 2670, -1004, 610},
		  {-1004, 610, 559, -1140},
	};
	Eigen::Matrix<float, 2, 2> rotMat{
		{0, -1},
		{1, 0},
	};
	Eigen::Vector2<float> Li[4];
	Eigen::Vector2<float> ui[4];
	float ri[4];

	void gather_data(){
		dist = 0;
		ang = 0;
	}

	void loop(int max_iterations){
		for (int i = 0; i++; i < max_iterations ){
			//sensor to Cartesian coordinates
			x = dist*cos(ang);
			y = dist*sin(ang);

			//sensor to Robot coordinates
			Eigen::Matrix<double, 3, 3> rotMatRobot {
					  {cos(sensorPose[3]), -1*sin(sensorPose[3]), sensorPose[1]},
					  {sin(sensorPose[3]), cos(sensorPose[3]), sensorPose[2]},
					  {0, 0, 1},
				};
			Eigen::Matrix<float, 1, 3> test{
				{x, y, 1},
			};

			Eigen::Matrix<double, 3, 3> Xs;

			Xs = rotMatRobot*test;
		}
	}

};

int main(void){
	coxMap map;
	map.setup();
	std::cout << "Here are the vectors u1:\n" << map.ri[0]  << std::endl;





}
