/*
 * coxRemade.cpp
 *
 *  Created on: Apr 27, 2023
 *      Author: Erik
 */
#include <stdio.h>
#include <stdlib.h>
#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include "lidar.h"
#include <fstream>
#include "kalman.h"


using namespace std;
using namespace Eigen;
using namespace chrono;

Eigen::Vector3d posFix;
Eigen::MatrixXd covarianceFix (3,3);
bool coxDone = false;

extern int Array_full_flag;
extern bool needData;
extern Eigen::Matrix<double, 1, 500> DIS;
extern Eigen::Matrix<double, 1, 500> ang;


const int num_meas = 500;



double dists[500];
double angs[500];
Eigen::MatrixXd x;
Eigen::MatrixXd y;

double sensorPose[3] = {10, 0, 0};	

Eigen::Matrix<double, 4, 4> line {
            {0, 0, 0, 3635},
            {0, 3635, 2434, 3634},
            {2434, 3634, 2430, 0},
            {2430, 0, 0, 0},
    };
/*
Eigen::Matrix<double, 4, 4> line {
            {0, 0, 0, 1700},
            {0, 1700, 1400, 1700},
            {1400, 1700, 1400, 0},
            {1400, 0, 0, 0},
    };
*/

Eigen::Matrix2d rotMat{
	{0,-1}, 
	 {1, 0},
};

Eigen::Vector2d Li[4];
Eigen::Vector2d ui[4];
double ri[4];


void setup(){
	for (int i=0; i<line.rows(); i++){
		ui[i] << line(i,2) - line(i,0), line(i,3) - line(i,1);
		//cout << ui[i] << endl;
		ui[i] = rotMat*ui[i]/ui[i].norm();
		//cout << ui[i] << endl;
		Li[i] << line(i,0), line(i,1);
		ri[i] = ui[i].dot(Li[i]);
	}
};


void gather_data(){
	std::string line;
	std::string quality;
	std::string angle;
	std::string distance;
	std::fstream data;
	data.open("testfile.txt");
	data.seekg(-12*410, std::ios_base::end);
	for (int i=0; i<=400;i++){
		std::getline(data, line);
		if(line.length()>4){
			std::size_t pos1 = line.find_first_of(' ');
			quality = line.substr(0,pos1);
			std::size_t pos2 = line.find_first_of(' ', pos1+1);
			angle = line.substr(pos1+1,pos2-2);
			distance = line.substr(pos2+1);
			//std::cout << quality << "|" << angle << "|" << distance << "|" << i << std::endl;
			angs[i-1] = std::stod(angle);
			dists[i-1] = std::stod(distance);

		}
	}
	//cout << sizeof(angs)/8 << endl;
	ang = Eigen::Map<MatrixXd>(angs, 1, 500);
	DIS = Eigen::Map<MatrixXd>(dists, 1, 500);

	//std::remove("testfile.txt");
	data.close();
	//std::ofstream empty;
	//empty.open("textfile.txt");
	//std::rename("textfile.txt","testfile.txt");
	//empty.close();
}

void loop(int maxIterations, float prevX, float prevY, float prevA){
	setup();
	float posX = position(0);
	float posY = position(1);
	float posA = position(2);
	int iter = 0;
	double ddx = 0;
	double ddy = 0;
	double dda = 0;

	double sigX = 0;
	double sigY = 0;
	double sigA = 0;

	while(iter < maxIterations){
		iter++;
	
		Eigen::Matrix<double, 5, num_meas> worldData;	//index, x, y, line, distance
		worldData = Eigen::MatrixXd::Zero(5, num_meas);
	
		Eigen::MatrixXd prunedData(5, num_meas/2);	//index, x, y, line, distance to line
		prunedData = Eigen::MatrixXd::Zero(5, num_meas/2);
	
		//step 1:	Translate and rotate datapoints
		//sensor to cartesian

		//cout << DIS << "DIS" << endl;

		x = DIS.array() * cos(ang.array()*M_PI/180); //
		y = DIS.array() * sin(ang.array()*M_PI/180); //
		
		//cout << x << "cartesian X" << endl;
		//cout << y << "cartesian Y" << endl;

		Eigen::MatrixXd xySensor(3,x.cols());
		xySensor.row(0) = x;
		xySensor.row(1) = y;
		xySensor.row(2).setOnes();
		//sensor to robot
		Eigen::Matrix3d rotMatRobot {
			{cos(sensorPose[2]), -1*sin(sensorPose[2]), sensorPose[0]},
			{sin(sensorPose[2]), cos(sensorPose[2]), sensorPose[1]},
			{0, 0, 1}
		};

	
		Eigen::MatrixXd xyRobot = rotMatRobot*xySensor;
		Eigen::Matrix<double, 3, 1> b;
		b << 0, 0, 0;
		Eigen::Matrix3d rotMatWorld {								//change
				{cos(prevA+dda), -1*sin(prevA+dda), prevX+ddx},
				{sin(prevA+dda), cos(prevA+dda), prevY+ddy},
				{0, 0, 1},
		};

		//cout << "xyRobotX " << xyRobot.row(0) << endl;
		//cout << "xyRobotY " << xyRobot.row(1) << endl;
	
		Eigen::MatrixXd xyWorld = rotMatWorld*xyRobot;
		//cout << "xyWorld X " << xyWorld.row(0) << endl;
		//cout << "xyWorld Y " << xyWorld.row(1) << endl;
		worldData.row(1) = xyWorld.row(0);
		worldData.row(2) = xyWorld.row(1);
		for(int i = 0; i < num_meas; i++){
			worldData(0, i) = i;
			worldData(3, i) = 0;
			worldData(4, i) = 20000;
		}
		// find target points
		double distPointLine;
		for(int i = 0; i < num_meas; i++){
			Eigen::Vector2d pointTemp(worldData(1, i), worldData(2, i));
			for(int j = 0; j < 4; j++){
				distPointLine = ri[j] - ui[j].dot(pointTemp);
				if(abs(distPointLine) <= abs(worldData(4, i))){
					worldData(3, i) = j;
					worldData(4, i) = distPointLine;
				}
			}
		}

		Eigen::Array<double, num_meas, 1> tempDist = worldData.row(4).transpose();
		tempDist = tempDist.abs();
		sort(tempDist.begin(), tempDist.end());
		int matrixCounter = 0;
		int medianIdx = tempDist.size()/2;
		double outlierTh = (tempDist[medianIdx]+tempDist[medianIdx-1])/2;
		//cout << "threshold " << outlierTh << endl;

		for(int i=0; i < num_meas; i++){
			if(abs(worldData(4, i))<=outlierTh){
				prunedData.col(matrixCounter) = worldData.col(i);
				matrixCounter++;
				if (matrixCounter == num_meas/2){
					break;
				}
			}
			else{
				//cout << "rejected " << worldData.col(i) << endl;
			}
			if(i == num_meas-1 && matrixCounter < num_meas/2){
				for(int j = matrixCounter; j < prunedData.cols(); j++){
					//prunedData.col(j) = prunedData.col(j-1);
			}
		}
	}
	
//cout << "worldData " << worldData << endl;
//cout << "prunedData " << prunedData << endl;
	Eigen::Vector2d robotPos(prevX+ddx, prevY+ddy);
	// step 3 linear equation system
	Eigen::MatrixXd xi1 (1, prunedData.cols());
	Eigen::MatrixXd xi2 (1, prunedData.cols());
	Eigen::MatrixXd xi3 (1, prunedData.cols());
	for(int i = 0; i < prunedData.cols(); i++){
		xi1(i) = ui[(int)prunedData(3, i)](0);
		xi2(i) = ui[(int)prunedData(3, i)](1);
		Eigen::Vector2d unitVxy(xi1(i),xi2(i));
		Eigen::Vector2d pointXY(prunedData(1,i), prunedData(2,i));
		xi3(i) = unitVxy.transpose() * rotMat * (pointXY-robotPos); //abs?
	}

/*
	Eigen::MatrixXd A(3, prunedData.cols());
	A.row(0) = xi1.row(0);
	A.row(1) = xi2.row(0);
	A.row(2) = xi3.row(0);
	A.transposeInPlace();
*/
	Eigen::MatrixXd A(prunedData.cols(), 3);
	A.col(0) = xi1.row(0).transpose();
	A.col(1) = xi2.row(0).transpose();
	A.col(2) = xi3.row(0).transpose();

	//Eigen::MatrixXd AtA = A.transpose() * A;
	//Eigen::MatrixXd AtYi = A.transpose() * prunedData.row(4).transpose();
	//Vector3d b(AtA.inverse() * AtYi);
	b = (A.transpose() * A).inverse() * A.transpose() * prunedData.row(4).transpose();
//cout << 'b' << b << endl;

	double S2 = (prunedData.row(4).transpose() - A * b).dot(prunedData.row(4).transpose() - A * b) / (A.rows() - 4);
//cout << "S2 " << S2 << endl;
	Eigen::MatrixXd C (3,3);
	C << S2 * (A.transpose() * A).inverse();
	sigX = C(0,0);
	sigY = C(1,1);
	sigA = C(2,2);

	// Step 4
	ddx += b(0);
	ddy += b(1);
	dda = modulosAngle(dda + b(2));
	//dda = dda (2*M_PI);

	xi1.setZero();
	xi2.setZero();
	xi3.setZero();
	A.setZero();
	
	// Step 5
	if (sqrt(pow(b(0), 2) + pow(b(1), 2)) < 5 && abs(b(2)) < 0.1 * M_PI / 180) {
		covarianceFix = C;
		posFix(0) = ddx;
		posFix(1) = ddy;
		posFix(2) = dda;
		posX += ddx;
		posY += ddy;
		posA = modulosAngle(posA + dda);
		Array_full_flag = 0;
//std::cout << "Position Fix: X=" << posFix(0) << " Y=" << posFix(1) << " A=" << posFix(2) << std::endl;
//std::cout << "Position Cox: X=" << posX << " Y=" << posY << " A=" << posA << std::endl;
//std::cout << "Uncertainty Cox: " << covarianceFix << std::endl;
//cout << "coxDone" << endl;
		coxDone = true;
		return;
   }




	}

}


int coxMain(void){
	//coxMap map;
	
	//std::cout << "Here are the vectors u1:\n" << map.ri[0]  << std::endl;
	while(1){
		while(Array_full_flag == 0 || !needData){}
		needData = false;
		//map.gather_data();
		loop(20, 1200, 100, M_PI/2);
		Array_full_flag = 0;
		coxDone = true;
		//std::cout << "Position Fix: X=" << posFix(0) << " Y=" << posFix(1) << " A=" << posFix(2) << std::endl;
		//std::cout << "Position: X=" << map.posX << " Y=" << map.posY << " A=" << map.posA << std::endl;
		//std::cout << "Uncertainty Cox: " << covarianceFix << std::endl;
	}
	cout << "Finished" << endl;
};
