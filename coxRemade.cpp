/*
 * coxRemade.cpp
 *
 *  Created on: Apr 27, 2023
 *      Author: Erik
 */
#include <stdio.h>
#include <stdlib.h>
#include "Eigen/Dense"
#include <iostream>
#include <fstream>
//#include <math.h>
#include <string>

using namespace std;
using namespace Eigen;

class coxMap{
	public:
	double dists[400];
	double angs[400];
	//double sensorPose[] = {1000000,20000000,3000000};
	MatrixXd x;
	MatrixXd y;

	MatrixXd DIS;
	MatrixXd ang;

	//double sensorPose[3] = {660 ,0,-90*M_PI/180};
	double sensorPose[3] = {0, 0, 0};
	float prevX = 0;
	float prevY = 0;
	float prevA = 0;

	double ddx = 0;
	double ddy = 0;
	double dda = 0;
	double sigX = 0;
	double sigY = 0;
	double sigA = 0;

	 Matrix<double, 4, 4> line {
	            {0, 0, 0, 3635},
	            {0, 3635, 2434, 3634},
	            {2434, 3634, 2430, 0},
	            {2430, 0, 0, 0},
	    };
/*
	MatrixXd line {
		  {559, 1140, 2856, -910},
		  {2856, -910, 1336, -2670},
		  {1336, -2670, -1004, -610},
		  {-1004, -610, 559, 1140},
	};*/
	MatrixXd rotMat{
		{0, -1},
		{1, 0},
	};
	Vector2d Li[4];
	Vector2d ui[4];
	double ri[4];


	void setup(){
		for (int i=0; i<line.rows(); i++){
				ui[i] << line(i,0) - line(i,2), line(i,1) - line(i,3);
				ui[i] = rotMat*ui[i]/ui[i].norm();
				Li[i] << line(i,0), line(i,1);
				ri[i] = ui[i].dot(Li[i]);
			}
	}

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
		ang = Eigen::Map<MatrixXd>(angs, 1, 400);
		DIS = Eigen::Map<MatrixXd>(dists, 1, 400);

		//std::remove("testfile.txt");
		data.close();
		//std::ofstream empty;
		//empty.open("textfile.txt");
		//std::rename("textfile.txt","testfile.txt");
		//empty.close();
	}

	void loop(int maxIterations){
		int iter = 0;
		while(iter < maxIterations){
		iter++;
		Matrix<double, 5, 400> worldData;	//index, x, y, line, distance
		worldData = MatrixXd::Zero(5, 400);
		Matrix<double, 5, 200> prunedData;	//index, x, y, line, distance to line
		prunedData = MatrixXd::Zero(5, 200);
		//step 1:	Translate and rotate datapoints
		//sensor to cartesian
		x = DIS.array() * cos(ang.array()*M_PI/180);
		y = DIS.array() * sin(ang.array()*M_PI/180);

        MatrixXd xySensor(3,x.cols());
        xySensor.row(0) = x;
        xySensor.row(1) = y;
        xySensor.row(2).setOnes();

		//sensor to robot
		Matrix3d rotMatRobot {
			{cos(sensorPose[2]), -1*sin(sensorPose[2]), sensorPose[0]},
			{sin(sensorPose[2]), cos(sensorPose[2]), sensorPose[1]},
			{0, 0, 1},
		};

        MatrixXd xyRobot = rotMatRobot*xySensor;
        Matrix<double, 3, 1> b;
        b << 0, 0, 0;

        Matrix3d rotMatWorld {								//change
			{cos(prevA+dda), -1*sin(prevA+dda), prevX+ddx},
			{sin(prevA+dda), cos(prevA+dda), prevY+ddy},
			{0, 0, 1},
        };

        MatrixXd xyWorld = rotMatWorld*xyRobot;

        worldData.row(1) = xyWorld.row(0);
        worldData.row(2) = xyWorld.row(1);
        for(int i = 0; i < 400; i++){
        	worldData(0, i) = i;
        	worldData(3, i) = 0;
        	worldData(4, i) = 20000;
        }

        // find target points
        double distPointLine;
        for(int i = 0; i < 400; i++){
        	Vector2d pointTemp(worldData(1, i), worldData(2, i));
        	for(int j = 0; j < 4; j++){
        		distPointLine = ri[j] - ui[j].dot(pointTemp);

        		if(abs(distPointLine) <= abs(worldData(4, i))){
        			worldData(3, i) = j;
        			worldData(4, i) = distPointLine;
        		}
        	}
        }



		Array<double, worldData.cols(), 1> tempDist = worldData.row(4).transpose();
		tempDist = tempDist.abs();
		sort(tempDist.begin(), tempDist.end());
		int matrixCounter = 0;
		int medianIdx = tempDist.size()/2;
		double outlierTh = (tempDist[medianIdx]+tempDist[medianIdx-1])/2;

		for(int i=0; i < 400; i++){
			if(abs(worldData(4, i))<outlierTh){
				prunedData.col(matrixCounter) = worldData.col(i);
				matrixCounter++;
			}
			if(i == 399 && matrixCounter < 200){
				for(int j = matrixCounter; j < prunedData.cols(); j++){
					prunedData.col(j) = prunedData.col(j-1);
				}
			}
		}

		Vector2d robotPos(prevX+ddx, prevY+ddy);
		// step 3 linear equation system
		MatrixXd xi1 (1, prunedData.cols());
		MatrixXd xi2 (1, prunedData.cols());
		MatrixXd xi3 (1, prunedData.cols());
		for(int i = 0; i < prunedData.cols(); i++){
			xi1(i) = ui[(int)prunedData(3, i)](0);
			xi2(i) = ui[(int)prunedData(3, i)](1);
			Vector2d unitVxy(xi1(i),xi2(i));
			Vector2d pointXY(prunedData(1,i), prunedData(2,i));
			xi3(i) = unitVxy.transpose() * rotMat * (pointXY-robotPos);
		}

		MatrixXd A(3, prunedData.cols());
		A.row(0) = xi1.row(0);
		A.row(1) = xi2.row(0);
		A.row(2) = xi3.row(0);
		A.transposeInPlace();

		MatrixXd AtA = A.transpose() * A;
		MatrixXd AtYi = A.transpose() * prunedData.row(4).transpose();
		//Vector3d b(AtA.inverse() * AtYi);
		b = AtA.inverse() * AtYi;

		MatrixXd S2 = ((prunedData.row(4).transpose() - A * b).transpose() * (prunedData.row(4).transpose() - A * b)) / (400);
		double s2 = S2(0,0);

		MatrixXd C (3,3);
		C << s2 * (A.transpose() * A).inverse();
		sigX = C(0,0);
		sigY = C(1,1);
		sigA = C(2,2);
		//std::cout << " C\n";
		//std::cout << C << std::endl;

		// Step 4
		ddx += b(0);
		ddy += b(1);
		dda += b(2);

		//std::cout << "b" << endl;
		//std::cout << b << endl;
		//std::cout << "X Y A" << endl;
		//std::cout << ddx << ' ' << ddy << ' ' << dda*180/M_PI << endl;
		// Step 5
		if (sqrt(pow(b(0), 2) + pow(b(1), 2)) < 5 && abs(b(2)) < 0.1 * M_PI / 180) {
			//std::cout << "break" << endl;
			return;
	   }




		}

	}

};




int main(void){

	coxMap map;
	map.setup();
	std::cout << "Here are the vectors u1:\n" << map.ri[0]  << std::endl;
	while(1){
	map.gather_data();
	map.loop(10);
	std::cout << "Position: X=" << map.ddx << " Y=" << map.ddy << " A=" << map.dda << std::endl;
	std::cout << "Uncertainty: X=" << map.sigX << " Y=" << map.sigY << " A=" << map.sigA << std::endl;
	}
	cout << "Finished" << endl;
};
