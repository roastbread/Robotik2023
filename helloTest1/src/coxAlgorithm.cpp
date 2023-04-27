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

	MatrixXd line {
		  {559, 1140, 2856, -910},
		  {2856, -910, 1336, -2670},
		  {1336, -2670, -1004, -610},
		  {-1004, -610, 559, 1140},
	};
	MatrixXd rotMat{
		{0, -1},
		{1, 0},
	};
	Vector2d Li[4];
	Vector2d ui[4];
	float ri[4];

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
		cout << sizeof(angs)/8 << endl;
		ang = Eigen::Map<MatrixXd>(angs, 1, 400);
		DIS = Eigen::Map<MatrixXd>(dists, 1, 400);

		//std::remove("testfile.txt");
		data.close();
		//std::ofstream empty;
		//empty.open("textfile.txt");
		//std::rename("textfile.txt","testfile.txt");
		//empty.close();
	}
    void loop(int max_iterations){
        for (int i = 0; i < max_iterations; i++ ){
        	//x = x.array();
        	//y = y.array();
            //sensor to Cartesian coordinates
        	//std::cout << "hej" << std::endl;
        	//std::cout << DIS << std::endl;
            x = DIS.array() * cos(ang.array()*M_PI/180);
            y = DIS.array() * sin(ang.array()*M_PI/180);
            //sensor to Robot coordinates
            //std::cout << x << std::endl;
            //std::cout << x.cols() << std::endl;
            //std::cout << y << std::endl;
            //std::cout << y.cols() << std::endl;

            MatrixXd rotMatRobot {
                    {cos(sensorPose[2]), -1*sin(sensorPose[2]), sensorPose[0]},
                    {sin(sensorPose[2]), cos(sensorPose[2]), sensorPose[1]},
                    {0, 0, 1},
            };

            //std::cout << x.row(0) << std::endl;
            MatrixXd xy1T(3,x.cols());
            xy1T.row(0) = x;
            xy1T.row(1) = y;
            xy1T.row(2).setOnes();



            MatrixXd Xs;

            Xs = rotMatRobot*xy1T;


            //Robot coordinates to world coordinates
            MatrixXd rotMatWorld {
                    {cos(prevA+dda), -1*sin(prevA+dda), prevX+ddx},
                    {sin(prevA+dda), cos(prevA+dda), prevY+ddy},
                    {0, 0, 1},
            };

            MatrixXd Xw;
            Xw = rotMatWorld*Xs;

            // Step 2 Find targets for data points
            // V is the number of points from the laser scan

            MatrixXd V(2,x.cols());
			V.row(0) = Xw.row(0);
			V.row(1) = Xw.row(1);

			MatrixXd dist_All(2,x.cols());
			dist_All.row(0).setZero();
			dist_All.row(1).setZero();

			MatrixXd qi(2,x.cols()); //qi == u1 från matlab cox
			qi.row(0).setZero();
			qi.row(1).setZero();

			MatrixXd r(1,x.cols());
			r.row(0).setZero();

			MatrixXd yi(1,x.cols());
			yi.row(0).setZero();

			for (int k = 0; k < V.cols(); k++) {
				VectorXd dist(line.rows());
				for (int j = 0; j < line.cols(); j++) {
					// Tar ut distansen från laser scan till väggarna
					dist(j) = pointLineDistance(V(0, k), V(1, k), line(j, 0), line(j, 1), line(j, 2), line(j, 3));
				}

				Index index;
				double minVal = dist.minCoeff(&index);

				dist_All.col(k) << minVal, index;
				qi.col(k) = ui[index];

				//z
				VectorXd z = line.row(index).head<2>();


				//r

					r(k) = abs(qi.col(k).dot(z));


				//yi
					yi(k) = r(k) - abs(qi.col(k).dot(V.col(k)));

			}

			//step 3

			Matrix2d M;
			M << 0, -1, 1, 0;



			MatrixXd vm{
					{prevX+ddx},
					{prevY+ddy},
			};


			MatrixXd xi3(1,x.cols());
			xi3.row(0).setZero();

			for (int g = 0; g < 400; g++) {
				xi3(g) = qi.col(g).transpose() * M * (V.col(g) - vm);
			}

			VectorXd tempx = dist_All.row(0).transpose();

			// Calculate the median using std::nth_element
			auto middle = tempx.begin() + tempx.size() / 2;
			std::cout << tempx.transpose() << endl;
			nth_element(tempx.begin(), middle, tempx.end());
			std::cout << tempx.transpose() << endl;
			double threshold = *middle;
			std::cout << threshold << endl;
			//double threshold = 18.7341;

			vector<long long> outliers;
			for (int j = 0; j < tempx.rows(); j++) {
				if (tempx(j) > threshold) { //changed to contain inliers
					outliers.push_back(j);
				}}
			for(int f=0; f<=outliers.size(); f++){
				std::cout << outliers[f] << endl;
			}


			// Remove outliers from the matrices
			for (int j = outliers.size()-1; j >= 0; j--) {
				long long outlierIndex = outliers[j];
				qi.col(outlierIndex).swap(qi.col(qi.cols()-1)); // Swap outlier column with last column
				qi.conservativeResize(qi.rows(), qi.cols()-1); // Remove last column
				yi.col(outlierIndex).swap(yi.col(yi.cols()-1));
				yi.conservativeResize(yi.rows(), yi.cols()-1);
				xi3.col(outlierIndex).swap(xi3.col(xi3.cols()-1));
				xi3.conservativeResize(xi3.rows(), xi3.cols()-1);
				Xw.col(outlierIndex).swap(Xw.col(Xw.cols()-1));
				Xw.conservativeResize(Xw.rows(), Xw.cols()-1);
			}
			std::cout << yi << endl;


			MatrixXd xi1 (1,qi.cols());
			xi1 << qi.row(0);
			MatrixXd xi2 (1,qi.cols());
			xi2 << qi.row(1);


			MatrixXd A(3, qi.cols());
			A.row(0) = xi1.row(0);
			A.row(1) = xi2.row(0);
			A.row(2) = xi3.row(0);
			A.transposeInPlace();


			MatrixXd AtA = A.transpose() * A;
			MatrixXd AtYi = A.transpose() * yi.transpose();
			MatrixXd b = AtA.inverse() * AtYi;

			MatrixXd S2 = ((yi.transpose() - A * b).transpose() * (yi.transpose() - A * b)) / (400 - 4);
			double s2 = S2(0,0);

			MatrixXd C (3,3);
			C << s2 * (A.transpose() * A).inverse();
			//std::cout << " C\n";
			//std::cout << C << std::endl;

			// Step 4
			ddx += b(0);
			ddy += b(1);
			dda += b(2);

			std::cout << "b" << endl;
			std::cout << b << endl;
			std::cout << "X Y A" << endl;
			std::cout << ddx << ' ' << ddy << ' ' << dda*180/M_PI << endl;
			// Step 5
			if (sqrt(pow(b(0), 2) + pow(b(1), 2)) < 5 && abs(b(2)) < 0.1 * M_PI / 180) {
				std::cout << "break" << endl;
				break;
          /**/  }

        }
    };


double pointLineDistance(double x0, double y0, double x1, double y1, double x2, double y2) {
    double dist = abs((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1) / sqrt(pow(y2-y1, 2) + pow(x2-x1, 2));
    return dist;
}
};

int main(void){
	coxMap map;
	map.setup();
	std::cout << "Here are the vectors u1:\n" << map.ri[0]  << std::endl;
	map.gather_data();
	map.loop(10);
	cout << "Finished" << endl;
}
