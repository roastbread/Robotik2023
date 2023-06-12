#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <cmath>
#include <chrono>
#include "coxRemade.h"
#include "lidar.h"
#include "motors.h"

using namespace std;
using namespace Eigen;

//Vector3d position(280, 1200, 0);
Vector3d position(1270, 195, M_PI/2);
Matrix3d covariance = Matrix3d::Identity();

float velocity;
float omega;

unsigned long t_old;
float wheelbase = 205;
float circumference = 43 * M_PI;
float reduction = 6.6*15;
float pulsesPerTurn = 4096*reduction;
float mmPerPulse = circumference/pulsesPerTurn;
const float coeff = mmPerPulse;
//const double coeff = 10/30.0177;
//auto then = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
auto then = chrono::high_resolution_clock::now();
auto start = chrono::high_resolution_clock::now();

ofstream coxLogs;
long oldValM1 = MotorData.Encoder_M1;
long oldValM2 = MotorData.Encoder_M2;

void createCoxInstance(){		
    while(Array_full_flag == 0 || !needData){}
    needData = false;
    loop(100, position(0), position(1), position(2));
    Array_full_flag = 0;
    coxDone = true;

}

double modulosAngle(double vinkel) {
	double tempA = fmod(vinkel, 2*M_PI);

	if (tempA < 0){
		tempA += 2*M_PI;
	}

	return tempA;
}

void odometry(void){
/*
//cout <<"coeff odometry " << coeff << endl;
		auto duration = chrono::high_resolution_clock::now() - then;
		then = then + duration;
		double dT = (double)(chrono::duration_cast<chrono::microseconds>(duration).count()) / 1000000;
		//cout << "dT " << dT << endl;
if(dT > 1){
	dT = 0;
}
		float dDl = -MotorData.Act_Speed_M1 * coeff * dT; //mm
		float dDr = MotorData.Act_Speed_M2 *coeff * dT; //mm
*/


delay(15);
		float dDl = -(MotorData.Encoder_M1 - oldValM1) * coeff;
		float dDr = (MotorData.Encoder_M2 - oldValM2) * coeff;
if( dDl > 100 || dDr > 100){
	dDl = 0;
	dDr = 0;
}
		oldValM1 = MotorData.Encoder_M1;
		oldValM2 = MotorData.Encoder_M2;
//cout<< " dDl dDr " << dDl << ' ' << dDr << endl;
		float dD = (dDl + dDr)/2;
		float dA = (dDr - dDl)/wheelbase;
		float deltaTheta = dA;
		float compensation = sin((deltaTheta + pow(1,-100))/ 2.0) / ((deltaTheta + pow(1,-100))/ 2.0);
		float deltaX = dD * compensation * cos(position(2) + dA/2);
		float deltaY = dD * compensation * sin(position(2) + dA/2);

//cout << "deltaX " << deltaX << " deltaY " << deltaY << endl;

		//float dD = velocity;
		//float dA = omega;

		Matrix3d Axya;
        Axya << 1, 0, -deltaY,
                0, 1, deltaX,
                0, 0, 1;

        Matrix<double, 3, 2> JdRdL;
        JdRdL << 0.5 * cos(position(2) + deltaTheta / 2.0) - (dD / (2.0 * wheelbase)) * sin(position(2) + deltaTheta / 2.0),
                0.5 * cos(position(2) + deltaTheta / 2.0) + (dD / (2.0 * wheelbase)) * sin(position(2) + deltaTheta / 2.0),

                0.5 * sin(position(2) + deltaTheta / 2.0) + (dD / (2.0 * wheelbase)) * cos(position(2) + deltaTheta / 2.0),
                0.5 * sin(position(2) + deltaTheta / 2.0) - (dD / (2.0 * wheelbase)) * cos(position(2) + deltaTheta / 2.0),

                1.0 / wheelbase,
                -1.0 / wheelbase;

		double SigmaB = 0.05;
        double k = pow(0.005,2);

        Matrix2d Cdrdl;
        Cdrdl << k * abs(dDr), 0,
                 0, k * abs(dDl);

        double dDr_dDl = (dDr - dDl) / (2.0 * wheelbase);
        Vector3d jBB;
        jBB << ((dDr + dDl) / 2.0) * dDr_dDl * sin(position(2) + dDr_dDl),
                -((dDr + dDl) / 2.0) * dDr_dDl * cos(position(2) + dDr_dDl),
                -dDr_dDl / (wheelbase * wheelbase);

	position(0) = position(0) + deltaX;
        position(1) = position(1) + deltaY;
        position(2) = modulosAngle(position(2) + deltaTheta);

        covariance = Axya * covariance * Axya.transpose() +
                                    JdRdL * Cdrdl * JdRdL.transpose() +
                                    jBB * SigmaB * jBB.transpose();
}

double angDiff(double ang1, double ang2){
	double diff = (ang1 - ang2) + ((ang1 - ang2) < -M_PI) * (2*M_PI) + ((ang1 - ang2) > M_PI) * (-2*M_PI);
	/*
	if(ang1 > ang2){
		diff = ang2 + (2*M_PI-ang1);
	} 
	else if(ang1 < ang2){
		diff = ang1 + (2*M_PI-ang2);
	}
*/
	return diff;
}

void kalmanFilter(void){
	createCoxInstance();
        // cox differences from last pos
        double dx = posFix(0);
        double dy = posFix(1);
        double da = posFix(2);

	//dx = 0;
	//dy = 0;
	//da = 0;

        //Hard coded for now, its the uncertainty matrix from the cox scan matching algorithm.
        Matrix<double, 3, 3> C;
        C << covarianceFix;
	
	auto moment = chrono::high_resolution_clock::now();
	auto timestamp = (chrono::duration_cast<chrono::microseconds>(moment-start)) ;
	coxLogs << timestamp.count()/ 1000000 << ' ' << dx + position(0) << ' ' << dy + position(1) << ' ' << modulosAngle(da + position(2)) << ' ' << C(0,0) << ' ' << C(0,1) << ' ' << C(0,2) << ' ' << C(1,0) << ' ' << C(1,1) << ' ' << C(1,2) << ' ' << C(2,0) << ' ' << C(2,1) << ' ' << C(2,2) << "\n";
	//C = Matrix3d::Identity();

        // sigPF
        Matrix<double, 3, 3> sigPF;
        sigPF.row(0) = C.row(0).head(3);
        sigPF.row(1) = C.row(1).head(3);
        sigPF.row(2) = C.row(2).head(3);
	sigPF = C;

        //sigxHat
        Matrix<double, 3, 3> sigxHat;
        //sigxHat.row(0) = P.block(0, 0, 1, 3);
        //sigxHat.row(1) = P.block(0, 3, 1, 3);
        //sigxHat.row(2) = P.block(0, 6, 1, 3);
	sigxHat = covariance;

        //xHatPF
	double diff = angDiff(position(2) + da, position(2));
        Matrix<double, 3, 1> xHatPF;
        xHatPF(0,0) = position(0) + dx;
        xHatPF(1,0) = position(1) + dy;
        xHatPF(2,0) = 0;

        //xHatMinus
        Matrix<double, 3, 1> xHatMinus;
        xHatMinus(0,0) = position(0);
        xHatMinus(1,0) = position(1);
        xHatMinus(2,0) = diff;

//bool invert;
//Matrix3d inverse;
//double det;
//Matrix3d sig = sigPF + sigxHat;
//(sigPF + sigxHat).computeInverseAndDetWithCheck(inverse, det, invert);

        //xHatPlus
        MatrixXd inv_sigPF_sigxHat = (sigPF + sigxHat).inverse();
        MatrixXd term1 = sigPF * inv_sigPF_sigxHat * xHatMinus;
        MatrixXd term2 = sigxHat * inv_sigPF_sigxHat * xHatPF;
        MatrixXd xHatPlus = term1 + term2;

        //updating pos with index kk-1
        position(0) = xHatPlus(0);
        position(1) = xHatPlus(1);
        position(2) = modulosAngle(position(2) + xHatPlus(2));

        //sigxHatPlus
        MatrixXd inv_sigPF = sigPF.inverse();
        MatrixXd inv_sigxHat = sigxHat.inverse();
        MatrixXd inv_sum = (inv_sigPF + inv_sigxHat).inverse();
        covariance = inv_sum;
}

void kalmanMain(void){
	int counter = 0;
	ofstream odoLogs;
	ofstream kalmanLogs;
	kalmanLogs.open("Kalman.txt");
	odoLogs.open("Odometry.txt");
	coxLogs.open("Scanmatch.txt");
	while(1){
		odometry();
		auto timestamp = (chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now()-start).count()) / 1000000;

		odoLogs << timestamp << ' ' << position(0) << ' ' << position(1) << ' ' << position(2) << ' ' << covariance(0,0) << ' ' << covariance(0,1) << ' ' << covariance(0,2) << ' ' << covariance(1,0) << ' ' << covariance(1,1) << ' ' << covariance(1,2) << ' ' << covariance(2,0) << ' ' << covariance(2,1) << ' ' << covariance(2,2) << "\n";	 
		//cout << "Position = " << position.transpose() << endl;
		//cout << "Covariance = " << covariance << endl;
		if (counter == 40){
			needData = true;
			kalmanFilter();
			auto timestamp = (chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now()-start).count()) / 1000000;
			kalmanLogs << timestamp << ' ' << position(0) << ' ' << position(1) << ' ' << position(2) << ' ' << covariance(0,0) << ' ' << covariance(0,1) << ' ' << covariance(0,2) << ' ' << covariance(1,0) << ' ' << covariance(1,1) << ' ' << covariance(1,2) << ' ' << covariance(2,0) << ' ' << covariance(2,1) << ' ' << covariance(2,2) << "\n";
			counter = 0;
			cout << "Position after kalman = " << position.transpose() << endl;
			//cout << "Covariance after kalman = " << covariance << endl;
			then = chrono::high_resolution_clock::now();
		}
		counter++;
	}
	kalmanLogs.close();
	odoLogs.close();
	coxLogs.close(); 
	
}