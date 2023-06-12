/*
 * main.cpp
 *
 *  Created on: May 3, 2023
 *      Author: Erik
 */
#include <iostream>
#include <thread>
#include <Eigen/Dense>
#include "runCv.h"
#include "lidar.h"
#include "coxRemade.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include "spi_com.h"
#include "motors.h"
#include "kalman.h"

using namespace std;

// motor top speed is 4670 rpm 
// the encoder gives 4096 pulses per revolution of the motor shaft
// the communication program has th units of counts per 10ms
// if we want to drive the motor at top speed we should call 3188 counts
// for this 3000 counts seems to be sufficient
// remeber that the output shaft spins at 6.6 times the motor

//M1 is left motor with negative direction
//M2 is right motor with poitive direction

Eigen::Vector3d home(1270, 250, M_PI/2);
Eigen::Vector3d home2(1270, 250, 3*M_PI/2);
Eigen::Vector3d p1(1270, 400, 3*M_PI/2);
Eigen::Vector3d p2(600, 1070, M_PI/2);
Eigen::Vector3d p3(2030, 1817, M_PI/2);
Eigen::Vector3d p4(1270, 2435, 3*M_PI/2);
Eigen::Vector3d p5(660, 1150, M_PI/2);

void print_pos(void){
		std::cout << toMain() << endl;
}

class robot{
	int state_ = -1;
	char key;
	bool run = true;
	const double coeff = (15 * 6.6 * 4096)/(43*M_PI  * 100);
	//const double coeff = 1/1.45115;
	float wheelbase = 205;
	int cargo = 0;
	int turnDir = 1;
	

	enum state{
		POSITIONING,
		SEARCHING,
		PICKING_UP,
		RETRIEVING,
		DEPOSITING,
		EVALUATING,
	};

	public:
	
	//inverse kinematics
	void setSpeed(float speed, float angle){	
		float vR = speed + (angle * wheelbase)/2;
		float vL = speed - (angle * wheelbase)/2;	
		MotorData.Set_Speed_M1 = -vL*coeff;
		MotorData.Set_Speed_M2 = vR*coeff;
		//cout << "coeff control " << angle << endl;
		//cout << "Left speed actual " << MotorData.Act_Speed_M1 << endl;
		//cout << "Right speed actual " << MotorData.Act_Speed_M2 << endl;
		//cout << "Left speed set " << -vL*coeff << endl;
		//cout << "Right speed set " << vR*coeff << endl;
		//cout << "Left speed diff " << MotorData.Act_Speed_M1 - -vL*coeff << endl;
		//cout << "Right speed diff " << MotorData.Act_Speed_M2 - vR*coeff << endl;
		
	}
	
	double changeAngle(double angle){
		angle = modulosAngle(angle);
		if (angle > M_PI){
			angle -= 2*M_PI;
		}
		return angle;
	}

	void speedProfiler(Eigen::Vector3d target){
			//kRho > 0, kBeta < 0, kAlpha + 5/3*kBeta - 2/pi*kRho > 0
			float kRho = 0.3;
			float kAlpha = 5;
			float kBeta = -1;
			double blank = coeff;
			double targetAng = changeAngle(target(2));
			float dX = target(0) - position(0); 
			float dY = target(1) - position(1); 
			float dA = target(2) - position(2);
			float rho = sqrt(dX*dX + dY*dY);
			float alpha = changeAngle(-changeAngle(position(2)) + atan2(dY,dX));
			cout << "TURNING" << endl;
			while(abs(alpha) > 0.3){
				//cout << "alpha " << alpha << "angle " << position(2) << "atan2 " << atan2(dY,dX) << endl;
				if(abs(alpha) > M_PI/2){
					setSpeed(0, 0.5/alpha);
				}
				else{
					setSpeed(0, alpha*0.5);
				}
				delay(100);
				alpha = changeAngle(-changeAngle(position(2)) + atan2(dY,dX));
			}
			cout << "GOING TO TARGET" << endl;
			while(1){
				double robAng = changeAngle(position(2));
				dX = target(0) - position(0); //position(0)
				dY = target(1) - position(1); //position(1)
		
				float rho = sqrt(dX*dX + dY*dY);
				float alpha = -changeAngle(position(2)) + atan2(dY,dX);
				float beta = -changeAngle(position(2)) - alpha;//targetAng;//
				setSpeed(kRho*rho, kAlpha*changeAngle(alpha) + kBeta*changeAngle(beta));
				if(abs(position(0)-target(0)) < 100 && abs(position(1)-target(1)) < 100 ){
					MotorData.Set_Speed_M1=0;
					MotorData.Set_Speed_M2=0;
					cout << "ARRIVED" << endl;
					break;
				}
				
			}
	}



	void stateTransition(char com){
		while(run){
		if (position(0) > 1930){
			turnDir = -1;
		}
		else if(position(0) < 500){
			turnDir = 1;
		}
		switch(state_){
			case POSITIONING:
			{
				speedProfiler(p2);
				state_ = SEARCHING;
				break;
			}
//-------------------------------------------------------------------------------------
			case SEARCHING:
			{
				// steer motors to pos
			int d = boxPos;
//cout << "d " << d << endl;
//cout << "trueD " << trueD << endl;

				if (d == 500){
					
					setSpeed(0,- trueD*0.0001);			
					if(abs(trueD) < 80){	
						state_ = EVALUATING;
						cout << "EVALUATING " << endl;
						setSpeed(0,0);
						break;
					}
				}
				//else if (d == 2000){
				//	setSpeed(30,0);
				//}
				else if(d==1000){
					setSpeed(0,-0.25 * turnDir);
				}
				else{
				//int leftMotor = -d*3 -3000/log10(abs(d+1)) - d*2;           //-3000/((abs(d)+1) - d*2;
				//int rightMotor = -d*3 + 3000/log10(abs(d+1)) - d*2;           //3000/((abs(d)+1) - d*2;
				//MotorData.Set_Speed_M1 = leftMotor;
				//MotorData.Set_Speed_M2 = rightMotor;
					setSpeed(70/log10(abs(d+1)), - d*0.0005);
//Scout << "forward " << 10/log10(abs(d+1)) << " turn " << - d*0.001 << endl;
				}
				break;
			}
//--------------------------------------------------------------------------------------
			case EVALUATING:
			{
				
				setSpeed(0,0);
				if (boxEval(10)){
					state_ =PICKING_UP;
					cout << "PICKING_UP" << endl;
				}
				else{
					setSpeed(0,-0.3 * turnDir);
					cout << "going " << endl;
					delay(4000);
					cout << " here " << endl;
					setSpeed(0,0);
					state_ = SEARCHING;
					cout << "SEARCHING" << endl;
				}
				break;
			}
//--------------------------------------------------------------------------------------

			case PICKING_UP:
			{	
				setSpeed(50,0);
				cout << "going " << endl;
				delay(8000);
				cout << " here " << endl;
				setSpeed(0,0);
				cargo++;
				if (cargo == 2){
					state_ = RETRIEVING;
					cout << "RETRIEVING" << endl;
				}
				else{
					state_ =  SEARCHING;
					cout << "SEARCHING" << endl;
				}
				break;
			}
//--------------------------------------------------------------------------------------

			case RETRIEVING:
			{	
				//thCam.detach();
				speedProfiler(home);
				setSpeed(-50, 0);
				delay(6000);
				setSpeed(0,-0.3);
				delay(4000);
				setSpeed(0,0);
				state_ = -1;
				cout << "HOME" << endl;
				state_ = SEARCHING;
				break;
			}
//--------------------------------------------------------------------------------------

			case DEPOSITING:
			{	cout << "DEPOSITING" << endl;
					cin >> key;
					if (key == 'i'){
						cout << "switching" << endl;
						state_ = SEARCHING;
					}
				break;
			}
//--------------------------------------------------------------------------------------

			default:
			{		cin >> key;
					cout << "TESTING" << endl;
					if (key == 'i'){
						cout << "switching" << endl;
						state_ = POSITIONING;
					}
					else if (key == 'c'){
						cout << boxPos << endl;
						showFrame();
					}
					else if (key == 's'){
						needData = true;
						while(!coxDone){}
						coxDone = false;
						//cout << posFix.transpose() << endl;
					}
					else if (key == 'q'){
						int stop = system("python3 stop.py &");
						run = false;
						exit(0);

					}
					else if (key == 'l'){
						setSpeed(0,0.001*200);
						cout << "left" << endl;
					}
					else if (key == 'r'){
						setSpeed(0,-1);
						cout << "right" << endl;

					}
					else if (key == 'f'){
						setSpeed(67.55, 0);
						cout << "forward" << endl;

					}
					
					else if (key == 'h'){

						speedProfiler(p2);
						speedProfiler(p3);
						speedProfiler(p4);
						speedProfiler(home2);

						
					}
					else if (key == 'm'){
						MotorData.Set_Speed_M1=-40.96*6.6*15;
						MotorData.Set_Speed_M2=40.96*6.6*15;
						cout << "max" << endl;
						cout << "Left speed actual " << MotorData.Act_Speed_M1 << endl;
						cout << "Right speed actual " << MotorData.Act_Speed_M2 << endl;

					}
					else if (key == 'p'){
						setSpeed(0,0);
						cout << "STOP" << endl;
					}
					else if (key == '2'){
						state_ = SEARCHING;
						cout << "STOP" << endl;
					}
					else if (key == '3'){
						state_ = EVALUATING;
						cout << "STOP" << endl;
					}
					else if (key == '4'){
						state_ = PICKING_UP;
						cout << "STOP" << endl;
					}
					break;
				}
			}
			}
		}
};


int main(void){
	thread thCam(camMain);
	thread thMotor(motorMain);
	thread thLidar(lidarRead);
	thread thKalman(kalmanMain);
	robot bot;
	bot.stateTransition('t');
	thCam.join();
	thMotor.join();
	thLidar.join();
	thKalman.join();
	return 1;
}

/*
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
'
*/