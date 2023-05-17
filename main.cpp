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
#include "spi_com.h"
#include "motors.h"

using namespace std;

// motor top speed is 4670 rpm 
// the encoder gives 4096 pulses per revolution of the motor shaft
// the communication program has th units of counts per 10ms
// if we want to drive the motor at top speed we should call 3188 counts
// for this 3000 counts seems to be sufficient
// remeber that the output shaft spins at 6.6 times the motor

void print_pos(void){
		std::cout << toMain() << endl;
}

class robot{
	int state_;
	char key;
	bool run = true;

	enum state{
		SEARCHING,
		PICKING_UP,
		RETRIEVING,
		DEPOSITING,
	};

	public:

	void speedProfiler(float targetX, float targetY, float targetA){
		// need odometry after all
	}

	void stateTransition(char com){
		while(run){
		switch(state_){
			case SEARCHING:
				// steer motors to pos
				int d = toMain();
				if (d == 1000){
					state_ = PICKING_UP;
					break;
				}
				int leftMotor = 1000 + d*10;
				int rightMotor = 1000 - d*10;
				MotorData.Set_Speed_M1 = leftMotor;
				MotorData.Set_Speed_M2 = rightMotor;
				break;

//--------------------------------------------------------------------------------------

			case PICKING_UP:
				cout << "PICKING_UP" << endl;
					cin >> key;
					if (key == 'y'){
						cout << "switching" << endl;
						state_ = RETRIEVING;
					}
				break;

//--------------------------------------------------------------------------------------

			case RETRIEVING:
				cout << "RETRIEVING" << endl;
					cin >> key;
					if (key == 'u'){
						cout << "switching" << endl;
						state_ = DEPOSITING;
					}
				break;

//--------------------------------------------------------------------------------------

			case DEPOSITING:
				cout << "DEPOSITING" << endl;
					cin >> key;
					if (key == 'i'){
						cout << "switching" << endl;
						state_ = SEARCHING;
					}
				break;

//--------------------------------------------------------------------------------------

			default:
					cin >> key;
					cout << "TESTING" << endl;
					if (key == 'i'){
						cout << "switching" << endl;
						state_ = SEARCHING;
					}
					else if (key == 'c'){
						cout << boxPos << endl;
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

					}
					else if (key == 'l'){
						MotorData.Set_Speed_M1=1000;
						MotorData.Set_Speed_M2=-1000;
						cout << "left" << endl;
					}
					else if (key == 'r'){
						MotorData.Set_Speed_M1=-1000;
						MotorData.Set_Speed_M2=1000;
						cout << "right" << endl;

					}
				break;
		}
	}
	}

};




int main(void){
	//thread thCam(camMain);
	thread thMotor(motorMain);
	thread thLidar(lidarRead);
	thread thCox(coxMain);

	//thPrint.join();
	//float nums[] = {1, 2, 3, 4, 5};
	//float meanum = mean(nums);
	//std::cout << meanum << std::endl;

	robot bot;
	bot.stateTransition('t');
	//thCam.join();
	thLidar.join();
	thCox.join();
	return 1;
}



