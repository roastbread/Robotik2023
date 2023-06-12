/*
 * runCv.h
 *
 *  Created on: May 3, 2023
 *      Author: Erik
 */

#ifndef RUNCV_H_
#define RUNCV_H_

#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <mutex>
//#include "runCv.cpp"

using namespace cv;
using namespace std;

extern int boxPos;
extern int trueD;

void showFrame();

int toMain(void);

bool boxEval(int counts);

void camMain(void);



#endif /* RUNCV_H_ */
