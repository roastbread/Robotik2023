/*
 * runCv.cpp
 *
 *  Created on: Mar 23, 2023
 *      Author: Erik
 */



#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <mutex>


using namespace cv;
using namespace std;

int boxPos;



VideoCapture initCamera(void){
	VideoCapture cap;
	int deviceID = 0;             // 0 = open default camera
	int apiID = cv::CAP_ANY;
	cap.open(deviceID, apiID);
	// check if we succeeded
	if (!cap.isOpened()) {
		cerr << "ERROR! Unable to open camera\n";
		exit(1);
	}
	//--- GRAB AND WRITE LOOP
	cout << "Start grabbing" << endl
		<< "Press any key to terminate" << endl;
	return cap;
}

Mat takePicture(VideoCapture cap){
	Mat frame;
	// wait for a new frame from camera and store it into 'frame'
	cap.read(frame);
	// check if we succeeded
	if (frame.empty()) {
		cerr << "ERROR! blank frame grabbed\n";
		exit(1);
	}
	return frame;

}

Mat colorSegmentation(Mat frame, int minHue, int maxHue){
	Mat frame_threshold, imHSV;
	Mat kernel = (Mat_<unsigned char>(3,3) <<  0, 1, 0, 1, 1, 1, 0, 1, 0);
	cvtColor(frame, imHSV, COLOR_BGR2HSV);
	inRange(imHSV, Scalar(minHue, 50, 50), Scalar(maxHue, 230, 230), frame_threshold);
	morphologyEx(frame_threshold, frame_threshold, cv::MORPH_OPEN, kernel, Point(-1,-1), 5);
	return frame_threshold;
}

int trackBox(Mat frame_threshold){
	vector<vector<Point> > contours;
	findContours(frame_threshold, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	vector<Moments> mu(contours.size());
	for( size_t i = 0; i < contours.size(); i++ )
			{
				mu[i] = moments( contours[i] );
	}
	vector<Point2f> mc(1);
	int blobSize = 0;
	size_t index = 0;
	for( size_t i = 0; i < contours.size(); i++ ){
		//add 1e-5 to avoid division by zero
		if(mu[i].m00 > blobSize){
			blobSize = mu[i].m00;
			index = i;
		}
	}
	if (blobSize >= 0.9 * frame_threshold.size[0] * frame_threshold.size[1]){
		return 1000;		// condition to go to next state
	}
	int posX = 500;  //Om nåt är fel är det förmodligen på grund av detta, borde vara null
	if(blobSize != 0){
		mc.at(0) = Point2f( static_cast<float>(mu[index].m10 / (mu[index].m00 + 1e-5)),
					static_cast<float>(mu[index].m01 / (mu[index].m00 + 1e-5)) );
		circle( frame_threshold, mc[0], 4, Scalar(100, 200, 0), -1 );

		posX = static_cast<int>(mc[0].x) - 640/2;
	}
	return posX;
}



int toMain(void){
	return boxPos;
}

void camMain(void){
	mutex m;
	Mat frame, frame_threshold;
	VideoCapture cap = initCamera();
	while(1){
		frame = takePicture(cap);
		frame_threshold = colorSegmentation(frame, 90, 120);
		boxPos = trackBox(frame_threshold);

	}

}



