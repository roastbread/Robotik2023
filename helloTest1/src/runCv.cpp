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

using namespace cv;
using namespace std;

void print_imArray(Mat image){
	for (int i=0;image.size[0]; i++){
		for (int j=0;image.size[1]; j++){
			printf("%d", image.at<int>(i,j));
		}
		printf("\n");
	}
}

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

Mat colorSegmentation(Mat frame, int minHue = 90, int maxHue = 120){
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
	int posX = 500;  //Om nåt är fel är det förmodligen på grund av detta, borde vara null
	if(blobSize != 0){
		mc.at(0) = Point2f( static_cast<float>(mu[index].m10 / (mu[index].m00 + 1e-5)),
					static_cast<float>(mu[index].m01 / (mu[index].m00 + 1e-5)) );
		circle( frame_threshold, mc[0], 4, Scalar(100, 200, 0), -1 );

		posX = static_cast<int>(mc[0].x) - 640/2;
	}
	return posX;
}

int maintttt(int, char**) {
	Mat frame, frame_threshold;
	VideoCapture cap = initCamera();
	for(;;){
		frame = takePicture(cap);
		frame_threshold = colorSegmentation(frame);
		cout << "Target position = " << trackBox(frame_threshold) << endl;
		imshow("frame",frame);
		imshow("thresh",frame_threshold);
		char key = (char) waitKey(30);
		if (key == 'q' || key == 27)
		{
			break;
		}
	}

	return EXIT_SUCCESS;
}



