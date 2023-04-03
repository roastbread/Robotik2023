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

int main(int, char**) {
    Mat frame, imHSV, frame_threshold;
    //--- INITIALIZE VIDEOCAPTURE
    VideoCapture cap;
    // open the default camera using default API
    // cap.open(0);
    // OR advance usage: select any API backend
    // Trackbars to set thresholds for HSV values
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_ANY;      // 0 = autodetect default API

    Mat kernel = (Mat_<unsigned char>(3,3) <<  0, 1, 0, 1, 1, 1, 0, 1, 0);
    vector<vector<Point> > contours;
    // open selected camera using selected API
    cap.open(deviceID, apiID);
    // check if we succeeded
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
    //--- GRAB AND WRITE LOOP
    cout << "Start grabbing" << endl
        << "Press any key to terminate" << endl;
    for (;;)
    {
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);
        // check if we succeeded
        if (frame.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }
        cvtColor(frame, imHSV, COLOR_BGR2HSV);
        inRange(imHSV, Scalar(100, 0, 0), Scalar(120, 255, 255), frame_threshold);
        morphologyEx(frame_threshold, frame_threshold, cv::MORPH_OPEN, kernel, Point(-1,-1), 5);

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
		if(blobSize != 0){
			mc.at(0) = Point2f( static_cast<float>(mu[index].m10 / (mu[index].m00 + 1e-5)),
						static_cast<float>(mu[index].m01 / (mu[index].m00 + 1e-5)) );
			circle( frame_threshold, mc[0], 4, Scalar(100, 200, 0), -1 );

			// show live and wait for a key with timeout long enough to show images
			cout << "Target position = " << static_cast<int>(mc[0].x) - 1280/2 << endl;
		}
        imshow("frame",frame);
        imshow("thresh",frame_threshold);
        char key = (char) waitKey(30);
		if (key == 'q' || key == 27)
		{
			break;
		}
    }
    int red;
	red = frame.at<int>(0,0);
    printf("Dimensions are %d, %d %d \n", frame.dims, frame.size[0], frame.size[1]);
    printf("%d", red);
    // the camera will be deinitialized automatically in VideoCapture destructor
	return EXIT_SUCCESS;
}



