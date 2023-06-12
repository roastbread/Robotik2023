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

int boxPos = 0;
int trueD;
double sampling = 50;
double lpFilter = 0.5;

complex<double> j(0,1);
vector<complex<double>> refs[2];
Mat frame_show;

void showFrame(){
	imshow("image",frame_show);
	waitKey(0);
	destroyAllWindows();
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

VideoCapture cap = initCamera();

Mat takePicture(VideoCapture cap){
	Mat frame;
	// wait for a new frame from camera and store it into 'frame'
	cap.read(frame);
	// check if we succeeded
	if (frame.empty()) {
		cerr << "ERROR! blank frame grabbed\n";
		exit(1);
	}
	flip(frame, frame, -1);
	return frame;

}

Mat colorSegmentation(Mat frame, int minHue, int maxHue){
	Mat frame_threshold, imHSV;
	Mat kernel = (Mat_<unsigned char>(3,3) <<  0, 1, 0, 1, 1, 1, 0, 1, 0);
	cvtColor(frame, imHSV, COLOR_BGR2HSV);
	inRange(imHSV, Scalar(minHue, 70, 70), Scalar(maxHue, 255, 255), frame_threshold);
	morphologyEx(frame_threshold, frame_threshold, cv::MORPH_OPEN, kernel, Point(-1,-1), 5);

	//Mat kernel2 = getStructuringElement(MORPH_RECT, Size(10,10),Point(-1,-1));
	//morphologyEx(frame_threshold, frame_threshold, cv::MORPH_CLOSE, kernel2, Point(-1,-1), 4);
	
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<vector<Point> > contours_new;
    	findContours( frame_threshold, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
	for( size_t i = 0; i < contours.size(); i++ )
    	{	
        	if(hierarchy[i][3] == -1){
			contours_new.push_back(contours[i]);
		}
    	}
   	vector<vector<Point> >hull( contours_new.size() );
    	for( size_t i = 0; i < contours_new.size(); i++ )
    	{
        	convexHull( contours_new[i], hull[i] );
    	}	
	Mat frame_copy = frame.clone();
	drawContours(frame_copy, hull, -1, Scalar(255,255,255),-1);
	inRange(frame_copy, Scalar(254, 254, 254), Scalar(255, 255, 255), frame_threshold);
frame_show = frame_threshold;
	return frame_threshold;
}

Mat findYellow(Mat frame){
	Mat frame_threshold, imHSV;
	vector<vector<Point> > contours;
	vector<vector<Point> > hierarchy;
	Mat kernel = (Mat_<unsigned char>(3,3) <<  0, 1, 0, 1, 1, 1, 0, 1, 0);
	cvtColor(frame, imHSV, COLOR_BGR2HSV);
	inRange(imHSV, Scalar(20, 100, 140), Scalar(30, 255, 255), frame_threshold);
	morphologyEx(frame_threshold, frame_threshold, cv::MORPH_OPEN, kernel, Point(-1,-1), 5);
	return frame_threshold;
}

Mat findPurple(Mat frame){
	Mat frame_threshold, imHSV;
	vector<vector<Point> > contours;
	vector<vector<Point> > hierarchy;
	Mat kernel = (Mat_<unsigned char>(3,3) <<  0, 1, 0, 1, 1, 1, 0, 1, 0);
	cvtColor(frame, imHSV, COLOR_BGR2HSV);
	inRange(imHSV, Scalar(125, 60, 0), Scalar(145, 255, 255), frame_threshold);
	morphologyEx(frame_threshold, frame_threshold, cv::MORPH_OPEN, kernel, Point(-1,-1), 5);
	return frame_threshold;
}

int yellowPurpleMask(Mat frame){
	int width = frame.cols;
	int height = frame.rows;	
	int cropWidth = static_cast<int>(width*0.1);
	Rect roi(cropWidth, 0, width-2*cropWidth, height);
	Mat croppedFrame = frame(roi);
	Mat mask, yellow, purple;
	yellow = findYellow(croppedFrame);
	purple = findPurple(croppedFrame);
	bitwise_or(yellow, purple, mask);
	//cout << "purpleAndYellowPixels: " << countNonZero(mask) << endl;
	return countNonZero(mask);
}

Mat numberSegmentation(Mat frame, int minHue, int maxHue){
	Mat frame_threshold, imHSV, box, num;
	//Mat kernel = Mat::ones(3,3, CV_8U);
	Mat kernel = getStructuringElement(MORPH_RECT, Size(3,3),Point(-1,-1));
	cvtColor(frame, imHSV, COLOR_BGR2HSV);
	//GaussianBlur(imHSV, imHSV, Size(0,0), 1);
	inRange(imHSV, Scalar(minHue, 70, 70), Scalar(maxHue, 255, 255), frame_threshold);
	morphologyEx(frame_threshold, frame_threshold, cv::MORPH_OPEN, kernel, Point(-1,-1), 5);

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<vector<Point> > contours_new;
    	findContours( frame_threshold, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE );
	for( size_t i = 0; i < contours.size(); i++ )
    	{	
        	if(hierarchy[i][3] == -1){
			contours_new.push_back(contours[i]);
		}
    	}	
   	vector<vector<Point> >hull( contours_new.size() );
    	for( size_t i = 0; i < contours_new.size(); i++ )
    	{	
        	convexHull( contours_new[i], hull[i] );
    	}	
	Mat frame_copy = frame.clone();
	drawContours(frame_copy, hull, -1, Scalar(255,255,255),-1);
	inRange(frame_copy, Scalar(254, 254, 254), Scalar(255, 255, 255), frame_threshold);

	//Mat kernel2 = getStructuringElement(MORPH_RECT, Size(90,100),Point(-1,-1));
	//morphologyEx(frame_threshold, frame_threshold, cv::MORPH_CLOSE, kernel2, Point(-1,-1), 1);
	erode(frame_threshold, frame_threshold, kernel, Point(-1,-1), 3);

	bitwise_and(imHSV, imHSV, num, frame_threshold);
	inRange(num, Scalar(0, 0, 100), Scalar(180, 100, 255), frame_threshold);
	morphologyEx(frame_threshold, frame_threshold, cv::MORPH_OPEN, kernel, Point(-1,-1), 2);
/*
	imshow("hull", frame_copy);
	int key = cv::waitKey(5);
	key = (key==255) ? -1 : key;
	if (key == 'q'){
		exit(0);
	}
*/
	return frame_threshold;
}

double centroids(vector<vector<Point>> one, vector<vector<Point>> zero){
	vector<Point> oneCentroid;
	vector<Point> zeroCentroid;
	double xVal = 0;
	int counter = 0;
	int zeroPoint;
	for (int i = 0; i < (int)one.size()-1; i++){
		Moments mu = moments(one[i]);
		oneCentroid.push_back(Point(static_cast<float>(mu.m10 / (mu.m00 + 1e-5)),
			static_cast<float>(mu.m01 / (mu.m00 + 1e-5))));
		xVal += oneCentroid[i].x;
		counter += 1;
	}
	for (int i = 0; i < (int)zero.size()-1; i++){
		Moments mu = moments(zero[i]);
		zeroCentroid.push_back(Point(static_cast<float>(mu.m10 / (mu.m00 + 1e-5)),
			static_cast<float>(mu.m01 / (mu.m00 + 1e-5))));
		zeroPoint = zeroCentroid[i].x;
		if (zeroPoint < -150 || zeroPoint > 150){
			xVal += -zeroCentroid[i].x;
			counter += 1;
		}
		else if (zeroPoint < 0){
			xVal += zeroCentroid[i].x % 300;
			counter += 1;
		}
		else if (zeroPoint > 0){
			xVal += -zeroCentroid[i].x % 300;
			counter += 1;
		}
		else{
			xVal += 500;
			counter += 1;
		}
	}
	return xVal/counter;
}

// 100 px from both sides
double blockSearch(vector<vector<Point>> one, vector<vector<Point>> zero){
	if (one.size() > zero.size()*2){
		return 1;
	}
	else{
		return 0;
	}
	int state = 0;
	vector<Point> oneCentroid;
	vector<Point> zeroCentroid;
	Point tempZero;
	double zeroSum = 0;
	Point tempY = Point(0,500);
	for (int i = 0; i < (int)one.size(); i++){
		Moments mu = moments(one[i]);
		oneCentroid.push_back(Point(static_cast<float>(mu.m10 / (mu.m00 + 1e-5)),
			static_cast<float>(mu.m01 / (mu.m00 + 1e-5))));
		if (oneCentroid[i].x > 100 && oneCentroid[i].x < 540){
			state = 1;
			if (oneCentroid[i].y < tempY.y){
				tempY = oneCentroid[i];
			}
		}
	}
	for (int i = 0; i < (int)zero.size(); i++){
		Moments mu = moments(zero[i]);
		tempZero = Point(static_cast<float>(mu.m10 / (mu.m00 + 1e-5),
			static_cast<float>(mu.m01 / (mu.m00 + 1e-5))));
		if (tempZero.x > 50 && tempZero.x < 590){
			zeroCentroid.push_back(tempZero);
			zeroSum += tempZero.x;
			for ( int j = 0; j < (int)oneCentroid.size()-1; j++){
				if (oneCentroid[j].y < tempZero.y && oneCentroid[j].y > 50 && oneCentroid[j].y < 590){
					state = 1;

				}
				else{
					state = 0;
				}
			}
		}
	}
	if(state == 0){

		return 0;
		if (zeroSum/zeroCentroid.size() > 0){
			return -500;
		}
		else{
			return 500;
		}
	}
	else if(state == 1){

		return 1;
		return tempY.x;
	}
}

int trackBox(Mat frame_threshold, Mat frame){
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
	
	int posX = 1000;  
	if(blobSize != 0){
		mc.at(0) = Point2f( static_cast<float>(mu[index].m10 / (mu[index].m00 + 1e-5)),
					static_cast<float>(mu[index].m01 / (mu[index].m00 + 1e-5)) );
		circle( frame_threshold, mc[0], 4, Scalar(100, 200, 0), -1 );

		posX = static_cast<int>(mc[0].x) - 640/2;
		trueD = posX;
	}
/*
		//cout << "trueD " << trueD << endl;
		imshow("blues", frame_threshold);
		int key = cv::waitKey(5);
		key = (key==255) ? -1 : key;
		if (key == 'q'){
			exit(0);
		}
*/
//cout << "blobSize " << blobSize << endl;

	if (blobSize > 20000){
		return 500;		// condition to go to next state
	}
//cout << yellowPurpleMask(frame) << endl;
	if (yellowPurpleMask(frame) > 6000){
		//cout << "Too risky to go forward, spinning initiated! (frames = " << yellowPurpleMask(frame) << " )" <<endl;
		return 1000;
	}

	return posX;
}


vector<complex<double>> resample(vector<complex<double>> contour, double samples){
	int rate = (int)(1/(samples/contour.size()));
	vector<complex<double>> out;
	for(int i=0; i<(int)contour.size(); i+=rate){
		if ((double)out.size() == samples){
			return out;
		}
		out.push_back(contour[i]);
	}

	return out;
}

double contourDiff(vector<Point> contours){
	vector<complex<double>> fD, FD, ND;
	for (int i=0; i<(int)contours.size(); i++){
			fD.push_back((double)contours[i].x -(double)contours[i].y * j);
	}
	fD = resample(fD, sampling);
	dft(fD, FD);
	//FD = resample(FD, sampling);
	for (int i=2; i < (int)FD.size()*lpFilter; i++){
			ND.push_back(abs(FD[i])/abs(FD[1]));
	}
	double diff0 = norm(ND, refs[0]);
	double diff1 = norm(ND, refs[1]);
	if (diff0 <= diff1){
		return 0;
	}
	else{
		return 1;
	}
	return diff0/diff1; //
}

int checkContour(Mat frame){
	Mat thresh;
	vector<vector<Point> > unknown;
	vector<vector<Point> > one;
	vector<vector<Point> > zero;
	vector<vector<Point>> best; 
	size_t contourSize = 0;
	int sizeIndex;
	vector<Point> bigContour;
	best.push_back({{0,0}});
	double bestGuess = 0;
	thresh = numberSegmentation(frame,105,127);
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(thresh, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);

	for (int i=0; i<(int)contours.size(); i++){
		if((int)contours[i].size() < sampling){
			unknown.push_back(contours[i]);
		}
		else if(hierarchy[i][3] == -1){
			if (contours[i].size() > contourSize){
				contourSize = contours[i].size();
				bigContour = contours[i];
			}
			if(hierarchy[i][2] != -1){
				zero.push_back(contours[i]);
			}
			else{
				double guess = contourDiff(contours[i]);
				if (guess == 1){
					one.push_back(contours[i]);
				}
				else{
					zero.push_back(contours[i]);
				}
				if (guess > bestGuess){
					best[0] = contours[i];
					bestGuess = guess;
	
				}
			}
		}
	}
/*
	Mat frame_copy = frame.clone();
	if (one.size() > 0){
	drawContours(frame_copy, one, -1, Scalar(0,255,0),2);
	}
	if (best.size() > 0){
	//drawContours(frame_copy, best, 0, Scalar(0,0,255),2);
	}
	if (zero.size() > 0){
	drawContours(frame_copy, zero, -1, Scalar(255,0,0),2);
	}
	if (unknown.size() > 0){
	drawContours(frame_copy, unknown, -1, Scalar(255,255,0),2);
	}

	
	imshow("contours", frame_copy);
	int key = cv::waitKey(5);
	key = (key==255) ? -1 : key;
	if (key == 'q'){
		exit(0);
	}
*/
	if (bigContour.size() != 0){
		return round(contourDiff(bigContour));
	}
	return blockSearch(one, zero);
}


void setupFD(double samples){
	string ims[] = {"ref00.bmp","ref11.bmp"};
	for (int i=0; i<2; i++){
		Mat thresh = imread(ims[i]);
		cvtColor(thresh, thresh, COLOR_BGR2GRAY);
		threshold(thresh, thresh, 150, 255, THRESH_BINARY);
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours(thresh, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
		vector<complex<double>> fD, FD, nD, ND;
		for (int i=0; i<(int)contours[0].size(); i++){
			fD.push_back((double)contours[0][i].x -(double)contours[0][i].y * j);
		}
		fD = resample(fD, sampling);
		dft(fD,FD);
		//FD = resample(FD, samples);
		for (int i=2; i < (int)FD.size()*lpFilter; i++){
			ND.push_back(abs(FD[i])/abs(FD[1]));
		}
		refs[i] = ND;
	}
}

Mat cropImage(Mat frame){
	int width = frame.cols;
	int height = frame.rows;	
	Rect roi(0, 150, width, height-150);
	Mat croppedFrame = frame(roi);
	return croppedFrame;
}

int toMain(void){
	return boxPos;
}

//fil vector boxGuess with guesses of one or zero and return rounded average to main;
bool boxEval(int counts){
	
	Mat frame;	
	//VideoCapture cap = initCamera();
	float sum = 0;	
	for (int i = 0; i < counts; i++){
		frame = takePicture(cap);
		frame = cropImage(frame);
		sum += checkContour(frame);	
	}
//cout << "eval " << sum/counts << " counts " << counts << endl;
	if (sum/counts > 0.5){
		return true;
	}
	else{
		return false;
	}
}



//double boxDistance(Mat frame){
//	return frame.size;
//}

void camMain(void){
	mutex m;
	Mat frame, frame_return;
	setupFD(sampling);
	while(1){
		frame = takePicture(cap);
		frame = cropImage(frame);
		frame_return = colorSegmentation(frame, 105, 127);
		boxPos = trackBox(frame_return, frame);
//cout << "pixels " << countNonZero(frame_return) << endl;
		if (countNonZero(frame_return) > 7500){
			boxPos = 500;
		}
		//else if (countNonZero(frame_return) > 6000){
		//	boxPos = 2000;
		//}
	}

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
