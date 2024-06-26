// File to test the camera and C++ OpenCV libraries
// compile with
// g++ $(pkg-config --libs --cflags opencv) -o camera_test main.cpp
// run with
// ./cameratest

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>


using namespace cv;
using namespace std;

int maint(int argc,char ** argv)
{
  VideoCapture cap(0);
  if (!cap.isOpened()) {
    cerr << "ERROR: Unable to open the camera" << endl;
    return 0;
  }

  Mat frame;
  cout << "Start grabbing, press a key on Live window to terminate" << endl;
  

  while(1) {
    cap >> frame;
    if (frame.empty()) {
        cerr << "ERROR: Unable to grab from the camera" << endl;
        break;
    }
    imshow("Live",frame);
    int key = cv::waitKey(5);
    key = (key==255) ? -1 : key; //#Solve bug in 3.2.0
    if (key>=0)
      break;
  }
  cout << "Closing the camera" << endl;
  cap.release();
  destroyAllWindows();
  cout << "bye!" <<endl;
  return 0;
}



