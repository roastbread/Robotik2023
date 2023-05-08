#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace cv::ml;
using namespace std;

int main(int argc, char** argv)
{
    // creates necessary matrices and vectors.
    Mat train_data, train_labels;
    vector<String> train_files;

    //Takes care of the zero cases
    glob("C:/Users/danie/CLionProjects/untitled3/zeros/*.jpg", train_files, false);
    for (int i = 0; i < train_files.size(); i++) {
        Mat img = imread(train_files[i], IMREAD_GRAYSCALE);  //reads the image as grayscale.
        if (img.empty()) continue;
        resize(img, img, Size(28, 28));  // resize the image to 28x28, common with image processing and digit recognition.
        train_data.push_back(img.reshape(1, 1));  // pushes the image to the train_data object, which is transformed into a vector of dimensions 1x(28*28).
        train_labels.push_back(0);  // adds the image's corresponding label to the train_labels object.
    }

    //Takes care of the non_zero cases
    train_files.clear();
    glob("C:/Users/danie/CLionProjects/untitled3/non_zeros/*.jpg", train_files, false);
    for (int i = 0; i < train_files.size(); i++) {
        Mat img = imread(train_files[i], IMREAD_GRAYSCALE);  // resize the image to 28x28, common with image processing and digit recognition.
        if (img.empty()) continue;
        resize(img, img, Size(28, 28));  // resize the image to 28x28.
        train_data.push_back(img.reshape(1, 1));  // pushes the image to the train_data object, which is transformed into a vector of dimensions 1x(28*28).
        train_labels.push_back(1);  // adds the image's corresponding label to the train_labels object.
    }

    //Converts train_data to desired properties:
    train_data.convertTo(train_data, CV_32F); // Converts the train_data into (32-bit floating-point) to make it compatible with the SVM classifier.

    // Train the SVM classifier:
    Ptr<SVM> svm = SVM::create();  // Creates a SVM classifier and a 'smart pointer' -"Ptr" pointing at it.
    svm->setType(SVM::C_SVC);  // Sets the SVM classifier to C-Support Vector Classification.    (the C value can me modified later to control the seperation)
    svm->setKernel(SVM::LINEAR);  // Sets the separation boundary of the data points to linear.
    svm->train(train_data, ROW_SAMPLE, train_labels);  // Trains the classifier using the data and labels attained from the images.

    // Open camera
    VideoCapture cap(0);  // Opens the default camera.

    // Checks if the camera above is opened successfully.
    if (!cap.isOpened()) { cout << "Error opening video stream" << endl; return -1;}

    // Detects the digit '1' in real time using the above trained linear classifier.
    while (true) {
        // Captures a frame from the camera
        Mat frame;
        cap >> frame;

        // Converts the frame to grayscale
        Mat gray;
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // Resize the grayscale frame to 28x28
        resize(gray, gray, Size(28, 28));

        // Reshape the grayscale frame to a feature vector
        Mat test_data = gray.reshape(1, 1);

        // Classify the frame using the SVM classifier
        float response = svm->predict(test_data);

        // Print the classification result
        if (response == 0) {
            cout << "0" << endl;
        } else {
            cout << "1" << endl;
        }

        // Displays the camera feed
        imshow("Live Camera Feed", frame);

        // Exit if the user presses the 'q' key
        if (waitKey(1) == 'q')
        {
            break;
        }
    }

    // Release the camera and close all windows
    cap.release();
    destroyAllWindows();

    return 0;
}