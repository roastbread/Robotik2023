#include <stdio.h>
#include <stdlib.h>
#include "C:/Users/danie/CLionProjects/untitled2/eigen-3.4.0/Eigen/Eigen"
#include <iostream>
#include <fstream>
#include <math.h>
#include <string>

using namespace Eigen;
using namespace std;

class coxMap{

public:
    //float dist[400];
    //float ang[400];
    //double sensorPose[] = {1000000,20000000,3000000};
    Matrix<double, 1, 400> x;
    Matrix<double, 1, 400> y;
    Matrix<double, 1, 400> DIS;
    Matrix<double, 1, 400> ang;
    double sensorPose[3] = {660 ,0,-90*M_PI/180};
    double prevX = 0;
    double prevY = 0;
    double prevA = 0;

    double ddx = 0;
    double ddy = 0;
    double dda = 0;

    void setup(){
        for (int i=0; i<line.rows(); i++){
            ui[i] << line(i,0) - line(i,2), line(i,1) - line(i,3);
            ui[i] = rotMat*ui[i]/ui[i].norm();
            Li[i] << line(i,0), line(i,1);
            ri[i] = ui[i].dot(Li[i]);
        }
    }

    Matrix<double, 4, 4> line {
            {559, -1140, 2856, 910},
            {2856, 910, 1336, 2670},
            {1336, 2670, -1004, 610},
            {-1004, 610, 559, -1140},
    };
    Matrix<double, 2, 2> rotMat{
            {0, -1},
            {1, 0},
    };
    Vector2<double> Li[4];
    Vector2<double> ui[4];
    double ri[4];

    void gather_data(){
        cout << "Entered 'gather_data()' function" << std::endl;
        string line2;
        string quality;
        string angle;
        string distance;
        fstream data;
        data.open("testfile.txt");
        data.seekg(-12*410, std::ios_base::end);
        for (int i=0; i<=400;i++){
            getline(data, line2);
            if(line2.length()>4){
                size_t pos1 = line2.find_first_of(' ');
                quality = line2.substr(0,pos1);
                size_t pos2 = line2.find_first_of(' ', pos1+1);
                angle = line2.substr(pos1+1,pos2-2);
                distance = line2.substr(pos2+1);
                cout << quality << "|" << angle << "|" << distance << "|" << i << endl;
                ang(0,i-1) = stod(angle);
                DIS(0,i-1) = stod(distance);
            }
        }
        //std::remove("testfile.txt");
        data.close();
        //std::ofstream empty;
        //empty.open("textfile.txt");
        //std::rename("textfile.txt","testfile.txt");
        //empty.close();
    }
    void loop(int max_iterations){
        cout << "Entered 'loop(int max_iterations)' function" << std::endl;
        for (int i = 0; i < max_iterations; i++ ){

            //sensor to Cartesian coordinates
            x = DIS.array() * cos(ang.array());
            y = DIS.array() * sin(ang.array());

            //sensor to Robot coordinates
            Matrix<double, 3, 3> rotMatRobot {
                    {cos(sensorPose[2]), -1*sin(sensorPose[2]), sensorPose[0]},
                    {sin(sensorPose[2]), cos(sensorPose[2]), sensorPose[1]},
                    {0, 0, 1},
            };
            
            long long n = x.cols();
            Matrix<double, 3, 400> xy1T;
            xy1T.row(0) = x;
            xy1T.row(1) = y;
            xy1T.row(2).setZero();

            //std::cout << DIS << std::endl;

            Matrix<double, 3, 400> Xs;
            Xs = rotMatRobot*xy1T;

            //Robot coordinates to world coordinates
            Matrix<double, 3, 3> rotMatWorld {
                    {cos(prevA), -1*sin(prevA), prevX},
                    {sin(prevA), cos(prevA), prevY},
                    {0, 0, 1},
            };

            MatrixXd Xw;

            Xw = rotMatWorld*Xs;

            // Step 2 Find targets for data points
            // V is the number of points from the laser scan
            Matrix<double, 2, 400 > V;
            V.row(0) = Xw.row(0);
            V.row(1) = Xw.row(1);

            Matrix<double, 2, 400 > dist_All;
            dist_All.row(0).setZero();
            dist_All.row(1).setZero();

            MatrixXd qi; //qi == u1 från matlab cox
            qi.row(0).setZero();
            qi.row(1).setZero();

            MatrixXd r;
            r.row(0).setZero();

            MatrixXd yi;
            yi.row(0).setZero();

            for (int k = 0; k < V.cols(); k++) {
                Matrix<double, 1, 4 > dist;
                for (int j = 0; j < line.cols(); j++) {
                    // Tar ut distansen från laser scan till väggarna
                    dist(j) = pointLineDistance(V(0, k), V(1, k), line(j, 0), line(j, 1), line(j, 2), line(j, 3));
                }

                Index index;
                double minVal = dist.minCoeff(&index);

                dist_All.col(i) << minVal, index;
                qi.col(i) = qi.row(index).transpose();

                //z
                VectorXd z = line.row(index).head<2>();

                //r
                for (int j = 0; j < qi.cols(); j++) {
                    r(j) = qi.col(j).dot(z);
                }

                //yi
                for (int j = 0; j < qi.cols(); j++) {
                    yi(j) = r(j) - qi.col(j).dot(V.col(j));
                }
            }

            //step 3

            Matrix<double, 2, 2> M;
            M << 0, -1, 1, 0;

            Matrix<double, 2, 1> vm{
                    {prevX},
                    {prevY},
            };

            VectorXd xi3(V.size());
            xi3.setZero();

            for (int g = 0; g < 400; g++) {
                xi3(g) = qi.col(g).transpose() * M * (V.col(g) - vm);
            }

            VectorXd tempx = dist_All.row(0).transpose();

            // Calculate the median using std::nth_element
            auto middle = tempx.begin() + tempx.size() / 2;
            nth_element(tempx.begin(), middle, tempx.end());
            double threshold = *middle;

            vector<long long> outliers;
            for (int j = 0; j < tempx.size(); ++j) {
                if (tempx(j) > threshold) {
                    outliers.push_back(j);
                }
            }
            // Remove outliers from the matrices
            for (int j = 0; j < outliers.size(); j++) {
                long long outlierIndex = outliers[j];
                qi.col(outlierIndex).swap(qi.col(qi.cols()-1)); // Swap outlier column with last column
                qi.conservativeResize(qi.rows(), qi.cols()-1); // Remove last column
                yi.col(outlierIndex).swap(yi.col(yi.cols()-1));
                yi.conservativeResize(yi.rows(), yi.cols()-1);
                xi3.col(outlierIndex).swap(xi3.col(xi3.cols()-1));
                xi3.conservativeResize(xi3.rows(), xi3.cols()-1);
                Xw.col(outlierIndex).swap(Xw.col(Xw.cols()-1));
                Xw.conservativeResize(Xw.rows(), Xw.cols()-1);
            }

            MatrixXd xi1 = qi.row(0);
            MatrixXd xi2 = qi.row(1);

            MatrixXd A(tempx.size(), 3);
            A.col(0) = xi1;
            A.col(1) = xi2;
            A.col(2) = xi3;
            A.transpose();

            MatrixXd AtA = A.transpose() * A;
            MatrixXd AtYi = A.transpose() * yi.transpose();
            MatrixXd b = AtA.inverse() * AtYi;

            VectorXd residual = yi.transpose() - A * b;

            MatrixXd S2 = ((yi.transpose() - A * b).transpose() * (yi.transpose() - A * b)) / (n - 4);

            MatrixXd C = S2 * (A.transpose() * A).inverse();

            // Step 4
            ddx += b(0);
            ddy += b(1);
            dda += b(2);

            prevX += b(1);
            prevY += b(2);
            prevA += b(3);

            // Step 5
            if (sqrt(pow(b(0), 2) + pow(b(1), 2)) < 5 && abs(b(2)) < 0.1 * M_PI / 180) {
                break;
            }
        }
    }

    double pointLineDistance(double x0, double y0, double x1, double y1, double x2, double y2) {
        double dist = abs((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1) / sqrt(pow(y2-y1, 2) + pow(x2-x1, 2));
        return dist;
    }
};

int main(void){
    coxMap map;
    map.setup();
    //std::cout << "Here are the vectors u1:\n" << map.ri[0]  << std::endl;
    map.gather_data();
    map.loop(1);
    cout << "test";
    std::cout << "The code has compiled succesfully! :D" << map.ri[0]  << std::endl;
}