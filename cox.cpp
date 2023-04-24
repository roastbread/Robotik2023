/*
 * coxAlgorithm.cpp
 *
 *  Created on: Apr 18, 2023
 *      Author: Erik
 */


#include <stdio.h>
#include <stdlib.h>
#include "Eigen/Dense"
#include <iostream>
#include <fstream>
#include <math.h>

class coxMap{
public:
    Eigen::Matrix<double, 1, 400> x;
    Eigen::Matrix<double, 1, 400> y;
    Eigen::Matrix<double, 1, 400> DIS;
    Eigen::Matrix<double, 1, 400> ang;
    double sensorPose[3] = {660 ,0,-90*M_PI/180};
    float prevX;
    float prevY;
    float prevA;

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

    Eigen::Matrix<float, 4, 4> line {
            {559, -1140, 2856, 910},
            {2856, 910, 1336, 2670},
            {1336, 2670, -1004, 610},
            {-1004, 610, 559, -1140},
    };
    Eigen::Matrix<float, 2, 2> rotMat{
            {0, -1},
            {1, 0},
    };
    Eigen::Vector2<float> Li[4];
    Eigen::Vector2<float> ui[4];
    float ri[4];

    void gather_data(){
        DIS.setZero();
        ang.setZero();
    }

    void loop(int max_iterations){
        for (int i = 0; i++; i < max_iterations ){
            //sensor to Cartesian coordinates
            x = DIS*cos(ang);
            y = DIS*sin(ang);

            //sensor to Robot coordinates

            int n = max(x.rows(), x.cols());

            Eigen::Matrix<double, 3, 3> rotMatRobot {
                    {cos(sensorPose[3]), -1*sin(sensorPose[3]), sensorPose[1]},
                    {sin(sensorPose[3]), cos(sensorPose[3]), sensorPose[2]},
                    {0, 0, 1},
            };

            Eigen::Matrix<float, 3, n> xy1T;
            xy1T.row(0) = x;
            xy1T.row(1) = y;
            xy1T.row(2).setZero();

            Eigen::Matrix<double, 3, n> Xs;

            Xs = rotMatRobot*test;

            //Robot coordinates to world coordinates
            Eigen::Matrix<double, 3, 3> rotMatWorld {
                    {cos(prevA), -1*sin(prevA), prevX},
                    {sin(prevA), cos(prevA), prevY},
                    {0, 0, 1},
            };

            Eigen::Matrix<double, 3, n> Xw;

            Xw = rotMatWorld*Xs;

            // Step 2 Find targets for data points
            // V is the number of points from the laser scan

            Eigen::Transpose<float, 2, n > V;
            V.row(0) = Xw.row(0);
            V.row(1) = Xw.row(1);

            Eigen::Transpose<float, 2, n > dist_All;
            dist_All.row(0).setZero();
            dist_All.row(1).setZero();

            Eigen::Transpose<float, 2, n > qi; //qi == u1 från matlab cox
            Qi.row(0).setZero();
            Qi.row(1).setZero();

            Eigen::Transpose<float, 1, n > r;
            r.row(0).setZero();

            Eigen::Transpose<float, 1, n > yi;
            y1.row(0).setZero();

            for (int i = 0; i < V.cols(); i++) {
                Eigen::Transpose<float, 1, 4 > dist;
                for (int j = 0; i < line.cols(); i++) {
                    // Tar ut distansen från laser scan till väggarna
                    dist(j) = pointLineDistance(V(0, i), V(1, i), line(j, 0), line(j, 1), line(j, 2), line(j, 3));
                }

                Eigen::Index index;
                double minVal = dist.minCoeff(&index);

                dist_all.col(i) << minVal, index;
                qi.col(i) = ui.row(index).transpose();

                Eigen::VectorXd z = line.row(index).head<2>();
                for (int i = 0; i < N; i++) {
                    r(i) = qi.col(i).dot(z);
                }

                yi(i) = r(i) - qi.col(i).dot(V.col(i));
            }

            //step 3

            Eigen::Matrix<float, 2, 2> M;
            M << 0, -1,
                 1, 0;

            Eigen::Matrix<double, 2, 1> vm(prevX, prevY);

            Eigen::VectorXf xi3(length(V));
            for (int g = 0; g < length(V); g++) {
                xi3(g) = qi.col(g).transpose() * M * (V.col(g) - vm);
            }

            Eigen::VectorXf tempx = dist_all.row(0);

            // Calculate the median using std::nth_element
            auto middle = tempx.begin() + tempx.size() / 2;
            std::nth_element(tempx.begin(), middle, tempx.end());
            float threshold = *middle;

            std::vector<int> outliers;
            for (int i = 0; i < tempx.size(); ++i) {
                if (tempx(i) > threshold) {
                    outliers.push_back(i);
                }
            }

            // Remove outliers from the matrices
            for (int i = outliers.size() - 1; i >= 0; --i) {
                int index = outliers[i];

                qi.col(index).swap(qi.col(qi.cols() - 1));
                qi.conservativeResize(qi.rows(), qi.cols() - 1);

                yi.col(index).swap(yi.col(yi.cols() - 1));
                yi.conservativeResize(yi.rows(), yi.cols() - 1);

                r.col(index).swap(r.col(r.cols() - 1));
                r.conservativeResize(r.rows(), r.cols() - 1);

                xi3.col(index).swap(xi3.col(xi3.cols() - 1));
                xi3.conservativeResize(xi3.rows(), xi3.cols() - 1);

                Xw.col(index).swap(Xw.col(Xw.cols() - 1));
                Xw.conservativeResize(Xw.rows(), Xw.cols() - 1);
            }

            Eigen::VectorXf xi1 = ui.row(0);
            Eigen::VectorXf xi2 = ui.row(1);

            Eigen::MatrixXf A(tempx.size(), 3);
            A.col(0) = xi1;
            A.col(1) = xi2;
            A.col(2) = xi3;
            A.transpose();

            Eigen::MatrixXf AtA = A.transpose() * A;
            Eigen::MatrixXf AtYi = A.transpose() * yi.transpose();
            Eigen::MatrixXf b = AtA.inverse() * AtYi;

            Eigen::VectorXf residual = yi - A * b;

            float S2 = (residual.transpose() * residual) / (n - 4);

            Eigen::MatrixXf C = S2 * (A.transpose() * A).inverse();

            // Step 4
            ddx += b(0);
            ddy += b(1);
            dda += b(2);

            prevX = prevX + b(1);
            prevY = prevY + b(2);
            prevA = prevA + b(3);

            // Step 5
            if (sqrt(pow(b(0), 2) + pow(b(1), 2)) < 5 && abs(b(2)) < 0.1 * M_PI / 180) {
                break;
            }

        }
    }

};

double pointLineDistance(double x0, double y0, double x1, double y1, double x2, double y2) {
    double dist = abs((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1) / sqrt(pow(y2-y1, 2) + pow(x2-x1, 2));
    return dist;
}

int main(void){
    coxMap map;
    map.setup();
    std::cout << "Here are the vectors u1:\n" << map.ri[0]  << std::endl;





}