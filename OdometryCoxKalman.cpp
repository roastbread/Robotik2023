//
// Created by danie on 2023-05-10.
//

#include "C:/Users/danie/CLionProjects/untitled2/eigen-3.4.0/Eigen/Eigen"
#include <iostream>
#include <fstream>
#include <cmath>

using namespace Eigen;
using namespace std;

int main(void) {

    bool kalman = false;
    double no_inputs = 500;  //for example

    for (int kk = 0; kk < no_inputs; kk++) {

        kalman = true;
        MatrixXd P{
                {1, 0, 0, 0, 1, 0, 0, 0, pow((1 * 3.14 / 180), 2)},
        };

        //creates the position matrices
        Matrix<double, 1, 500> X;
        Matrix<double, 1, 500> Y;
        Matrix<double, 1, 500> A;

        //if statement to run kalman
        if(kalman == true){

            // odometry differences from last pos
            double dx = 0;
            double dy = 0;
            double da = 0;

            //Hard coded for now, its the uncertainty matrix from the cox scan matching algorithm.
            Matrix<double, 2, 3> C{
                    {2, 3, 4},
                    {5, 6, 7},
                    {8, 9, 10},
            };

            // sigPF
            Matrix<double, 3, 3> sigPF;
            sigPF.row(0) = C.row(0).head(3);
            sigPF.row(1) = C.row(1).head(3);
            sigPF.row(2) = C.row(2).head(3);

            //sigxHat
            Matrix<double, 3, 3> sigxHat;
            sigxHat.row(0) = P.block(0, 0, 1, 3);
            sigxHat.row(1) = P.block(0, 3, 1, 3);
            sigxHat.row(2) = P.block(0, 6, 1, 3);

            //xHatPF
            Matrix<double, 3, 1> xHatPF;
            xHatPF(0,0) = X(kk - 1) + dx;
            xHatPF(1,0) = Y(kk - 1) + dy;
            xHatPF(2,0) = A(kk - 1) + da;

            //xHatMinus
            Matrix<double, 3, 1> xHatMinus;
            xHatMinus(0,0) = X(kk - 1);
            xHatMinus(1,0) = Y(kk - 1);
            xHatMinus(2,0) = A(kk - 1);

            //xHatPlus
            MatrixXd inv_sigPF_sigxHat = (sigPF + sigxHat).inverse();
            MatrixXd term1 = sigPF * inv_sigPF_sigxHat * xHatMinus;
            MatrixXd term2 = sigxHat * inv_sigPF_sigxHat * xHatPF;
            MatrixXd xHatPlus = term1 + term2;

            //updating pos with index kk-1
            X(kk-1) = xHatPlus(0);
            Y(kk-1) = xHatPlus(1);
            A(kk-1) = xHatPlus(2);

            //sigxHatPlus
            MatrixXd inv_sigPF = sigPF.inverse();
            MatrixXd inv_sigxHat = sigxHat.inverse();
            MatrixXd inv_sum = (inv_sigPF + inv_sigxHat).inverse();
            MatrixXd sigxHatPlus = inv_sum;

            //updating P
            MatrixXd subarray(1, 9);
            subarray <<  sigxHatPlus.row(0).segment(0, 3),
                    sigxHatPlus.row(1).segment(0, 3),
                    sigxHatPlus.row(2).segment(0, 3);
            P.row(kk - 1).segment(0, 9) = subarray;
        }

        double V = 0;
        double a = 0;
        double T = 0;
        double L = 0;

        double dD = V * cos(a) * T;
        double dA = (V * sin(a) * T) / L;
        double dX = dD * cos(A(kk - 1) + dA / 2);
        double dY = dD * cos(A(kk - 1) + dA / 2);

        X(kk) = X(kk - 1) + cos(a) * V * cos(A(kk - 1)) * T;
        Y(kk) = Y(kk - 1) + cos(a) * V * sin(A(kk - 1)) * T;
        A(kk) = static_cast<int>(A(kk - 1) + sin(a) * V * T / L) % static_cast<int>(2 * 3.14);

        Matrix<double, 3, 3> sigXYA;
        sigXYA.row(0) = P.block(0, 0, 1, 3);
        sigXYA.row(1) = P.block(0, 3, 1, 3);
        sigXYA.row(2) = P.block(0, 6, 1, 3);

        double k = 0.01;

        Matrix<double, 3, 3> sigVAT;
        sigVAT.row(0).segment(0, 3) << k * abs(V), 0, 0;
        sigVAT.row(1).segment(0, 3) << 0, k * abs(a), 0;
        sigVAT.row(2).segment(0, 3) << 0, 0, k * T;

        Matrix<double, 3, 3> jXYA;
        jXYA.row(0).segment(0, 3) << 1, 0, -T * V * sin(A(kk - 1) + (T * V * sin(a)) / (2 * L)) * cos(a);
        jXYA.row(1).segment(0, 3) << 0, 1, T * V * cos(A(kk - 1) + (T * V * sin(a)) / (2 * L));
        jXYA.row(2).segment(0, 3) << 0, 0, 1;

        Matrix<double, 3, 3> jVAT;
        jXYA.row(0).segment(0, 3) << 1, 0, -T * V * sin(A(kk - 1) + (T * V * sin(a)) / (2 * L)) * cos(a);
        jXYA.row(1).segment(0, 3) << 0, 1, T * V * cos(A(kk - 1) + (T * V * sin(a)) / (2 * L));
        jXYA.row(2).segment(0, 3) << 0, 0, 1;

        Matrix<double, 3, 3> jXYA_new;
        jXYA_new = jXYA * sigXYA * jXYA.transpose() + jVAT * sigVAT * jVAT.transpose();

        MatrixXd subarray(1, 9);
        subarray << jXYA_new.row(0).segment(0, 3),
                jXYA_new.row(1).segment(0, 3),
                jXYA_new.row(2).segment(0, 3);
        P.row(kk).segment(0, 9) = subarray;
    }
}