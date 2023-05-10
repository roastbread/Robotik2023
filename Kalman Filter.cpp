//
// Created by danie on 2023-05-10.
//

#include "C:/Users/danie/CLionProjects/untitled2/eigen-3.4.0/Eigen/Eigen"
#include <iostream>
#include <fstream>

using namespace Eigen;
using namespace std;

int main(void) {

    double no_inputs = 500;
    for (int kk = 0; kk < no_inputs; kk++) {

        // position
        Matrix<double, 1, 500> X;
        Matrix<double, 1, 500> Y;
        Matrix<double, 1, 500> A;

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

        //Hard coded for now, its the uncertainty matrix from the Odometry.
        Matrix<double, 1, 9> P{
                {1, 0, 0, 0, 1, 0, 0, 0, pow((1 * 3.14 / 180), 2)},
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
        P.block(0, 0, 1, 3) = sigxHatPlus.row(0);
        P.block(0, 3, 1, 3) = sigxHatPlus.row(1);
        P.block(0, 6, 1, 3) = sigxHatPlus.row(2);

    }
}