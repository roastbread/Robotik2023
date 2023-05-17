//
// Created by danie on 2023-05-10.
//

#include "C:/Users/danie/CLionProjects/untitled2/eigen-3.4.0/Eigen/Eigen"
#include <iostream>
#include <fstream>
#include <cmath>
#include "coxRemade.cpp"

using namespace Eigen;
using namespace std;


coxMap createCoxInstance(){
    coxMap map;
    map.setup();
    std::cout << "Here are the vectors u1:\n" << map.ri[0]  << std::endl;
    while(Array_full_flag == 0 || !needData){}
    needData = false;
    map.loop(100, map.posX, map.posY, map.posA);
    Array_full_flag = 0;
    coxDone = true;
    std::cout << "Position Fix: X=" << posFix(0) << " Y=" << posFix(1) << " A=" << posFix(2) << std::endl;
    std::cout << "Position: X=" << map.posX << " Y=" << map.posY << " A=" << map.posA << std::endl;
    std::cout << "Uncertainty: X=" << map.sigX << " Y=" << map.sigY << " A=" << map.sigA << std::endl;
    cout << "Finished" << endl;
    return map;
}

int main(void) {

    double WHEEL_BASE = 53;
    double WHEEL_DIAMETER = 15.3;
    double PULSES_PER_REVOLUTION = 600;
    double CCP = 3.14*WHEEL_DIAMETER;
    double MM_PER_PULSE = CCP/PULSES_PER_REVOLUTION;
    double SIGMA_WHEEL_ENCODER = 0.5/12;
    double SIGMAl = SIGMA_WHEEL_ENCODER;
    double SIGMAr = SIGMA_WHEEL_ENCODER;

    // Dr = ENC(1:10:end,2) * MM_PER_PULSE;
    // Dl = ENC(1:10:end,1) * MM_PER_PULSE;

    Eigen::VectorXd Dr;
    Dr << 0;
    Eigen::VectorXd Dl;
    Dl << 0;


    int N = Dr.size(); //riktiga
    N = 500; //hårdkodad

    Eigen::VectorXd X(N);
    Eigen::VectorXd Y(N);
    Eigen::VectorXd A(N);

    X(0) = 0;
    Y(0) = 0;
    A(0) = 90 * 3.14 / 180;

    MatrixXd P{
            {1, 0, 0, 0, 1, 0, 0, 0, pow((1 * 3.14 / 180), 2)},
    };

    bool kalman = false;
    double no_inputs = 500;  //for example

    for (int kk = 2; kk < N; kk++) {

        double dDr = Dr(kk) - Dr(kk - 1);
        double dDl = Dl(kk) - Dl(kk - 1);

        double dD = (dDr + dDl) / 2.0;
        double dA = (dDr - dDl)/WHEEL_BASE;

        double dX = dD * cos(A(kk - 1) + dA / 2.0); //without the compensation term
        double dY = dD * sin(A(kk - 1) + dA / 2.0); //without the compensation term

        //double dX = (sin(dA / 2.0) / (dA / 2.0)) * dD * cos(A(kk - 1) + dA / 2.0); //with the compensation term
        //double dY = (sin(dA / 2.0) / (dA / 2.0)) * dD * sin(A(kk - 1) + dA / 2.0); //with the compensation term

        X(kk) = X(kk - 1) + dX;
        Y(kk) = Y(kk - 1) + dY;
        A(kk) = fmod(A(kk - 1) + dA, 2.0 * 3.14);

        Matrix3d Cxya_old;
        Cxya_old.row(0) = P.row(kk - 1).segment<3>(0);
        Cxya_old.row(1) = P.row(kk - 1).segment<3>(3);
        Cxya_old.row(2) = P.row(kk - 1).segment<3>(6);

        double CV = (SIGMAr * SIGMAr - SIGMAl * SIGMAl) / (2.0 * WHEEL_BASE);
        Matrix2d Cu;
        Cu << (SIGMAr * SIGMAr - SIGMAl * SIGMAl) / 4.0, CV,
                CV, (SIGMAr * SIGMAr - SIGMAl * SIGMAl) / (WHEEL_BASE * WHEEL_BASE);

        Matrix3d Axya;
        Axya << 1, 0, -dY,
                0, 1, dX,
                0, 0, 1;

        Matrix<double, 3, 2> JdRdL;
        JdRdL << 0.5 * cos(A(kk - 1) + dA / 2.0) - (dD / (2.0 * WHEEL_BASE)) * sin(A(kk - 1) + dA / 2.0),
                0.5 * cos(A(kk - 1) + dA / 2.0) + (dD / (2.0 * WHEEL_BASE)) * sin(A(kk - 1) + dA / 2.0),

                0.5 * sin(A(kk - 1) + dA / 2.0) + (dD / (2.0 * WHEEL_BASE)) * cos(A(kk - 1) + dA / 2.0),
                0.5 * sin(A(kk - 1) + dA / 2.0) - (dD / (2.0 * WHEEL_BASE)) * cos(A(kk - 1) + dA / 2.0),

                1.0 / WHEEL_BASE,
                -1.0 / WHEEL_BASE;

        double SigmaB = 0.1;
        double k = pow(0.01,2);

        Matrix2d Cdrdl;
        Cdrdl << k * abs(dDr), 0,
                 0, k * abs(dDl);

        double dDr_dDl = (dDr - dDl) / (2.0 * WHEEL_BASE);
        Vector3d jBB;
        jBB << ((dDr + dDl) / 2.0) * dDr_dDl * sin(A(kk - 1) + dDr_dDl),
                -((dDr + dDl) / 2.0) * dDr_dDl * cos(A(kk - 1) + dDr_dDl),
                -dDr_dDl / (WHEEL_BASE * WHEEL_BASE);

        Matrix3d Au;
        Au << cos(A(kk - 1) + dA / 2.0), -0.5 * dD * sin(A(kk - 1) + dA / 2.0),
                sin(A(kk - 1) + dA / 2.0), 0.5 * dD * cos(A(kk - 1) + dA / 2.0),
                0, 1;

        Matrix3d Cxya_new = Axya * Cxya_old * Axya.transpose() +
                                    JdRdL * Cdrdl * JdRdL.transpose() +
                                    jBB * SigmaB * jBB.transpose();

        kalman = true;

        //if statement to run kalman
        if(kalman == true){

            coxMap cox = createCoxInstance();

            // cox differences from last pos
            double dx = cox.posX;
            double dy = cox.posY;
            double da = cox.posA;

            //Hard coded for now, its the uncertainty matrix from the cox scan matching algorithm.
            Matrix<double, 3, 3> C;
            C << cox.covariance;

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

        /*
        //creates the position matrices
        Matrix<double, 1, 500> X;
        Matrix<double, 1, 500> Y;
        Matrix<double, 1, 500> A;

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
        jVAT.row(0).segment(0, 3) <<
                T*cos(A(kk-1)+(T*V*sin(a))/(2*L))*cos(a)-(pow(T,2)*V*sin(A(kk-1)+(T*V*sin(a))/(2*L))*cos(a)*sin(a))/(2*L),
                -T*V*cos(A(kk-1)+(T*V*sin(a))/(2*L))*sin(a)-(pow(T,2)*pow(V,2)*sin(A(kk-1)+(T*V*sin(a))/(2*L))*pow(cos(a),2))/(2*L),
                V*cos(A(kk-1)+(T*V*sin(a))/(2*L))*cos(a)-(T*pow(V,2)*sin(A(kk-1)+(T*V*sin(a))/(2*L))*cos(a)*sin(a))/(2*L);

        jVAT.row(1).segment(0, 3) <<
                T*sin(A(kk-1)+(T*V*sin(a))/(2*L))*cos(a)+(pow(T,2)*V*cos(A(kk-1)+(T*V*sin(a))/(2*L))*cos(a)*sin(a))/(2*L),
                (pow(T,2)*pow(V,2)*cos(A(kk-1)+(T*V*sin(a))/(2*L))*pow(cos(a),2))/(2*L)-T*V*sin(A(kk-1)+(T*V*sin(a))/(2*L))*sin(a),
                V*sin(A(kk-1)+(T*V*sin(a))/(2*L))*cos(a)+(T*pow(V,2)*cos(A(kk-1)+(T*V*sin(a))/(2*L))*cos(a)*sin(a))/(2*L);

        jVAT.row(2).segment(0, 3) <<
                (T*sin(a))/L,
                (T*V*cos(a))/L,
                (V*sin(a))/L;

        Matrix<double, 3, 3> jXYA_new;
        jXYA_new = jXYA * sigXYA * jXYA.transpose() + jVAT * sigVAT * jVAT.transpose();

        MatrixXd subarray(1, 9);
        subarray <<
                jXYA_new.row(0).segment(0, 3),
                jXYA_new.row(1).segment(0, 3),
                jXYA_new.row(2).segment(0, 3);
        P.row(kk).segment(0, 9) = subarray;

         */
    }
}