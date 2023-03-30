/*
 * normal1.cpp
 *
 *  Created on: Mar 29, 2023
 *      Author: Erik
 */
#include <iostream>
#include "Eigen/Dense"

using Eigen::MatrixXd;

int not_main()
{
  MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << m << std::endl;
  return 1;
}



