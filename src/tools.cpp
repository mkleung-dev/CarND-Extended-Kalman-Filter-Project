#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * Calculate the RMSE here.
   */

  // Check Inputs.
  if (estimations.size() == 0) {
     std::cout << "Tools::CalculateRMSE() - Error - "
                  "estimations size is 0." << 
                  std::endl;
     return VectorXd();
  }
  if (ground_truth.size() == 0) {
     std::cout << "Tools::CalculateRMSE() - Error - "
                  "ground_truth size is 0." << 
                  std::endl;
     return VectorXd();
  }
  if (estimations.size() != ground_truth.size()) {
     std::cout << "Tools::CalculateRMSE() - Error - "
                  "estimations size is not equal to ground_truth size." << 
                  std::endl;
     return VectorXd();
  }

  // Compute Root Mean Square.
  VectorXd rmse = VectorXd::Zero(estimations[0].rows());
  for (int i = 0; i < estimations.size(); i++)
  {
     VectorXd temp = estimations[i].array() - ground_truth[i].array();
     temp = temp.array() * temp.array();
     rmse += temp;
  }
  rmse = rmse / estimations.size();
  return rmse.array().sqrt();
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3, 4);
  // Check Inputs.
  if (x_state.rows() != 4) {
      std::cout << "Tools::CalculateJacobian() - Error - "
                   "x_state rows is not 4." << 
                   std::endl;
      return Hj;
  }

  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // Return 0 matrix if px is 0 or py is 0.
  if (px == 0 && py == 0) {
      Hj << 0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0;
      return Hj;
  }

  // Calculate a Jacobian here.
  float fTemp1 = px * px + py * py;
  float fTemp2 = sqrt(fTemp1);
  float fTemp3 = fTemp1 * fTemp2;

  Hj(0, 0) = px / fTemp2;
  Hj(0, 1) = py / fTemp2;
  Hj(0, 2) = 0;
  Hj(0, 3) = 0;

  Hj(1, 0) = -py / fTemp1;
  Hj(1, 1) = px / fTemp1;
  Hj(1, 2) = 0;
  Hj(1, 3) = 0;

  Hj(2, 0) = py * (vx * py - vy * px) / fTemp3;
  Hj(2, 1) = px * (vy * px - vx * py) / fTemp3;
  Hj(2, 2) = Hj(0, 0);
  Hj(2, 3) = Hj(0, 1);

  return Hj;
}
