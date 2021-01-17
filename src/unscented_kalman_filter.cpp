#include "unscented_kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

UnscentedKalmanFilter::UnscentedKalmanFilter() {}

UnscentedKalmanFilter::~UnscentedKalmanFilter() {}

void UnscentedKalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                                 MatrixXd &R_in, MatrixXd &Q_in) {
  /**
   * Init Initializes Unscented Kalman filter
   */
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  R_ = R_in;
  Q_ = Q_in;
}

void UnscentedKalmanFilter::Predict() {
  /**
   * Predict the state
   */
  int n_x = x_.rows();
  int n_aug = x_.rows() + Q_.rows();
  int n_aug_cnt = 2 * n_aug + 1;
  int lamda = 3 - n_aug;

  // create weight vector
  VectorXd weights = VectorXd(n_aug_cnt);
  weights(0) = 1.0 * lamda / (lamda + n_aug);
  for (int i = 1; i < weights.rows(); i++) {
     weights(i) = 1.0 / (2 * (lamda + n_aug));
  }

  VectorXd x_aug = VectorXd(n_aug);
  x_aug.fill(0.0);
  x_aug.head(x_.rows()) = x_;
  
  // create augmented covariance matrix
  P_aug_ = MatrixXd(n_aug, n_aug);
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(P_.rows(), P_.cols()) = P_;
  P_aug_.bottomRightCorner(Q_.rows(), Q_.cols()) = Q_;

  // create augmented sigma points
  MatrixXd sqrtMat = P_aug_.llt().matrixL();
  double sqrtValue = sqrt(lamda + n_aug);
  sqrtMat = sqrtValue * sqrtMat;
  x_aug_ = MatrixXd(n_aug, n_aug_cnt);
  x_aug_.col(0) = x_aug;
  for (int i = 0; i < n_aug; i++) {
      x_aug_.col(i+1) = x_aug + sqrtMat.col(i);
      x_aug_.col(n_aug+i+1) = x_aug - sqrtMat.col(i);
  }

  // predict sigma points
  x_sig_pred_ = MatrixXd(n_x, n_aug_cnt);
  for (int i = 0; i < n_aug_cnt; i++) {
      float px = x_aug_(0, i);
      float py = x_aug_(1, i);
      float v = x_aug_(2, i);
      float radian = x_aug_(3, i);
      float radian_v = x_aug_(4, i);
      float a = x_aug_(5, i);
      float radian_a = x_aug_(6, i);
      float dt = delta_t_;
      float dt2 = delta_t_ * delta_t_;
      if (radian_v == 0) {
          x_sig_pred_(0, i) = px + dt * v * cos(radian) + dt2 * cos(radian) * a / 2;
          x_sig_pred_(1, i) = py + dt * v * sin(radian) + dt2 * sin(radian) * a / 2;
      } else {
          x_sig_pred_(0, i) = px + 
                              v / radian_v * (sin(radian + dt * radian_v) - sin(radian)) +
                              dt2 * cos(radian) * a / 2;
          x_sig_pred_(1, i) = py +
                              v / radian_v * ((-cos(radian + dt * radian_v)) + cos(radian)) +
                              dt2 * sin(radian) * a / 2;
      }
      x_sig_pred_(2, i) = v + delta_t_ * a;
      x_sig_pred_(3, i) = radian + radian_v * delta_t_ + dt2 * radian_a / 2;
      x_sig_pred_(4, i) = radian_v + delta_t_ * radian_a;
  }

  // predict state mean
  VectorXd x = VectorXd(n_x);
  x.fill(0.0);
  for (int i = 0; i < n_aug_cnt; i++) {
      x = x + weights(i) * x_sig_pred_.col(i);
  }

  // predict state covariance matrix
  MatrixXd P = MatrixXd(n_x, n_x);
  P.fill(0.0);
  for (int i = 0; i < n_aug_cnt; i++) {
    MatrixXd x_diff = x_sig_pred_.col(i) - x;

    while (x_diff(3) > M_PI) {
      x_diff(3) -= 2.0 * M_PI;
    }
    while (x_diff(3) < -M_PI) {
      x_diff(3) += 2.0 * M_PI;
    }
    while (x_diff(4) > M_PI) {
      x_diff(4) -= 2.0 * M_PI;
    }
    while (x_diff(4) < -M_PI) {
      x_diff(4) += 2.0 * M_PI;
    }
    P = P + weights(i) * x_diff * x_diff.transpose();
  }

  x_ = x;
  P_ = P;
}

void UnscentedKalmanFilter::UpdateByLaserMeasurement(const VectorXd &z) {
  /**
   * update the state by using Laser Measurement
   */

  int n_x = x_.rows();
  int n_aug = x_.rows() + Q_.rows();
  int n_z = z.rows();
  int n_aug_cnt = 2 * n_aug + 1;
  int lamda = 3 - n_aug;

  // create weight vector
  VectorXd weights = VectorXd(n_aug_cnt);
  weights(0) = 1.0 * lamda / (lamda + n_aug);
  for (int i = 1; i < weights.rows(); i++) {
     weights(i) = 1.0 / (2 * (lamda + n_aug));
  }

  // transform sigma points into measurement space
  MatrixXd z_sig_pred = MatrixXd(n_z, 2 * n_aug + 1);
  for (int nCol = 0; nCol < n_aug_cnt; nCol++) {
      float px = x_sig_pred_(0, nCol);
      float py = x_sig_pred_(1, nCol);
      float v = x_sig_pred_(2, nCol);
      float radian = x_sig_pred_(3, nCol);
      float radian_v = x_sig_pred_(4, nCol);

      z_sig_pred(0, nCol) = px;
      z_sig_pred(1, nCol) = py;
  }

  // calculate mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int nCol = 0; nCol < n_aug_cnt; nCol++) {
      z_pred += weights(nCol) * z_sig_pred.col(nCol);
  }

  // calculate innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S = R_;
  for (int nCol = 0; nCol < n_aug_cnt; nCol++) {
      MatrixXd tempMat = z_sig_pred.col(nCol) - z_pred;
      S += weights(nCol) * tempMat * tempMat.transpose();
  }

  // calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x, n_z);
  MatrixXd x_diff = x_sig_pred_.colwise() - x_;
  MatrixXd z_diff = z_sig_pred.colwise() - z_pred;
  MatrixXd matrixWeight = MatrixXd(n_aug_cnt, n_aug_cnt);
  matrixWeight.fill(0.0);
  for (int nCol = 0; nCol < n_aug_cnt; nCol++) {
    while (x_diff(3, nCol) > M_PI) {
        x_diff(3, nCol) -= 2 * M_PI;
    }
    while (x_diff(3, nCol) < -M_PI) {
        x_diff(3, nCol) += 2 * M_PI;
    }
    while (x_diff(4, nCol) > M_PI) {
        x_diff(4, nCol) -= 2 * M_PI;
    }
    while (x_diff(4, nCol) < -M_PI) {
        x_diff(4, nCol) += 2 * M_PI;
    }
    matrixWeight(nCol, nCol) = weights(nCol);
  }
  Tc = x_diff * matrixWeight * z_diff.transpose();
  
  // calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // update state mean and covariance matrix
  x_ = x_ + K * (z - z_pred);
  P_ = P_ - K * S * K.transpose();
}

void UnscentedKalmanFilter::UpdateByRadarMeasurement(const VectorXd &z) {
  /**
   * update the state by using Extended Kalman Filter equations
   */
  
  int n_x = x_.rows();
  int n_aug = x_.rows() + Q_.rows();
  int n_z = z.rows();
  int n_aug_cnt = 2 * n_aug + 1;
  int lamda = 3 - n_aug;

  // create weight vector
  VectorXd weights = VectorXd(n_aug_cnt);
  weights(0) = 1.0 * lamda / (lamda + n_aug);
  for (int i = 1; i < weights.rows(); i++) {
     weights(i) = 1.0 / (2 * (lamda + n_aug));
  }

  // transform sigma points into measurement space
  MatrixXd z_sig_pred = MatrixXd(n_z, n_aug_cnt);
  for (int nCol = 0; nCol < n_aug_cnt; nCol++) {
      float px = x_sig_pred_(0, nCol);
      float py = x_sig_pred_(1, nCol);
      float v = x_sig_pred_(2, nCol);
      float radian = x_sig_pred_(3, nCol);
      float radian_v = x_sig_pred_(4, nCol);
      float fTemp = sqrt(px * px + py * py);
      z_sig_pred(0, nCol) = fTemp;
      z_sig_pred(1, nCol) = atan2(py, px);
      if (0 == fTemp) {
        z_sig_pred(2, nCol) = 0;
      } else {
        z_sig_pred(2, nCol) = (px * cos(radian) * v + py * sin(radian) * v) / fTemp;
      }
  }

  // calculate mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int nCol = 0; nCol < n_aug_cnt; nCol++) {
      z_pred += weights(nCol) * z_sig_pred.col(nCol);
  }
  
  // calculate innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S = R_;
  for (int nCol = 0; nCol < n_aug_cnt; nCol++) {
    VectorXd z_diff = z_sig_pred.col(nCol) - z_pred;
    while (z_diff(1) > M_PI) {
        z_diff(1) -= 2.*M_PI;
    }
    while (z_diff(1) < -M_PI) {
        z_diff(1) += 2.*M_PI;
    }
    S += weights(nCol) * z_diff * z_diff.transpose();
  }

  // calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x, n_z);
  MatrixXd x_diff = x_sig_pred_.colwise() - x_;
  MatrixXd z_diff = z_sig_pred.colwise() - z_pred;
  MatrixXd matrixWeight = MatrixXd(n_aug_cnt, n_aug_cnt);
  matrixWeight.fill(0.0);
  for (int nCol = 0; nCol < n_aug_cnt; nCol++) {
    while (z_diff(1, nCol) > M_PI) {
        z_diff(1, nCol) -= 2 * M_PI;
    }
    while (z_diff(1, nCol) < -M_PI) {
        z_diff(1, nCol) += 2 * M_PI;
    }
    while (x_diff(3, nCol) > M_PI) {
        x_diff(3, nCol) -= 2 * M_PI;
    }
    while (x_diff(3, nCol) < -M_PI) {
        x_diff(3, nCol) += 2 * M_PI;
    }
    matrixWeight(nCol, nCol) = weights(nCol);
  }
  Tc = x_diff * matrixWeight * z_diff.transpose();

  z_diff = z - z_pred;
  while (z_diff(1) > M_PI) {
      z_diff(1) -= 2 * M_PI;
  }
  while (z_diff(1) < -M_PI) {
      z_diff(1) += 2 * M_PI;
  }
  
  // calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // update state mean and covariance matrix
  x_ = x_ + K * (z_diff);
  P_ = P_ - K * S * K.transpose();
}
