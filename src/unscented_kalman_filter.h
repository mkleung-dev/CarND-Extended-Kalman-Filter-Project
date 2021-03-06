#ifndef UNSCENTED_KALMAN_FILTER_H_
#define UNSCENTED_KALMAN_FILTER_H_

#include "Eigen/Dense"

class UnscentedKalmanFilter {
 public:
  /**
   * Constructor
   */
  UnscentedKalmanFilter();

  /**
   * Destructor
   */
  virtual ~UnscentedKalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
            Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   */
  void Predict();

  /**
   * Updates the state by using Laser Measurement
   * @param z The measurement at k+1
   */
  void UpdateByLaserMeasurement(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Radar Measurement
   * @param z The measurement at k+1
   */
  void UpdateByRadarMeasurement(const Eigen::VectorXd &z);
  
  // state vector
  Eigen::VectorXd x_;
  Eigen::MatrixXd x_aug_;
  Eigen::MatrixXd x_sig_pred_;

  // state covariance matrix
  Eigen::MatrixXd P_;
  Eigen::MatrixXd P_aug_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;

  // time for each prediction and update
  float delta_t_;
};

#endif // KALMAN_FILTER_H_
