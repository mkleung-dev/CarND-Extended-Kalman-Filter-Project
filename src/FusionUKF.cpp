#include "FusionUKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionUKF::FusionUKF(bool bUseLaser, bool bUseRadar) : FusionKF(bUseLaser, bUseRadar) {
  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225,      0,
                   0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09,      0,    0,
                 0, 0.0009,    0,
                 0,      0, 0.09;

  /**
   * Finish initializing the FusionUKF.
   * Set the process and measurement noises
   */
  ukf_ = UnscentedKalmanFilter();
  /**
   * Proces Noise Q (2 x 2):
   * longitudinal acceleration noise,                      0
   *                               0, yaw acceleration noise
   * longitudinal acceleration noise: Range +- 4 m/s^2
   * yaw acceleration noise: Range +- 0.6 rad/s^2
   */
  ukf_.Q_ = MatrixXd(2, 2);
  ukf_.Q_ << 4,    0,
             0, 0.09;
}

/**
 * Destructor.
 */
FusionUKF::~FusionUKF() {}

void FusionUKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * Initialize the state ukf_.x_ with the first measurement.
     * Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    // px, py, velocity, angle, angular velocity;
    ukf_.x_ = VectorXd(5);
    ukf_.x_ << 1, 1, 1, 1, 1;

    ukf_.P_ = MatrixXd(5, 5);
    ukf_.P_ <<    1,    0,    0,    0,    0,
                  0,    1,    0,    0,    0,
                  0,    0,    1,    0,    0,
                  0,    0,    0,    1,    0,
                  0,    0,    0,    0,    1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      ukf_.x_ << measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]),
                 measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]),
                 0,
                 0,
                 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Initialize state.
      ukf_.x_ << measurement_pack.raw_measurements_[0],
                 measurement_pack.raw_measurements_[1],
                 0,
                 0,
                 0;
   }
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }
  if (!bUseRadar_ && measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    return;
  }
  if (!bUseLaser_ && measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    return;
  }

  /**
   * Prediction
   */

  /**
   * Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  ukf_.delta_t_ = dt;

  ukf_.Predict();

  /**
   * Update
   */

  /**
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ukf_.R_ = R_radar_;
    ukf_.UpdateByRadarMeasurement(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ukf_.R_ = R_laser_;
    ukf_.UpdateByLaserMeasurement(measurement_pack.raw_measurements_);
  }
  previous_timestamp_ = measurement_pack.timestamp_;

  // print the output
  cout << "x_ = " << endl << ukf_.x_ << endl;
  cout << "P_ = " << endl << ukf_.P_ << endl;
}

/**
 * Return the estimated position x.
 */
float FusionUKF::GetPx() {
  return ukf_.x_(0);
}

/**
 * Return the estimated position y.
 */
float FusionUKF::GetPy() {
  return ukf_.x_(1);
}

/**
 * Return the estimated velocity x.
 */
float FusionUKF::GetVx() {
  return ukf_.x_(2) * cos(ukf_.x_(3));
}

/**
 * Return the estimated velocity y.
 */
float FusionUKF::GetVy() {
  return ukf_.x_(2) * sin(ukf_.x_(3));
}
