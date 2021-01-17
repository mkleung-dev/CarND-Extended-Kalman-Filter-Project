#ifndef FusionUKF_H_
#define FusionUKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "unscented_kalman_filter.h"
#include "measurement_package.h"
#include "FusionEKF.h"
#include "tools.h"

class FusionUKF : public FusionEKF {
 public:
  /**
   * Constructor.
   */
  FusionUKF();

  /**
   * Destructor.
   */
  virtual ~FusionUKF();

  /**
   * Run the whole flow of the Kalman Filter from here.
   */
  virtual void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
   * Return the estimated position x.
   */
  virtual float GetPx();

  /**
   * Return the estimated position y.
   */
  virtual float GetPy();

  /**
   * Return the estimated velocity x.
   */
  virtual float GetVx();
  
  /**
   * Return the estimated velocity y.
   */
  virtual float GetVy();

  /**
   * Kalman Filter update and prediction math lives in here.
   */
  UnscentedKalmanFilter ukf_;

 private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd Hj_;
};

#endif // FusionUKF_H_
