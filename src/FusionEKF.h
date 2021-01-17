#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"
#include "FusionKF.h"
#include "tools.h"

class FusionEKF : public FusionKF {
 public:
  /**
   * Constructor.
   */
  FusionEKF(bool bUseLaser, bool bUseRadar);

  /**
   * Destructor.
   */
  virtual ~FusionEKF();

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
  KalmanFilter ekf_;

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

#endif // FusionEKF_H_
