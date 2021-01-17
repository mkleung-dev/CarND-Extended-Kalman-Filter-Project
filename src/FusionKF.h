#ifndef FusionKF_H_
#define FusionKF_H_

#include "measurement_package.h"
#include "tools.h"

class FusionKF {
 public:
  /**
   * Constructor.
   */
  FusionKF(bool bUseLaser, bool bUseRadar);

  /**
   * Run the whole flow of the Kalman Filter from here.
   */
  virtual void ProcessMeasurement(const MeasurementPackage &measurement_pack) = 0;

  /**
   * Return the estimated position x.
   */
  virtual float GetPx() = 0;

  /**
   * Return the estimated position y.
   */
  virtual float GetPy() = 0;

  /**
   * Return the estimated velocity x.
   */
  virtual float GetVx() = 0;
  
  /**
   * Return the estimated velocity y.
   */
  virtual float GetVy() = 0;

 protected:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // option to use measurement
  bool bUseLaser_;
  bool bUseRadar_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
};

#endif // FusionKF_H_
