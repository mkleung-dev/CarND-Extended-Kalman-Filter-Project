#include "FusionKF.h"

/**
 * Constructor.
 */
FusionKF::FusionKF(bool bUseLaser, bool bUseRadar) {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  bUseLaser_ = bUseLaser;
  bUseRadar_ = bUseRadar;
}
