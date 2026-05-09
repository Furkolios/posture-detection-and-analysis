#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include "posture_types.h"

/* Classify a fused relative-pitch angle into a PostureState.
 *
 * Phase 1 returns a random valid state; the angle argument is ignored.
 * Phase 2 will replace the implementation with threshold logic against
 * a calibrated neutral pose. The signature is frozen: callers should
 * never need to change. */
PostureState classifier_classify(float angle_deg);

#endif /* CLASSIFIER_H */
