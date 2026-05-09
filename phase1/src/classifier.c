#include <stdlib.h>
#include "classifier.h"

/* Phase 1 stub. Returns a uniformly random valid PostureState and
 * ignores its argument. Phase 2 will replace this with calibration-
 * relative threshold logic; the function signature is frozen. */
PostureState classifier_classify(float angle_deg) {
    (void)angle_deg;  /* deliberately ignored in Phase 1 */
    int r = rand() % POSTURE_STATE_COUNT;
    return (PostureState)r;
}
