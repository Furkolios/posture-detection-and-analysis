# posture-detection-and-analysis
We propose a wearable system that monitors the user’s spinal posture in real time, alerts the user
when sustained poor posture is detected, and streams posture data over a wireless link to a host
computer for visualization. Two inertial sensors mounted on the body — one at the hip and one
in the upper thoracic region — measure orientation continuously. The relative orientation between
the two sensors corresponds directly to the curvature of the spine, which is the quantity of interest
for posture analysis.
The MSP432 acts as the central processing unit: it reads both sensors, performs sensor fusion to
obtain stable orientation estimates, classifies the user’s current posture against a calibrated neutral
reference, triggers an audible warning when poor posture persists, and transmits telemetry to the
host PC. The PC application receives the telemetry, renders the user’s pose as a stick figure, and
produces a session-level posture profile.
