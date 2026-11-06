# ArmPi encoder observations — 6 November 2026

Robot supported with wheels free; user manually turned wheels individually.
Pi used the candidate combined I2C register select/read on bus 1, address
`0x34`, register `0x3c`, decoding four signed little-endian 32-bit counts.
The diagnostic sent no motor commands and no count reset. The driver remained
running; one diagnostic process was stopped after the test. This is behavioral
evidence for the read protocol on this board, not a verified board revision.

| Wheel (robot forward as reference) | Channel | Count direction when top moves forward |
| --- | ---: | --- |
| Front-left | 3 | decreases |
| Front-right | 1 | increases |
| Rear-left | 4 | increases |
| Rear-right | 2 | decreases |

Each wheel turn changed its corresponding channel while the others stayed
fixed. Front-left reverse turn changed the channel in the opposite direction.
At rest initial counts `(0, -1, -449, 723)` stayed stable. After front-left
first forward and approximately reverse turns channel 3 went -449 -> -4423
-> -310; those were approximate unmarked rotations, not scale measurements.

The user marked front-left wheel and chassis and reported aligned full turns.
One forward turn changed channel 3 approximately -6039 -> -11828 (5789 ticks).
The subsequent three forward turns changed -11828 -> -29234 (17406 ticks,
5802 ticks/turn). The first marked turn included possible movement during
marking, so is only approximate. These are hand-aligned observations, not an
exact encoder specification; other wheels have not had full marked-turn
calibration. An approximate quarter forward turn of the front-right wheel gave
channel 1 5434 -> 6877 (1443 ticks); quarter-turn positions were not marked.
Rear-left channel 4 went 5266 -> 6825; rear-right channel 2 went -6315 ->
-7688 during user-described approximate quarter turns.

For the node's order FL, FR, RL, RR, measured channel order is `[3, 1, 4, 2]`
and measured polarity multipliers for forward wheel travel are `[-1, 1, 1, -1]`.
## Provisional bench configuration and live test

The user subsequently repeated marked three-revolution forward turns on the
other wheels. Counts were FR 6877 -> 24516 (17639 / 3 = 5879.7 per turn),
RL 7207 -> 24586 (17379 / 3 = 5793), and RR -7688 -> -25179
(-17491 / 3 = -5830.3). FL was 5802. Mean magnitude is 5826.25 per turn;
the temporary single-scale configuration uses 5826.0. These hand-measured
values are approximate; the RL user was uncertain about the count of turns,
though its implied scale agrees closely with the others.

At the user's direction the first test used the simulation radius 0.0475 m and
wheelbase sum 0.2167 m; the discontinuity limit 30000 ticks/s is provisional,
not a measured maximum. At 20 Hz, stationary wheel odometry was received over
TCP, and a manual lifted FL wheel turn changed the pose estimate; the ROS 2
bridge and wheel TF adapter then produced matching `wheel_odom -> base_link`.
The single-wheel lifted test is not a measured ground trajectory. The SLAM
camera, static TF and real-world kinematic calibration remain to validate.
Controller covariance values are placeholders.

The saved `wheel_odometry_bench.launch` defaults to disabled until explicitly
passed `accept_provisional:=true`; this enables the existing node's
`calibration_verified` gate for experimental use only. Do not treat that flag
as proof of ground calibration or navigation readiness.

## Measured chassis geometry

The user measured front-to-rear wheel-center spacing as 19.5 cm and left-to-right
wheel-center spacing as 22 cm (assuming measurements in centimeters). Their
half-sum is 0.2075 m. In a rotation test with that setting, the user estimated
the robot physically overshot 360° by 20–25°. An experimental effective value of
0.196 m was selected for testing; it is not the measured physical geometry,
and mecanum slip and the approximate visual angle may affect the estimate. On
the test surface, user judged a positive 360° turn accurate by eye (settled
wheel odometry +361.49°) and two negative turns within about 5° or less of the
initial heading (settled odometry -722.18°). Odometry triggered each stop; the
visual assessment is approximate and only one run per direction was performed.

Four short floor translations were also compared with user measurements:
forward odometry 2.56 cm versus approximately 2.5 cm; backward 2.57 cm versus
approximately 2.6 cm; left 2.49 cm versus approximately 2.5 cm; right 2.55 cm
versus approximately 2.5 cm. The user observed some uneven front/rear lateral
motion during the left test. These observations do not characterize drift,
slip or repeatability.
