---
layout: page
title: 3DoF Angle Estimation in MOKUKU
---

Error State Kalman Filter (ESKF) for three-degree-of-freedom orientation estimation using IMU (gyroscope + accelerometer) and scalar forward velocity.

**Go to [MOKUKU-INS/3dof](https://github.com/MOKUKU-TECH/MOKUKU-INS/tree/main/3dof) for more details.**


## Algorithm Flow (Overview)

1. **Load data** — IMU CSV and trajectory odom.txt; clip IMU to trajectory time range.
2. **Bias estimation** — Use first N seconds from segment start to estimate accelerometer and gyroscope biases from gravity and near-zero angular velocity.
3. **Filter loop** (per IMU frame):
   - **Predict**: Integrate gyroscope to update R_wc; R_cm nearly constant; add process noise to P.
   - **Update** (when `|dv/dt|` below threshold): Compare accelerometer with expected specific force from kinematics (`a_body = [0, dv/dt, 0] + ω×v`, then subtract gravity); correct R_wc and R_cm via Kalman gain.
4. **Output** — R_wc (vehicle-to-world), R_cm (device-to-vehicle); extract pitch for inclination.

**Observation model**: Expected specific force in IMU frame is `h = R_cmᵀ · (a_body - R_wcᵀ·g)`, where `a_body` uses velocity derivative and Coriolis term. Residual `y = acc_imu - h` drives the correction.

---

## Observability and Pitch Focus

**Due to insufficient observation constraints**, the filter is designed to focus on **pitch angle estimation**:

- **Pitch**: Observable from the accelerometer (gravity projection and longitudinal acceleration). The filter corrects pitch using the accelerometer residual.
- **Yaw**: Not observable from accelerometer alone; no magnetometer or other heading reference. Yaw drifts with gyroscope integration.
- **Roll**: Assumed small; weakly observable in full 3D motion.

**Overall angle estimation** (especially yaw and total 3D rotation) is therefore **relatively dependent on gyroscope integration**. The accelerometer mainly constrains pitch and roll to a lesser extent. Evaluation metrics (e.g. mean pitch error) focus on pitch, which is the best-constrained degree of freedom.

---

## Limitations

- **Yaw unobservable**: No magnetometer or heading reference; yaw drifts with gyroscope integration. Total 3D rotation error can be large even when pitch is accurate.
- **2D motion assumption**: Vehicle moves primarily along the forward (Y) axis; lateral velocity is neglected. Significant lateral motion degrades the observation model.
- **External velocity required**: Filter needs scalar forward velocity (from odometry, GNSS, or wheel encoder). No velocity estimation from IMU alone.
- **Axis/sign convention sensitive**: Accelerometer sign depends on sensor mounting (`inverse_imu_acc`). Misalignment causes systematic pitch bias.
- **Bias estimation from static segment**: Bias is estimated from the first N seconds of the processing segment. Thermal drift or motion during this period affects accuracy.
- **dv/dt noise**: Velocity derivative amplifies noise; large dv/dt triggers update skip. Very noisy velocity input degrades pitch estimation.

---


![Pitch estimation](/assets/img/post_image/pitch_plot.png)

---

