# GPS–IMU–Wind Extended Kalman Filter (EKF) for Aircraft State Estimation

## 📌 State Vector

We estimate the following 18 states:

\[
x =
\begin{bmatrix}
p_x & p_y & p_z &   % Position
v_x & v_y & v_z &   % Velocity
q_0 & q_1 & q_2 & q_3 &   % Quaternion (attitude)
b_{ax} & b_{ay} & b_{az} &   % Accelerometer bias
b_{gx} & b_{gy} & b_{gz} &   % Gyroscope bias
w_x & w_y  % Wind (horizontal)
\end{bmatrix}^T
\]

- \(p\): Position in NED or ENU  
- \(v\): Velocity in navigation frame  
- \(q\): Quaternion \([q_0, q_1, q_2, q_3]\), normalized  
- \(b_a\): Accelerometer bias  
- \(b_g\): Gyroscope bias  
- \(w\): Wind velocity in horizontal plane  

---

## 📌 System Dynamics (Prediction Step)

### Position
\[
\dot{p} = v
\]

### Velocity
\[
\dot{v} = R(q)(a_{meas} - b_a - n_a) + g
\]

- \(a_{meas}\): measured accelerometer vector  
- \(n_a\): accelerometer noise  
- \(R(q)\): rotation matrix from body → navigation  
- \(g\): gravity vector \([0, 0, -9.81]^T\)

### Attitude (Quaternion Kinematics)
\[
\dot{q} = \tfrac{1}{2} \Omega(\omega_{meas} - b_g - n_g) q
\]

where

\[
\Omega(\omega) =
\begin{bmatrix}
0 & -\omega_x & -\omega_y & -\omega_z \\
\omega_x & 0 & \omega_z & -\omega_y \\
\omega_y & -\omega_z & 0 & \omega_x \\
\omega_z & \omega_y & -\omega_x & 0
\end{bmatrix}
\]

- \(\omega_{meas}\): measured angular rate from gyros  
- \(n_g\): gyro noise  

### Bias Dynamics
\[
\dot{b}_a = n_{ba}, \quad \dot{b}_g = n_{bg}
\]

(modeled as random walk processes)

### Wind Dynamics
\[
\dot{w} = n_w
\]

(modeled as slow random walk)

---

## 📌 Measurement Models (Update Step)

### GPS Measurements
Position and velocity directly observed:

\[
z_{gps} =
\begin{bmatrix}
p_x \\ p_y \\ p_z \\
v_x \\ v_y \\ v_z
\end{bmatrix}
+ \nu_{gps}
\]

with measurement noise \(\nu_{gps}\).

### Airspeed Measurement
Airspeed sensor provides magnitude of **air-relative velocity**:

\[
z_{as} = \| v_{air} \| + \nu_{as}
\]

where

\[
v_{air} =
\begin{bmatrix}
v_x - w_x \\
v_y - w_y
\end{bmatrix}
\]

and \(\nu_{as}\) is sensor noise.

---

## 📌 EKF Equations

### Prediction
\[
x_{k|k-1} = f(x_{k-1}, u_k)
\]
\[
P_{k|k-1} = F P_{k-1|k-1} F^T + Q
\]

- \(f(\cdot)\): nonlinear state propagation using IMU  
- \(F\): Jacobian of dynamics w.r.t. state  
- \(Q\): process noise covariance  

### Update
\[
y = z - h(x_{k|k-1})
\]
\[
S = H P_{k|k-1} H^T + R
\]
\[
K = P_{k|k-1} H^T S^{-1}
\]
\[
x_{k|k} = x_{k|k-1} + K y
\]
\[
P_{k|k} = (I - K H) P_{k|k-1}
\]

- \(h(\cdot)\): measurement model (GPS or airspeed)  
- \(H\): Jacobian of measurement model  
- \(R\): measurement noise covariance  

---

## 📌 Filter Flow Diagram (Conceptual)

```
          +-------------+
          |   IMU (acc, gyro)   |
          +-------------+
                 |
                 v
         [Prediction Step]
                 |
                 v
     x_{k|k-1}, P_{k|k-1}
                 |
         -----------------
         |               |
         v               v
   GPS Measurements   Airspeed Sensor
         |               |
         -----------------
                 |
                 v
          [Update Step]
                 |
                 v
        x_{k|k}, P_{k|k}
```

---

## 📌 Applications in eVTOL

- Ensures drift-free navigation in urban AAM environments  
- Allows for **wind-aware trajectory planning** and safety margins  
- Integrates seamlessly with flight control systems for **precision hover and path tracking**  
