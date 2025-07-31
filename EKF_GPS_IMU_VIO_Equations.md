
# EKF with GPS, IMU, and VIO — Equations

## 📌 State Vector

\[
x =
\begin{bmatrix}
p_x & p_y & p_z & 
v_x & v_y & v_z & 
q_0 & q_1 & q_2 & q_3 & 
b_{ax} & b_{ay} & b_{az} &
b_{gx} & b_{gy} & b_{gz}
\end{bmatrix}^T
\]

- p = position in navigation frame  
- v = velocity in navigation frame  
- q = quaternion (attitude, body → navigation)  
- b_a, b_g = accelerometer and gyro biases  

---

## 📌 Process Model (Prediction via IMU)

### Position Dynamics
\[
\dot{p} = v
\]

\[
p_{k+1} = p_k + v_k \Delta t + \tfrac{1}{2}(R(q_k)(a_m - b_a - n_a) + g) \Delta t^2
\]

### Velocity Dynamics
\[
\dot{v} = R(q)(a_m - b_a - n_a) + g
\]

\[
v_{k+1} = v_k + (R(q_k)(a_m - b_a - n_a) + g) \Delta t
\]

### Attitude Dynamics
\[
\dot{q} = \tfrac{1}{2} \Omega(\omega_m - b_g - n_g) q
\]

\[
q_{k+1} = \text{normalize}\big( q_k \otimes \exp(\tfrac{1}{2} \Delta t \, \omega) \big)
\]

### Bias Dynamics
\[
\dot{b}_a = n_{ba}, \quad \dot{b}_g = n_{bg}
\]

---

## 📌 Covariance Propagation

\[
P_{k+1} = F P_k F^T + Q
\]

---

## 📌 Measurement Models

### GPS Update
\[
z_{gps} =
\begin{bmatrix}
p \\ v
\end{bmatrix}
+ \nu_{gps}
\]

Residual:
\[
y = z_{gps} - H_{gps} x
\]

Jacobian:
\[
H_{gps} =
\begin{bmatrix}
I_{3x3} & 0 & 0 & 0 & 0 \\
0 & I_{3x3} & 0 & 0 & 0
\end{bmatrix}
\]

### VIO Update
\[
z_{vio} =
\begin{bmatrix}
\Delta p \\ \Delta \theta
\end{bmatrix}
+ \nu_{vio}
\]

Residual:
\[
y = z_{vio} - h(x)
\]

Prediction:
\[
h(x) =
\begin{bmatrix}
v \Delta t \\ 0
\end{bmatrix}
\]

Jacobian:
\[
H_{vio} =
\begin{bmatrix}
0 & I_{3x3}\Delta t & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & -I_{3x3}\Delta t
\end{bmatrix}
\]

---

## 📌 EKF Update Step

\[
S = H P H^T + R
\]
\[
K = P H^T S^{-1}
\]
\[
x_{k+1} = x_k + K y
\]
\[
P_{k+1} = (I - K H) P
\]

Normalize quaternion:
\[
q_{k+1} \leftarrow \frac{q_{k+1}}{\|q_{k+1}\|}
\]

---

## 📊 Sensor Fusion Flow

```
        +-----------------+
        |    IMU (acc,ω)  |
        +-----------------+
                 |
           [Prediction]
                 |
                 v
         +----------------+
         |      EKF       |
         +----------------+
            ^          ^
            |          |
   +--------+          +--------+
   |                         |
+---------+           +---------------+
|   GPS   |           | VIO / SLAM    |
| (pos,vel)           | (Δpos, Δatt) |
+---------+           +---------------+
```
