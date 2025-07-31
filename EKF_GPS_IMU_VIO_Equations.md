
# EKF with GPS, IMU, and VIO — Equations

## 📌 State Vector



$$\left( 
x =
\begin{bmatrix}
p_x & p_y & p_z & 
v_x & v_y & v_z & 
q_0 & q_1 & q_2 & q_3 & 
b_{ax} & b_{ay} & b_{az} &
b_{gx} & b_{gy} & b_{gz}
\end{bmatrix}^T
\right)$$

- p = position in navigation frame  
- v = velocity in navigation frame  
- q = quaternion (attitude, body → navigation)  
- b_a, b_g = accelerometer and gyro biases  

---

## 📌 Process Model (Prediction via IMU)

### Position Dynamics
$$\left( 
\dot{p} = v
\right)$$

$$\left( 
p_{k+1} = p_k + v_k \Delta t + \tfrac{1}{2}(R(q_k)(a_m - b_a - n_a) + g) \Delta t^2
\right)$$

### Velocity Dynamics
$$\left( 
\dot{v} = R(q)(a_m - b_a - n_a) + g
\right)$$

$$\left( 
v_{k+1} = v_k + (R(q_k)(a_m - b_a - n_a) + g) \Delta t
\right)$$

### Attitude Dynamics
$$\left( 
\dot{q} = \tfrac{1}{2} \Omega(\omega_m - b_g - n_g) q
\right)$$

$$\left( 
q_{k+1} = \text{normalize}\big( q_k \otimes \exp(\tfrac{1}{2} \Delta t \, \omega) \big)
\right)$$

### Bias Dynamics
$$\left( 
\dot{b}_a = n_{ba}, \quad \dot{b}_g = n_{bg}
\right)$$

---

## 📌 Covariance Propagation

$$\left( 
P_{k+1} = F P_k F^T + Q
\right)$$

---

## 📌 Measurement Models

### GPS Update

GPS update: 



$$\left(
z_{gps} =
\begin{bmatrix}
p \\ v
\end{bmatrix}
+ \nu_{gps}
\right)$$


Residual:

$$\left(
y = z_{gps} - H_{gps} x
\right)$$


Jacobian:

$$\left(
H_{gps} =
\begin{bmatrix}
I_{3x3} & 0 & 0 & 0 & 0 \\
0 & I_{3x3} & 0 & 0 & 0
\end{bmatrix}
\right)$$


### VIO Update

update

$$\left(
z_{vio} =
\begin{bmatrix}
\Delta p \\ \Delta \theta
\end{bmatrix}
+ \nu_{vio}
\right)$$

Residual:

$$\left( 
y = z_{vio} - h(x)
\right)$$

Prediction:
$$\left( 
h(x) =
\begin{bmatrix}
v \Delta t \\ 0
\end{bmatrix}
\right)$$

Jacobian:
$$\left( 
H_{vio} =
\begin{bmatrix}
0 & I_{3x3}\Delta t & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & -I_{3x3}\Delta t
\end{bmatrix}
\right)$$

---

## 📌 EKF Update Step

$$\left( 
S = H P H^T + R
\right)$$
$$\left( 
K = P H^T S^{-1}
\right)$$
$$\left( 
x_{k+1} = x_k + K y
\right)$$
$$\left( 
P_{k+1} = (I - K H) P
\right)$$

Normalize quaternion:
$$\left( 
q_{k+1} \leftarrow \frac{q_{k+1}}{\|q_{k+1}\|}
\right)$$

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
