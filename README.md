# aircraft_GNC

 ## Problem Context
Aircraft state estimation means determining position, velocity, attitude (orientation), and sometimes higher-order states (biases, winds, etc.) in real time.
Since both GPS and IMU have limitations, they are fused together for robustness:

GPS: Provides absolute position & velocity (low-rate, subject to outages, multipath, and noise).

IMU: Provides high-rate acceleration & angular velocity (drifts over time due to sensor bias and noise).

By fusing them, we leverage the complementary strengths.

🔑 States Typically Estimated
𝑥
=
[
Position (ECEF/ENU)
Velocity
Attitude (quaternion/Euler)
IMU biases (gyro + accel)
]
x= 
​
  
Position (ECEF/ENU)
Velocity
Attitude (quaternion/Euler)
IMU biases (gyro + accel)
​
  
​
 
Depending on fidelity, may also include wind, scale factors, magnetometer biases, etc.

## ⚙️ Core Approach
The standard solution is an Extended Kalman Filter (EKF) (sometimes Unscented KF or Particle Filter for nonlinearities).

1. System Model (Prediction from IMU)
Using IMU readings:

Angular rate → integrate attitude

Specific force → rotate into navigation frame, subtract gravity, integrate to velocity

Integrate velocity → position

𝑝
˙
=
𝑣
,
𝑣
˙
=
𝑅
(
𝑞
)
(
𝑎
𝑚
𝑒
𝑎
𝑠
−
𝑏
𝑎
−
𝑛
𝑎
)
+
𝑔
,
𝑞
˙
=
1
2
Ω
(
𝜔
𝑚
𝑒
𝑎
𝑠
−
𝑏
𝑔
−
𝑛
𝑔
)
𝑞
p
˙
​
 =v, 
v
˙
 =R(q)(a 
meas
​
 −b 
a
​
 −n 
a
​
 )+g, 
q
˙
​
 = 
2
1
​
 Ω(ω 
meas
​
 −b 
g
​
 −n 
g
​
 )q
where:

𝑞
q = quaternion

𝑅
(
𝑞
)
R(q) = rotation matrix from body → navigation frame

𝑏
𝑎
,
𝑏
𝑔
b 
a
​
 ,b 
g
​
  = accelerometer and gyro biases

𝑛
𝑎
,
𝑛
𝑔
n 
a
​
 ,n 
g
​
  = sensor noise

2. Measurement Model (Correction from GPS)
GPS provides:

Position (lat, lon, alt → ENU/ECEF)

Velocity (from Doppler shift)

Update step compares GPS measurements with predicted states.

𝑧
=
𝐻
𝑥
+
𝜈
z=Hx+ν
where 
𝑧
z is GPS observation, 
𝜈
ν measurement noise.

📊 Typical Filter Structure
Prediction (Propagate IMU data)

High-rate (100–200 Hz)

Predict state and covariance forward

Update (Correct with GPS)

Lower rate (1–10 Hz)

Correct drift using GPS position/velocity

Bias Estimation

Constant or slowly varying IMU bias states are estimated in the EKF

Prevents long-term drift when GPS is available

🚀 Decision Flow (High-Level)
rust
Copy
Edit
IMU (100 Hz) ---> EKF Prediction ---> Estimated State
     GPS (5 Hz) ---> EKF Update -----^

Assume we have airspeed measurements (from pitot tube or equivalent sensor), which help separate true airspeed from ground-relative velocity.

State Vector
𝑥
=
[
𝑝
𝑥
,
𝑝
𝑦
,
𝑝
𝑧
,
  
𝑣
𝑥
,
𝑣
𝑦
,
𝑣
𝑧
,
  
𝑞
0
,
𝑞
1
,
𝑞
2
,
𝑞
3
,
  
𝑏
𝑎
𝑥
,
𝑏
𝑎
𝑦
,
𝑏
𝑎
𝑧
,
  
𝑏
𝑔
𝑥
,
𝑏
𝑔
𝑦
,
𝑏
𝑔
𝑧
,
  
𝑤
𝑥
,
𝑤
𝑦
]
𝑇
x=[p 
x
​
 ,p 
y
​
 ,p 
z
​
 ,v 
x
​
 ,v 
y
​
 ,v 
z
​
 ,q 
0
​
 ,q 
1
​
 ,q 
2
​
 ,q 
3
​
 ,b 
a
x
​
 ,b 
a
y
​
 ,b 
a
z
​
 ,b 
g
x
​
 ,b 
g
y
​
 ,b 
g
z
​
 ,w 
x
​
 ,w 
y
​
 ] 
T
 
Position (3)

Velocity in NED (3)

Attitude quaternion (4)

Accelerometer bias (3)

Gyro bias (3)

Wind (horizontal components: 
𝑤
𝑥
,
𝑤
𝑦
w 
x
​
 ,w 
y
​
 )

