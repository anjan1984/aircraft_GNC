import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

class GPSIMU_VIO_EKF:
    def __init__(self, dt=0.01):
        self.dt = dt
        # State: [pos(3), vel(3), quat(4), acc_bias(3), gyro_bias(3)]
        self.x = np.zeros(16)
        self.x[6] = 1.0  # quaternion init
        self.P = np.eye(16) * 0.1
        self.Q = np.eye(16) * 0.01
        self.R_gps = np.eye(6) * 2.0
        self.R_vio = np.diag([0.1, 0.1, 0.1,   # Δpos
                              0.01, 0.01, 0.01])  # Δatt
        self.g = np.array([0, 0, -9.81])  # gravity

    def normalize_quat(self, q):
        return q / np.linalg.norm(q)

    def predict(self, accel_meas, gyro_meas):
        dt = self.dt
        pos, vel = self.x[0:3], self.x[3:6]
        quat, b_a, b_g = self.x[6:10], self.x[10:13], self.x[13:16]

        # Correct IMU for biases
        accel = accel_meas - b_a
        omega = gyro_meas - b_g

        # Attitude update
        dq = R.from_rotvec(omega * dt).as_quat()
        quat_new = R.from_quat(quat) * R.from_quat(dq)
        quat_new = self.normalize_quat(quat_new.as_quat())
        R_nav = R.from_quat(quat_new).as_matrix()

        # Update velocity and position
        accel_nav = R_nav @ accel + self.g
        vel_new = vel + accel_nav * dt
        pos_new = pos + vel * dt + 0.5 * accel_nav * dt**2

        # Update state
        self.x[0:3], self.x[3:6], self.x[6:10] = pos_new, vel_new, quat_new

        # Covariance propagation
        F = np.eye(16)
        F[0:3, 3:6] = np.eye(3) * dt
        self.P = F @ self.P @ F.T + self.Q

    def update_gps(self, gps_pos, gps_vel):
        H = np.zeros((6, 16))
        H[0:3, 0:3] = np.eye(3)  # position
        H[3:6, 3:6] = np.eye(3)  # velocity

        z = np.concatenate([gps_pos, gps_vel])
        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R_gps
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(16) - K @ H) @ self.P
        self.x[6:10] = self.normalize_quat(self.x[6:10])

    def update_vio(self, delta_pos, delta_att):
        # Innovation: measured - predicted relative motion
        pred_delta_pos = self.x[3:6] * self.dt
        pred_delta_att = np.zeros(3)  # approx: drift due to gyro bias
        y = np.concatenate([delta_pos - pred_delta_pos,
                            delta_att - pred_delta_att])

        # Jacobian H
        H = np.zeros((6, 16))
        H[0:3, 3:6] = np.eye(3) * self.dt
        H[3:6, 13:16] = -np.eye(3) * self.dt

        # EKF update
        S = H @ self.P @ H.T + self.R_vio
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(16) - K @ H) @ self.P
        self.x[6:10] = self.normalize_quat(self.x[6:10])

# -------------------------
# Simulation
# -------------------------
ekf = GPSIMU_VIO_EKF(dt=0.01)
timesteps = 2000
positions = []
gps_positions = []

for t in range(timesteps):
    # IMU prediction
    accel_meas = np.array([0.01, 0.0, 0.0])  # small forward accel
    gyro_meas = np.array([0.0, 0.0, 0.001])  # small yaw rate
    ekf.predict(accel_meas, gyro_meas)

    # GPS update every 200 steps
    if t % 200 == 0:
        gps_pos = ekf.x[0:3] + np.random.randn(3) * 2
        gps_vel = ekf.x[3:6] + np.random.randn(3) * 0.5
        ekf.update_gps(gps_pos, gps_vel)
        gps_positions.append(gps_pos)
    else:
        gps_positions.append(np.array([np.nan, np.nan, np.nan]))

    # VIO update every 50 steps
    if t % 50 == 0:
        delta_pos = np.array([0.05, 0.0, 0.0]) + np.random.randn(3) * 0.01
        delta_att = np.array([0.0, 0.0, 0.0005]) + np.random.randn(3) * 0.0002
        ekf.update_vio(delta_pos, delta_att)

    positions.append(ekf.x[0:3])

positions = np.array(positions)
gps_positions = np.array(gps_positions)

# -------------------------
# Plot Results
# -------------------------
plt.figure(figsize=(10,6))
plt.plot(positions[:,0], positions[:,1], label="EKF Trajectory (GPS+VIO)")
plt.scatter(gps_positions[:,0], gps_positions[:,1], c='red', marker='x', label="GPS Measurements")
plt.xlabel("X Position [m]")
plt.ylabel("Y Position [m]")
plt.title("Aircraft Trajectory with EKF Fusing GPS and VIO")
plt.legend()
plt.grid(True)
plt.show()
