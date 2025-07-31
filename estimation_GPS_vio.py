import numpy as np
from scipy.spatial.transform import Rotation as R

class GPSIMU_VIO_EKF:
    def __init__(self, dt=0.01):
        self.dt = dt
        self.x = np.zeros(16)   # [pos(3), vel(3), quat(4), acc_bias(3), gyro_bias(3)]
        self.x[6] = 1.0         # quaternion init
        self.P = np.eye(16) * 0.1
        self.Q = np.eye(16) * 0.01
        self.R_gps = np.eye(6) * 2.0
        self.R_vio = np.diag([0.1, 0.1, 0.1,   # Δpos
                              0.01, 0.01, 0.01])  # Δatt

        self.g = np.array([0, 0, -9.81])

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

        # Update vel & pos
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
        H[0:3, 0:3] = np.eye(3)  # pos
        H[3:6, 3:6] = np.eye(3)  # vel

        z = np.concatenate([gps_pos, gps_vel])
        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R_gps
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(16) - K @ H) @ self.P
        self.x[6:10] = self.normalize_quat(self.x[6:10])

    def update_vio(self, delta_pos, delta_att):
        # Measurement residual
        pred_delta_pos = self.x[3:6] * self.dt
        pred_delta_att = np.zeros(3)  # approx (gyro bias drives error)
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
# Example Usage
# -------------------------
ekf = GPSIMU_VIO_EKF(dt=0.01)

for t in range(1000):
    accel_meas = np.array([0.0, 0.0, 0.0])
    gyro_meas = np.array([0.0, 0.0, 0.01])
    ekf.predict(accel_meas, gyro_meas)

    if t % 200 == 0:  # GPS update
        gps_pos = ekf.x[0:3] + np.random.randn(3) * 2
        gps_vel = ekf.x[3:6] + np.random.randn(3) * 0.5
        ekf.update_gps(gps_pos, gps_vel)

    if t % 50 == 0:  # VIO update more frequent
        delta_pos = np.array([0.1, 0.0, 0.0]) + np.random.randn(3) * 0.01
        delta_att = np.array([0.0, 0.0, 0.001]) + np.random.randn(3) * 0.0005
        ekf.update_vio(delta_pos, delta_att)

    if t % 200 == 0:
        quat = ekf.x[6:10]
        rpy = R.from_quat(quat).as_euler('xyz', degrees=True)
        print(f"t={t*ekf.dt:.1f}s | Pos={ekf.x[0:3]} | RPY={rpy}")
