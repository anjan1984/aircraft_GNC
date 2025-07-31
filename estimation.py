import numpy as np
from scipy.spatial.transform import Rotation as R

class GPSIMUWindEKF:
    def __init__(self, dt=0.01):
        self.dt = dt

        # State vector length: 18
        self.x = np.zeros(18)
        self.x[6] = 1.0  # quaternion = [1,0,0,0]

        self.P = np.eye(18) * 0.1

        self.Q = np.diag(
            [0.01]*3 +   # pos
            [0.05]*3 +   # vel
            [0.001]*4 +  # quat
            [0.0001]*3 + # accel bias
            [0.0001]*3 + # gyro bias
            [0.001,0.001]  # wind
        )

        self.R_gps = np.diag([2.0, 2.0, 2.0, 0.5, 0.5, 0.5])  # GPS pos+vel
        self.R_airspeed = np.array([[0.5]])  # scalar airspeed noise

        self.g = np.array([0, 0, -9.81])  # gravity vector

    def normalize_quat(self, q):
        return q / np.linalg.norm(q)

    def predict(self, accel_meas, gyro_meas):
        dt = self.dt
        pos = self.x[0:3]
        vel = self.x[3:6]
        quat = self.x[6:10]
        b_acc = self.x[10:13]
        b_gyro = self.x[13:16]

        # Bias-corrected IMU
        accel = accel_meas - b_acc
        omega = gyro_meas - b_gyro

        # Update attitude
        dq = R.from_rotvec(omega * dt).as_quat()
        quat_new = R.from_quat(quat) * R.from_quat(dq)
        quat_new = self.normalize_quat(quat_new.as_quat())

        rot_matrix = R.from_quat(quat_new).as_matrix()

        # Navigation-frame acceleration
        accel_nav = rot_matrix @ accel + self.g

        # Integrate velocity and position
        vel_new = vel + accel_nav * dt
        pos_new = pos + vel * dt + 0.5 * accel_nav * dt**2

        # Update state
        self.x[0:3] = pos_new
        self.x[3:6] = vel_new
        self.x[6:10] = quat_new
        # biases and wind persist

        # Propagate covariance
        F = np.eye(18)
        F[0:3, 3:6] = np.eye(3) * dt
        self.P = F @ self.P @ F.T + self.Q

    def update_gps(self, gps_pos, gps_vel):
        """GPS provides position and velocity"""
        H = np.zeros((6, 18))
        H[0:3, 0:3] = np.eye(3)   # pos
        H[3:6, 3:6] = np.eye(3)   # vel

        z = np.concatenate([gps_pos, gps_vel])
        z_pred = H @ self.x
        y = z - z_pred

        S = H @ self.P @ H.T + self.R_gps
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(18) - K @ H) @ self.P

    def update_airspeed(self, airspeed_meas):
        """Airspeed scalar measurement: |V_air|"""
        vel = self.x[3:6]
        wind = self.x[16:18]
        v_air = vel[0:2] - wind  # horizontal only
        v_air_norm = np.linalg.norm(v_air)

        # Measurement Jacobian
        H = np.zeros((1, 18))
        if v_air_norm > 1e-3:
            H[0, 3] = (v_air[0]) / v_air_norm
            H[0, 4] = (v_air[1]) / v_air_norm
            H[0, 16] = -(v_air[0]) / v_air_norm
            H[0, 17] = -(v_air[1]) / v_air_norm

        z_pred = np.array([v_air_norm])
        y = np.array([airspeed_meas]) - z_pred

        S = H @ self.P @ H.T + self.R_airspeed
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(18) - K @ H) @ self.P

# -----------------------
# Example Usage
# -----------------------
ekf = GPSIMUWindEKF(dt=0.01)

for t in range(1000):
    accel_meas = np.array([0.1, 0.0, 0.0])
    gyro_meas = np.array([0.0, 0.0, 0.01])

    ekf.predict(accel_meas, gyro_meas)

    if t % 100 == 0:  # GPS update 1Hz
        gps_pos = ekf.x[0:3] + np.random.randn(3) * 2.0
        gps_vel = ekf.x[3:6] + np.random.randn(3) * 0.5
        ekf.update_gps(gps_pos, gps_vel)

        airspeed_meas = np.linalg.norm(ekf.x[3:5] - ekf.x[16:18]) + np.random.randn() * 0.5
        ekf.update_airspeed(airspeed_meas)

        quat = ekf.x[6:10]
        rpy = R.from_quat(quat).as_euler('xyz', degrees=True)
        print(f"t={t*ekf.dt:.1f}s | Pos={ekf.x[0:3]} | Wind={ekf.x[16:18]} | RPY={rpy}")
