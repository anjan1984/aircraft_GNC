import numpy as np
import matplotlib.pyplot as plt

# ------------------------
# Trajectory Planner
# ------------------------

def compute_continuous_velocities(waypoints, segment_times):
    n = len(waypoints) - 1
    dt = np.array(segment_times)
    pos = np.array([waypoints[i] for i in range(len(waypoints))])
    velocities = np.zeros_like(pos)

    for dim in range(3):  # x, y, z
        A = np.zeros((n+1, n+1))
        d = np.zeros(n+1)

        # Boundary: zero velocity at start and end
        A[0,0] = 1; A[n,n] = 1
        d[0] = 0; d[n] = 0

        for i in range(1, n):
            A[i, i-1] = dt[i-1]
            A[i, i]   = 2*(dt[i-1] + dt[i])
            A[i, i+1] = dt[i]
            d[i] = 3*((pos[i+1,dim] - pos[i,dim]) / dt[i] -
                      (pos[i,dim] - pos[i-1,dim]) / dt[i-1])

        v = np.linalg.solve(A, d)
        velocities[:,dim] = v

    return velocities

def cubic_poly_coeffs(p0, pf, v0, vf, t0, tf):
    A = np.array([
        [1, t0, t0**2, t0**3],
        [0, 1, 2*t0, 3*t0**2],
        [1, tf, tf**2, tf**3],
        [0, 1, 2*tf, 3*tf**2]
    ])
    b = np.array([p0, v0, pf, vf])
    return np.linalg.solve(A, b)

def build_trajectory_with_velocity_continuity(waypoints, segment_times, velocities):
    traj_coeffs = {'x':[], 'y':[], 'z':[]}
    for i, dt in enumerate(segment_times):
        p0, pf = waypoints[i], waypoints[i+1]
        v0, vf = velocities[i], velocities[i+1]
        traj_coeffs['x'].append(cubic_poly_coeffs(p0[0], pf[0], v0[0], vf[0], 0, dt))
        traj_coeffs['y'].append(cubic_poly_coeffs(p0[1], pf[1], v0[1], vf[1], 0, dt))
        traj_coeffs['z'].append(cubic_poly_coeffs(p0[2], pf[2], v0[2], vf[2], 0, dt))
    return traj_coeffs

def eval_poly(coeffs, t):
    a0,a1,a2,a3 = coeffs
    return a0 + a1*t + a2*t**2 + a3*t**3

def eval_poly_derivative(coeffs, t):
    a0,a1,a2,a3 = coeffs
    return a1 + 2*a2*t + 3*a3*t**2

def get_segment_index(segment_times, t):
    time_accum = 0
    for i, dt in enumerate(segment_times):
        if time_accum <= t < time_accum + dt:
            return i, t - time_accum
        time_accum += dt
    return len(segment_times)-1, segment_times[-1]

def get_position(traj_coeffs, segment_times, t):
    i, t_seg = get_segment_index(segment_times, t)
    return np.array([
        eval_poly(traj_coeffs['x'][i], t_seg),
        eval_poly(traj_coeffs['y'][i], t_seg),
        eval_poly(traj_coeffs['z'][i], t_seg)
    ])

def get_velocity(traj_coeffs, segment_times, t):
    i, t_seg = get_segment_index(segment_times, t)
    return np.array([
        eval_poly_derivative(traj_coeffs['x'][i], t_seg),
        eval_poly_derivative(traj_coeffs['y'][i], t_seg),
        eval_poly_derivative(traj_coeffs['z'][i], t_seg)
    ])

# ------------------------
# Visualization
# ------------------------

def plot_smooth_trajectory(traj_coeffs, segment_times, total_time, waypoints):
    t_vals = np.linspace(0, total_time, 500)
    pos_vals = np.array([get_position(traj_coeffs, segment_times, t) for t in t_vals])
    vel_vals = np.array([get_velocity(traj_coeffs, segment_times, t) for t in t_vals])

    fig = plt.figure(figsize=(15,6))

    # 3D path
    ax1 = fig.add_subplot(131, projection='3d')
    ax1.plot(pos_vals[:,0], pos_vals[:,1], pos_vals[:,2], label="Trajectory", color="blue")
    wp = np.array(list(waypoints.values()))
    ax1.scatter(wp[:,0], wp[:,1], wp[:,2], color="red", marker="o", s=60, label="Waypoints")
    ax1.set_title("3D Flight Path")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_zlabel("Z (m)")
    ax1.legend(); ax1.grid(True)

    # Position vs time
    ax2 = fig.add_subplot(132)
    ax2.plot(t_vals, pos_vals[:,0], label="X")
    ax2.plot(t_vals, pos_vals[:,1], label="Y")
    ax2.plot(t_vals, pos_vals[:,2], label="Z")
    ax2.set_title("Position vs Time")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Position (m)")
    ax2.legend(); ax2.grid(True)

    # Velocity vs time
    ax3 = fig.add_subplot(133)
    ax3.plot(t_vals, vel_vals[:,0], label="Xdot")
    ax3.plot(t_vals, vel_vals[:,1], label="Ydot")
    ax3.plot(t_vals, vel_vals[:,2], label="Zdot")
    ax3.set_title("Velocity vs Time")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Velocity (m/s)")
    ax3.legend(); ax3.grid(True)

    plt.tight_layout()
    plt.show()

# ------------------------
# Main
# ------------------------

def main():
    waypoints = {
        0: np.array([0.0,   0.0,  0.0]),   # Takeoff start
        1: np.array([0.0,   0.0, 30.0]),   # Climb to 50m
        2: np.array([100.0, 0.0, 50.0]),   # Transition
        3: np.array([1000.0,0.0, 50.0]),   # Cruise
        4: np.array([1050.0,0.0, 30.0]),   # Cruise
        5: np.array([1100.0,0.0,  0.0])    # Landing
    }
    segment_times = [10, 20, 80, 10, 10]

    velocities = compute_continuous_velocities(waypoints, segment_times)
    traj_coeffs = build_trajectory_with_velocity_continuity(waypoints, segment_times, velocities)
    total_time = sum(segment_times)

    plot_smooth_trajectory(traj_coeffs, segment_times, total_time, waypoints)

if __name__ == "__main__":
    main()
