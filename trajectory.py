import numpy as np
import matplotlib.pyplot as plt

def compute_continuous_velocities(waypoints, segment_times):
    """
    Compute velocities at waypoints for velocity continuity
    using natural cubic spline approach per dimension.
    Assume zero velocity at first and last waypoint.
    """
    n = len(waypoints) - 1  # number of segments

    times = np.array(segment_times)
    dt = times

    pos = np.array([waypoints[i] for i in range(len(waypoints))])  # shape (n+1, 3)
    velocities = np.zeros_like(pos)  # initialize velocities

    for dim in range(3):  # x, y, z
        # Setup tridiagonal system Ax = d for velocities
        A = np.zeros((n+1, n+1))
        d = np.zeros(n+1)

        # Boundary conditions: natural spline (vel=0 at ends)
        A[0,0] = 1
        A[n,n] = 1
        d[0] = 0
        d[n] = 0

        # Interior points continuity
        for i in range(1, n):
            A[i, i-1] = dt[i-1]
            A[i, i] = 2 * (dt[i-1] + dt[i])
            A[i, i+1] = dt[i]

            d[i] = 3 * ( (pos[i+1,dim] - pos[i,dim]) / dt[i] - (pos[i,dim] - pos[i-1,dim]) / dt[i-1] )

        # Solve for velocities at waypoints in this dimension
        v = np.linalg.solve(A, d)
        velocities[:,dim] = v

    return velocities

def cubic_poly_coeffs(p0, pf, v0, vf, t0, tf):
    T = tf - t0
    A = np.array([
        [1, t0,   t0**2,    t0**3],
        [0, 1,  2*t0,     3*t0**2],
        [1, tf,   tf**2,    tf**3],
        [0, 1,  2*tf,     3*tf**2]
    ])
    b = np.array([p0, v0, pf, vf])
    coeffs = np.linalg.solve(A, b)
    return coeffs

def build_trajectory_with_velocity_continuity(waypoints, segment_times, velocities):
    n = len(segment_times)
    traj_coeffs = {'x':[], 'y':[], 'z':[]}
    t0 = 0

    for i in range(n):
        tf = t0 + segment_times[i]
        p0 = waypoints[i]
        pf = waypoints[i+1]
        v0 = velocities[i]
        vf = velocities[i+1]

        coeffs_x = cubic_poly_coeffs(p0[0], pf[0], v0[0], vf[0], t0, tf)
        coeffs_y = cubic_poly_coeffs(p0[1], pf[1], v0[1], vf[1], t0, tf)
        coeffs_z = cubic_poly_coeffs(p0[2], pf[2], v0[2], vf[2], t0, tf)

        traj_coeffs['x'].append(coeffs_x)
        traj_coeffs['y'].append(coeffs_y)
        traj_coeffs['z'].append(coeffs_z)
        t0 = tf

    return traj_coeffs

def eval_poly(coeffs, t):
    a0, a1, a2, a3 = coeffs
    return a0 + a1*t + a2*t**2 + a3*t**3

def eval_poly_derivative(coeffs, t):
    a0, a1, a2, a3 = coeffs
    return a1 + 2*a2*t + 3*a3*t**2

def eval_poly_second_derivative(coeffs, t):
    a0, a1, a2, a3 = coeffs
    return 2*a2 + 6*a3*t

def get_segment_index(segment_times, t):
    time_accum = 0
    for i, dt in enumerate(segment_times):
        if time_accum <= t <= time_accum + dt:
            return i, t - time_accum
        time_accum += dt
    return len(segment_times) - 1, segment_times[-1]

def get_position(traj_coeffs, segment_times, t):
    i, t_seg = get_segment_index(segment_times, t)
    x = eval_poly(traj_coeffs['x'][i], t_seg)
    y = eval_poly(traj_coeffs['y'][i], t_seg)
    z = eval_poly(traj_coeffs['z'][i], t_seg)
    return np.array([x, y, z])

def get_velocity(traj_coeffs, segment_times, t):
    i, t_seg = get_segment_index(segment_times, t)
    vx = eval_poly_derivative(traj_coeffs['x'][i], t_seg)
    vy = eval_poly_derivative(traj_coeffs['y'][i], t_seg)
    vz = eval_poly_derivative(traj_coeffs['z'][i], t_seg)
    return np.array([vx, vy, vz])

def get_acceleration(traj_coeffs, segment_times, t):
    i, t_seg = get_segment_index(segment_times, t)
    ax = eval_poly_second_derivative(traj_coeffs['x'][i], t_seg)
    ay = eval_poly_second_derivative(traj_coeffs['y'][i], t_seg)
    az = eval_poly_second_derivative(traj_coeffs['z'][i], t_seg)
    return np.array([ax, ay, az])

def main():
    waypoints = {
        0: np.array([0.0, 0.0, 0.0]),     
        1: np.array([0.0, 0.0, 50.0]),    
        2: np.array([100.0, 0.0, 50.0]),  
        3: np.array([1000.0, 0.0, 50.0]), 
        4: np.array([1050.0, 0.0, 0.0])   
    }
    segment_times = [10, 20, 80, 10]

    velocities = compute_continuous_velocities(waypoints, segment_times)
    print("Velocities at waypoints:\n", velocities)

    traj_coeffs = build_trajectory_with_velocity_continuity(waypoints, segment_times, velocities)

    total_time = sum(segment_times)
    time_samples = np.linspace(0, total_time, 1000)

    positions = np.array([get_position(traj_coeffs, segment_times, t) for t in time_samples])
    velocities_samples = np.array([get_velocity(traj_coeffs, segment_times, t) for t in time_samples])
    accelerations = np.array([get_acceleration(traj_coeffs, segment_times, t) for t in time_samples])

    fig = plt.figure(figsize=(14,6))
    ax1 = fig.add_subplot(131, projection='3d')
    ax1.plot(positions[:,0], positions[:,1], positions[:,2], label='Trajectory', color='b')
    ax1.scatter([wp[0] for wp in waypoints.values()], [wp[1] for wp in waypoints.values()], [wp[2] for wp in waypoints.values()], color='r', label='Waypoints')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D eVTOL Trajectory')
    ax1.legend()
    ax1.grid(True)

    speed = np.linalg.norm(velocities_samples, axis=1)
    ax2 = fig.add_subplot(132)
    ax2.plot(time_samples, speed, label='Speed (m/s)')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Speed (m/s)')
    ax2.set_title('Velocity Profile')
    ax2.grid(True)

    accel_mag = np.linalg.norm(accelerations, axis=1)
    ax3 = fig.add_subplot(133)
    ax3.plot(time_samples, accel_mag, label='Acceleration (m/s²)', color='r')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Acceleration (m/s²)')
    ax3.set_title('Acceleration Profile')
    ax3.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
