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

    for dim in range(3):
        A = np.zeros((n+1, n+1))
        d = np.zeros(n+1)

        # boundary condition: zero velocity at start & end
        A[0,0] = 1; A[n,n] = 1
        d[0] = 0; d[n] = 0

        for i in range(1, n):
            A[i,i-1] = dt[i-1]
            A[i,i]   = 2*(dt[i-1]+dt[i])
            A[i,i+1] = dt[i]
            d[i] = 3*((pos[i+1,dim]-pos[i,dim])/dt[i] -
                      (pos[i,dim]-pos[i-1,dim])/dt[i-1])
        v = np.linalg.solve(A,d)
        velocities[:,dim] = v
    return velocities

def cubic_poly_coeffs(p0, pf, v0, vf, t0, tf):
    A = np.array([
        [1, t0, t0**2, t0**3],
        [0, 1, 2*t0, 3*t0**2],
        [1, tf, tf**2, tf**3],
        [0, 1, 2*tf, 3*tf**2]
    ])
    b = np.array([p0,v0,pf,vf])
    return np.linalg.solve(A,b)

def build_trajectory_with_velocity_continuity(waypoints, segment_times, velocities):
    traj_coeffs={'x':[],'y':[],'z':[]}
    for i,dt in enumerate(segment_times):
        p0,pf = waypoints[i],waypoints[i+1]
        v0,vf = velocities[i],velocities[i+1]
        traj_coeffs['x'].append(cubic_poly_coeffs(p0[0],pf[0],v0[0],vf[0],0,dt))
        traj_coeffs['y'].append(cubic_poly_coeffs(p0[1],pf[1],v0[1],vf[1],0,dt))
        traj_coeffs['z'].append(cubic_poly_coeffs(p0[2],pf[2],v0[2],vf[2],0,dt))
    return traj_coeffs

def eval_poly(coeffs,t): 
    return coeffs[0]+coeffs[1]*t+coeffs[2]*t**2+coeffs[3]*t**3

def eval_poly_derivative(coeffs,t):
    return coeffs[1]+2*coeffs[2]*t+3*coeffs[3]*t**2

def get_segment_index(segment_times,t):
    time_accum=0
    for i,dt in enumerate(segment_times):
        if time_accum<=t<time_accum+dt:
            return i,t-time_accum
        time_accum+=dt
    return len(segment_times)-1,segment_times[-1]

def get_position(traj_coeffs,segment_times,t):
    i,t_seg=get_segment_index(segment_times,t)
    return np.array([
        eval_poly(traj_coeffs['x'][i],t_seg),
        eval_poly(traj_coeffs['y'][i],t_seg),
        eval_poly(traj_coeffs['z'][i],t_seg)
    ])

def get_velocity(traj_coeffs,segment_times,t):
    i,t_seg=get_segment_index(segment_times,t)
    return np.array([
        eval_poly_derivative(traj_coeffs['x'][i],t_seg),
        eval_poly_derivative(traj_coeffs['y'][i],t_seg),
        eval_poly_derivative(traj_coeffs['z'][i],t_seg)
    ])

# ------------------------
# PID Controller
# ------------------------

class PIDController:
    def __init__(self,kp,ki,kd,dt,integral_limit=5.0):
        self.kp,self.ki,self.kd,self.dt=kp,ki,kd,dt
        self.integral=np.zeros(3)
        self.prev_error=np.zeros(3)
        self.integral_limit=integral_limit

    def update(self,error):
        self.integral+=error*self.dt
        self.integral=np.clip(self.integral,-self.integral_limit,self.integral_limit)
        derivative=(error-self.prev_error)/self.dt
        output=self.kp*error+self.ki*self.integral+self.kd*derivative
        self.prev_error=error
        return output

# ------------------------
# Attitude & Motor Allocation
# ------------------------

def accel_to_attitude(acc_cmd,mass=10.0,g=9.81):
    acc_total=acc_cmd+np.array([0,0,g])
    thrust_mag=mass*np.linalg.norm(acc_total)
    T_norm=acc_total/np.linalg.norm(acc_total)
    theta=np.arctan2(T_norm[0],T_norm[2])  # pitch
    phi=np.arcsin(-T_norm[1])              # roll
    return thrust_mag,phi,theta,0.0

def motor_mixing(thrust,phi,theta,psi,arm=0.25,k_torque=0.02):
    """
    Simplified quad allocation: 4 rotors
    """
    # Desired forces/torques
    Fx=thrust*np.sin(theta)
    Fy=-thrust*np.sin(phi)
    Fz=thrust*np.cos(theta)*np.cos(phi)

    tau_roll=phi*10.0   # proportional torque demand
    tau_pitch=theta*10.0
    tau_yaw=psi*5.0

    # Mix to 4 motors (very simplified)
    m1=(Fz/4)+(tau_roll/(2*arm))+(tau_pitch/(2*arm))+(tau_yaw/(4*k_torque))
    m2=(Fz/4)-(tau_roll/(2*arm))+(tau_pitch/(2*arm))-(tau_yaw/(4*k_torque))
    m3=(Fz/4)-(tau_roll/(2*arm))-(tau_pitch/(2*arm))+(tau_yaw/(4*k_torque))
    m4=(Fz/4)+(tau_roll/(2*arm))-(tau_pitch/(2*arm))-(tau_yaw/(4*k_torque))
    return np.clip([m1,m2,m3,m4],0,None)

# ------------------------
# Simulation
# ------------------------

def simulate_tracking(traj_coeffs,segment_times,total_time,dt):
    controller=PIDController(kp=2.0,ki=0.05,kd=1.0,dt=dt)
    steps=int(total_time/dt)
    times=np.linspace(0,total_time,steps)
    pos=np.array([0.,0.,0.])
    vel=np.array([0.,0.,0.])
    mass=10.0

    pos_log=[];ref_log=[];err_log=[];thrust_log=[];roll_log=[];pitch_log=[];motor_log=[]

    for t in times:
        pos_ref=get_position(traj_coeffs,segment_times,t)
        error=pos_ref-pos
        acc_cmd=controller.update(error)

        thrust,phi,theta,psi=accel_to_attitude(acc_cmd,mass)
        motors=motor_mixing(thrust,phi,theta,psi)

        # Update simplified dynamics (only translational motion)
        acc=acc_cmd
        vel+=acc*dt
        pos+=vel*dt

        pos_log.append(pos.copy())
        ref_log.append(pos_ref.copy())
        err_log.append(np.linalg.norm(error))
        thrust_log.append(thrust)
        roll_log.append(np.degrees(phi))
        pitch_log.append(np.degrees(theta))
        motor_log.append(motors)

    return times,np.array(pos_log),np.array(ref_log),np.array(err_log),thrust_log,roll_log,pitch_log,np.array(motor_log)

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

    velocities=compute_continuous_velocities(waypoints,segment_times)
    traj_coeffs=build_trajectory_with_velocity_continuity(waypoints,segment_times,velocities)
    total_time=sum(segment_times)
    dt=0.05  # 20 Hz sim

    times,pos_log,ref_log,err_log,thrust_log,roll_log,pitch_log,motor_log=simulate_tracking(
        traj_coeffs,segment_times,total_time,dt)

    # -------- PLOTS --------
    fig=plt.figure(figsize=(15,8))

    # 3D path
    ax1=fig.add_subplot(231,projection='3d')
    ax1.plot(ref_log[:,0],ref_log[:,1],ref_log[:,2],label='Reference',color='blue')
    ax1.plot(pos_log[:,0],pos_log[:,1],pos_log[:,2],'--',label='Tracked',color='red')
    ax1.set_title("3D Trajectory");ax1.legend();ax1.grid(True)

    # Tracking error
    ax2=fig.add_subplot(232)
    ax2.plot(times,err_log,color='green')
    ax2.set_title("Position Error");ax2.set_xlabel("Time (s)");ax2.set_ylabel("Error (m)");ax2.grid(True)

    # Thrust command
    ax3=fig.add_subplot(233)
    ax3.plot(times,thrust_log,color='purple')
    ax3.set_title("Thrust Command");ax3.set_xlabel("Time (s)");ax3.set_ylabel("Thrust (N)");ax3.grid(True)

    # Attitude commands
    ax4=fig.add_subplot(234)
    ax4.plot(times,roll_log,label="Roll")
    ax4.plot(times,pitch_log,label="Pitch")
    ax4.set_title("Attitude Commands");ax4.set_xlabel("Time (s)");ax4.set_ylabel("Angle (deg)")
    ax4.legend();ax4.grid(True)

    # Motor outputs
    ax5=fig.add_subplot(235)
    ax5.plot(times,motor_log[:,0],label="Motor1")
    ax5.plot(times,motor_log[:,1],label="Motor2")
    ax5.plot(times,motor_log[:,2],label="Motor3")
    ax5.plot(times,motor_log[:,3],label="Motor4")
    ax5.set_title("Motor Thrusts");ax5.set_xlabel("Time (s)");ax5.set_ylabel("Force (N)")
    ax5.legend();ax5.grid(True)

    plt.tight_layout()
    plt.show()

if __name__=="__main__":
    main()
