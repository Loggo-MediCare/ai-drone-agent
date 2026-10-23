# 摘錄自《每位人工智慧工程師必須建構的 30 個智能體》第十六章，原文照錄（虛擬碼，無法直接執行）。
# Pseudocode: Lower-layer control skeletons: motion planning, trajectory control, and servo regulation

# --- Motion Planning layer (1-10 Hz, RRT/PRM) ---
def plan_motion_trajectory(goal_pose, world_model, obstacles):
    """Compute a collision-free path from current pose to goal.
    Frequency: 1-10 Hz.  Algorithm class: RRT*, PRM."""
    current_pose = world_model.get_current_state()["pose"]
    path = rrt_star(current_pose, goal_pose, obstacles)
    return path  # list of waypoints for trajectory layer


# --- Trajectory Control layer (50-200 Hz, PID/MPC) ---
def compute_trajectory_setpoints(path, dt=0.005):
    """Time-parameterise the path into setpoints at the
    control frequency.  Frequency: 50-200 Hz.
    Algorithm class: PID, model predictive control."""
    for waypoint in interpolate(path, dt):
        error = waypoint - read_encoder_pose()
        torque = pid_controller.compute(error, dt)
        yield torque  # consumed by servo layer


# --- Servo Control layer (1-10 kHz, current loops) ---
def servo_control_loop(torque_setpoint, dt=0.0001):
    """Regulate motor current to achieve commanded torque.
    Frequency: 1-10 kHz.  Rejects electrical and
    mechanical disturbances within fractions of a ms."""
    current = read_motor_current()
    target_current = torque_to_current(torque_setpoint)
    pwm_signal = current_pid.compute(
        target_current - current, dt
    )
    write_pwm(pwm_signal)
