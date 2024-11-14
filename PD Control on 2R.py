import mujoco_py
import numpy as np

# Load the model and create a simulation instance
model = mujoco_py.load_model_from_path("/home/samriddhi/leap_hand_mujoco/model/leap hand/2R.xml")
sim = mujoco_py.MjSim(model)
viewer = mujoco_py.MjViewer(sim)

# Constants based on XML file specifications
g = 9.81  # gravitational acceleration
m1 = 0.001  # mass of the first link
m2 = 0.001  # mass of the second link
l1 = 0.5  # length of the first link (from XML)
l2 = 0.5  # length of the second link (from XML)
r1 = l1 / 2  # center of mass for the first link (assuming uniform)
r2 = l2 / 2  # center of mass for the second link (assuming uniform)

# Set initial joint positions (e.g., at the desired start position)
def initialize_joints(sim):
    # Set initial joint positions
    sim.data.qpos[0] = np.pi / 6  # Joint 1 initial position (45 degrees)
    sim.data.qpos[1] = np.pi / 3  # Joint 2 initial position (60 degrees)
    
    # Optionally set initial joint velocities (zero velocities here)
    sim.data.qvel[0] = 0
    sim.data.qvel[1] = 0

# Compute gravity compensation torques with PD control
def compute_PD_control(sim):
    # Target joint positions (desired state)
    theta_desired = np.array([0, 0])  # Desired position for joints 1 and 2 (0 radians)
    
    # Proportional (P) and Derivative (D) gains
    Kp = np.diag([20, 20])  # Lower proportional gains for smoother control
    Kd = np.diag([10, 10])  # Lower derivative gains for smoother control

    # Current joint positions and velocities
    theta_current = sim.data.qpos[:2]  # Current joint positions
    theta_dot = sim.data.qvel[:2]      # Current joint velocities

    # Calculate the error (position error and velocity error)
    error = theta_desired - theta_current  # Position error
    error_dot = -theta_dot                # Velocity error (negative of joint velocity)

    # PD control law: tau = Kp * error + Kd * error_dot
    tau = Kp @ error + Kd @ error_dot

    # Apply the torques to the actuators
    sim.data.ctrl[0] = tau[0]  # Apply torque to joint1 motor
    sim.data.ctrl[1] = tau[1]  # Apply torque to joint2 motor

# Initialize joint positions before starting the control
initialize_joints(sim)

# Simulation loop
for i in range(10000):  # Run for 10000 steps or as needed
    # Compute and apply PD control torques
    compute_PD_control(sim)
    
    # Perform a step in the simulation
    sim.step()

    # Render the simulation in the viewer
    viewer.render()

    # Print torque values at intervals to verify
    if i % 100 == 0:
        print(f"Step {i}: Joint1 torque = {sim.data.ctrl[0]:.4f}, Joint2 torque = {sim.data.ctrl[1]:.4f}")
