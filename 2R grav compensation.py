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

# Set initial positions and velocities to avoid instability

sim.data.qvel[0] = 0  # Joint 1 velocity
sim.data.qvel[1] = 0  # Joint 2 velocity

# Apply some damping and inertia (ensure these values are set correctly in the XML)
def compute_gravity_compensation(sim):
    sim.data.qpos[0] = np.pi/4  # Joint 1 position
    sim.data.qpos[1] = np.pi/6    # Joint 2 position
    
    # Read joint angles (in radians)
    q1 = sim.data.qpos[0]  # Joint 1 angle
    q2 = sim.data.qpos[1]  # Joint 2 angle

    # Set initial positions and velocities to avoid instability

    sim.data.qvel[0] = 0  # Joint 1 velocity
    sim.data.qvel[1] = 0  # Joint 2 velocity

    # Calculate the torques for gravity compensation using provided equations
    tau1 = (m1 * g * r1 * np.cos(q1)) + (m2 * g * (l1 * np.cos(q1) + r2 * np.cos(q1 + q2)))
    tau2 = m2 * g * r2 * np.cos(q1 + q2)

    # Apply torques to each joint for gravity compensation
    sim.data.ctrl[0] = tau1
    sim.data.ctrl[1] = tau2

# Simulation loop
for i in range(10000):  # Run for 10000 steps or as needed
    # Compute and apply gravity compensation torques
    compute_gravity_compensation(sim)
    sim.step()

    # Render the simulation in the viewer
    viewer.render()

    # Print torque values at intervals to verify
    if i % 100 == 0:
        print(f"Step {i}: Joint1 torque = {sim.data.ctrl[0]:.4f}, Joint2 torque = {sim.data.ctrl[1]:.4f}")
