import pybullet as p
import time
import numpy as np

# Path to the Baxter URDF file
urdf_path = "/home/harishkumar/Downloads/pybullet_robots/data/baxter_common/baxter_description/urdf/baxter.urdf"

# Command file to monitor
command_file = "/home/harishkumar/Downloads/detection_summary.txt"

# Connect to PyBullet in GUI mode
p.connect(p.GUI)

# Set the search path for URDF files
p.setAdditionalSearchPath("/home/harishkumar/Downloads/pybullet_robots/data/baxter_common/baxter_description/urdf")

# Load the Baxter URDF file (with the fixed base)
baxter_robot = p.loadURDF(urdf_path, useFixedBase=True)

# Set gravity
p.setGravity(0, 0, -9.81)

# Get the number of joints in the Baxter robot
num_joints = p.getNumJoints(baxter_robot)

# Filter out fixed joints
movable_joints = [j for j in range(num_joints) if p.getJointInfo(baxter_robot, j)[2] != p.JOINT_FIXED]

# Manually map left and right arm joints
left_arm_joints = movable_joints[:7]  # Adjust based on URDF if incorrect
right_arm_joints = movable_joints[7:14]  # Adjust based on URDF if incorrect

# Define default joint positions for both arms
default_left_arm_positions = np.deg2rad([90, -20, 50, 0, 45, 60, 0])  # Left arm default posture
default_right_arm_positions = np.deg2rad([90, 20, -50, 0, -45, -60, 0])  # Right arm default posture

# Define the target positions for command == 1
shooting_left_arm_positions = np.deg2rad([90, 90, -45, 0, 0, 0, 0])
downward_right_arm_positions = np.deg2rad([0, -90, -45, 0, 0, 0, 0])

# Define pointing positions for both arms (used for command > 1)
pointing_left_arm_positions = np.deg2rad([90, 90, -45, 0, 0, 0, 0])
pointing_right_arm_positions = np.deg2rad([-90, -90, 45, 0, 0, 0, 0])

Kp = 34 # Increased proportional gain for better responsiveness
Kd = 20  # Damping gain to reduce oscillations

# Torque limit
max_torque = 50.0

# Deadband tolerance
position_tolerance = 0.02  # Allowable position error (radians)
velocity_tolerance = 0.1   # Allowable velocity error (rad/s)

# Initialize joints in position control mode to reach default positions
for joint_id, target_angle in zip(left_arm_joints, default_left_arm_positions):
    p.setJointMotorControl2(baxter_robot, joint_id, p.POSITION_CONTROL, targetPosition=target_angle)

for joint_id, target_angle in zip(right_arm_joints, default_right_arm_positions):
    p.setJointMotorControl2(baxter_robot, joint_id, p.POSITION_CONTROL, targetPosition=target_angle)

# Stabilize the robot
time.sleep(3)

# Switch to PD Torque Control
for joint_id in left_arm_joints + right_arm_joints:
    p.setJointMotorControl2(baxter_robot, joint_id, p.VELOCITY_CONTROL, force=0)  # Disable velocity control
    p.setJointMotorControl2(baxter_robot, joint_id, p.TORQUE_CONTROL, force=0)    # Enable torque control

# Function to read the command file
def read_commands(file_path):
    try:
        with open(file_path, 'r') as f:
            content = f.read().strip()
            return int(content)  # Assuming the file contains an integer
    except Exception as e:
        print(f"Error reading command file: {e}")
        return 0  # Default command

# Function to apply PD control
def apply_pd_control(robot, joint_ids, target_positions, joint_positions, joint_velocities, gravity_torques):
    for joint_id, target_angle in zip(joint_ids, target_positions):
        joint_index = joint_ids.index(joint_id)
        current_position = joint_positions[joint_index]
        current_velocity = joint_velocities[joint_index]

        # Compute errors
        position_error = target_angle - current_position
        velocity_error = -current_velocity

        # Apply a deadband to stop small oscillations
        if abs(position_error) < position_tolerance and abs(velocity_error) < velocity_tolerance:
            total_torque = 0
        else:
            # PD control torque
            control_torque = Kp * position_error + Kd * velocity_error

            # Add gravity compensation torque
            gravity_comp_torque = gravity_torques[joint_index]
            total_torque = control_torque + gravity_comp_torque

            # Clamp the torque
            total_torque = max(min(total_torque, max_torque), -max_torque)

        # Apply torque
        p.setJointMotorControl2(robot, joint_id, p.TORQUE_CONTROL, force=total_torque)

# Main simulation loop
while True:
    # Step the simulation
    p.stepSimulation()

    # Read the command file
    command = read_commands(command_file)

    # Get current joint states
    joint_positions = [p.getJointState(baxter_robot, j)[0] for j in movable_joints]
    joint_velocities = [p.getJointState(baxter_robot, j)[1] for j in movable_joints]

    # Calculate gravity compensation torques
    gravity_torques = p.calculateInverseDynamics(
        baxter_robot, joint_positions, [0] * len(movable_joints), [0] * len(movable_joints)
    )

    # Determine which arms to move based on the command
    if command == 1:
        apply_pd_control(baxter_robot, left_arm_joints, shooting_left_arm_positions, joint_positions, joint_velocities, gravity_torques)
        apply_pd_control(baxter_robot, right_arm_joints, downward_right_arm_positions, joint_positions, joint_velocities, gravity_torques)
    elif command > 1:
        apply_pd_control(baxter_robot, left_arm_joints, pointing_left_arm_positions, joint_positions, joint_velocities, gravity_torques)
        apply_pd_control(baxter_robot, right_arm_joints, pointing_right_arm_positions, joint_positions, joint_velocities, gravity_torques)
    else:
        apply_pd_control(baxter_robot, left_arm_joints, default_left_arm_positions, joint_positions, joint_velocities, gravity_torques)
        apply_pd_control(baxter_robot, right_arm_joints, default_right_arm_positions, joint_positions, joint_velocities, gravity_torques)

    # Simulate at 240 Hz
    time.sleep(1 / 240)

# Disconnect from PyBullet when done
p.disconnect()

