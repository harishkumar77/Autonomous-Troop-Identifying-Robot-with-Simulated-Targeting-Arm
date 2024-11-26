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

# Function to get the joint IDs for the two arms (left and right)
def get_arm_joint_ids(robot, arm_prefix="left"):
    arm_joint_ids = []
    for joint_id in range(num_joints):
        joint_name = p.getJointInfo(robot, joint_id)[1].decode("utf-8")
        if arm_prefix in joint_name:
            arm_joint_ids.append(joint_id)
    return arm_joint_ids

# Get left and right arm joint IDs
left_arm_joints = get_arm_joint_ids(baxter_robot, "left")
right_arm_joints = get_arm_joint_ids(baxter_robot, "right")

# Define default joint positions for both arms
default_left_arm_positions = np.deg2rad([90, -20, 50, 0, 45, 60, 0])  # Left arm default posture
default_right_arm_positions = np.deg2rad([90, 20, -50, 0, -45, -60, 0])  # Right arm default posture

# Define the target positions for both arms for command == 1 and right
shooting_left_arm_positions = np.deg2rad([90,90, -45, 0, 0, 0, 0])  # Left arm pointing straight forward (shooting posture)
downward_right_arm_positions = np.deg2rad([0, -90, -45, 0, 0, 0, 0])  # Right arm facing completely downwards

# Define the target positions for both arms for command == 1 and left
shooting_left_arm_positions_left = np.deg2rad([0, 90, 45, 0, 0, 0, 0])  # Left arm pointing straight forward (shooting posture)
downward_right_arm_positions_left = np.deg2rad([-90,-90, 45, 0, 0, 0, 0])  # Right arm facing completely downwards

# Define pointing positions for both arms (used for command > 1)
pointing_left_arm_positions = np.deg2rad([90,90, -45, 0, 0, 0, 0])
pointing_right_arm_positions = np.deg2rad([-90,-90, 45, 0, 0, 0, 0])

# Function to read the command file
def read_commands(file_path):
    try:
        with open(file_path, 'r') as f:
            content = f.read().strip()
            return int(content)  # Assuming the file contains an integer
    except Exception as e:
        print(f"Error reading command file: {e}")
        return 0  # Default command

# Main simulation loop
while True:
    # Step the simulation
    p.stepSimulation()

    # Read the command file
    command = read_commands(command_file)

    # Determine which arms to move based on the command
    if command == 71:
    # Move left arm to shooting posture and right arm to downwards position
        for joint_id, target_angle in zip(left_arm_joints, shooting_left_arm_positions):
            p.setJointMotorControl2(baxter_robot, joint_id, p.POSITION_CONTROL, targetPosition=target_angle)

        for joint_id, target_angle in zip(right_arm_joints, downward_right_arm_positions):
            p.setJointMotorControl2(baxter_robot, joint_id, p.POSITION_CONTROL, targetPosition=target_angle)

    elif command == 17:
    # Move left arm to shooting posture and right arm to downwards position
        for joint_id, target_angle in zip(left_arm_joints, shooting_left_arm_positions_left):
            p.setJointMotorControl2(baxter_robot, joint_id, p.POSITION_CONTROL, targetPosition=target_angle)
 
        for joint_id, target_angle in zip(right_arm_joints, downward_right_arm_positions_left):
            p.setJointMotorControl2(baxter_robot, joint_id, p.POSITION_CONTROL, targetPosition=target_angle)


    elif command >= 3:
        # Move both arms to pointing positions
        for joint_id, target_angle in zip(left_arm_joints, pointing_left_arm_positions):
            p.setJointMotorControl2(baxter_robot, joint_id, p.POSITION_CONTROL, targetPosition=target_angle)

        for joint_id, target_angle in zip(right_arm_joints, pointing_right_arm_positions):
            p.setJointMotorControl2(baxter_robot, joint_id, p.POSITION_CONTROL, targetPosition=target_angle)

    else:
        # Default posture for both arms
        for joint_id, target_angle in zip(left_arm_joints, default_left_arm_positions):
            p.setJointMotorControl2(baxter_robot, joint_id, p.POSITION_CONTROL, targetPosition=target_angle)

        for joint_id, target_angle in zip(right_arm_joints, default_right_arm_positions):
            p.setJointMotorControl2(baxter_robot, joint_id, p.POSITION_CONTROL, targetPosition=target_angle)

    # Simulate at 240 Hz
    time.sleep(1 / 240)

# Disconnect from PyBullet when done
p.disconnect()

