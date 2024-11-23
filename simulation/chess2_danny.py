import time
import asyncio
import numpy as np
import pybullet as p
import random
import math

from scipy.interpolate import CubicSpline
from simulation.robotiq import PickPlaceEnv, RRTPathPlanner, Pikafish, Robotiq2F85  # Import Pikafish directly from robotiq

# Constants for RRT
MAX_ITERS = 5000
DELTA_Q = 0.2
STEER_GOAL_P = 0.7
NUM_TRIALS = 1

# PyBullet Initialization
def initialize_pybullet():
    """Initialize PyBullet with GUI, gravity, and camera settings."""
    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)
    set_camera()

def set_camera():
    """Set the camera to visualize the chessboard better."""
    camera_distance = 1.2
    camera_yaw = 0
    camera_pitch = -80
    camera_target_position = [0.08, -0.05, 0.6]
    p.resetDebugVisualizerCamera(
        cameraDistance=camera_distance,
        cameraYaw=camera_yaw,
        cameraPitch=camera_pitch,
        cameraTargetPosition=camera_target_position
    )

# Convert chess notation to coordinates in the simulation
def chess_notation_to_coordinates(pos, env):
    """Convert chess notation (e.g., 'h2') to simulation coordinates."""
    col = ord(pos[0]) - ord('a')  # Convert 'a'-'i' to index 0-8
    row = int(pos[1])# Convert '1'-'10' to index 0-9
    return env.chessboard_positions[row][col]

# Path Planning with RRT
def rrt(q_init, q_goal, max_iters, delta_q, steer_goal_p, env):
    """
    Plan a path using RRT from start to goal configuration.
    """
    V, E = [q_init], []
    path, found = [], False

    for i in range(max_iters):
        # Sample a random configuration with a bias towards the goal
        q_rand = semi_random_sample(steer_goal_p, q_goal)
        q_nearest = nearest(V, q_rand)
        q_new = steer(q_nearest, q_rand, delta_q)
        if not check_collision(env, q_new):
            if q_new not in V:
                V.append(q_new)
            if (q_nearest, q_new) not in E:
                E.append((q_nearest, q_new))
                visualize_path_segment(q_nearest, q_new, env)
            if get_euclidean_distance(q_goal, q_new) < delta_q:
                V.append(q_goal)
                E.append((q_new, q_goal))
                visualize_path_segment(q_new, q_goal, env)
                found = True
                break

    if found:
        # Backtrack from the goal to the start to reconstruct the path
        current_q = q_goal
        path.append(current_q)
        while current_q != q_init:
            for edge in E:
                if edge[1] == current_q:
                    current_q = edge[0]
                    path.append(edge[0])
        path.reverse()
        return path
    else:
        return None

# Visualize path between two configurations
def visualize_path_segment(q_1, q_2, env, color=[0, 1, 0]):
    """Visualize the path segment between two configurations in the simulation."""
    set_joint_positions(env, q_1)
    point_1 = p.getLinkState(env.robot_id, env.tip_link_id)[0]
    set_joint_positions(env, q_2)
    point_2 = p.getLinkState(env.robot_id, env.tip_link_id)[0]
    p.addUserDebugLine(point_1, point_2, color, 1.0)

# Sampling function for RRT
def semi_random_sample(steer_goal_p, q_goal):
    """Generate a random sample with a probability of steering towards the goal."""
    prob = random.random()
    if prob < steer_goal_p:
        return q_goal
    else:
        return [random.uniform(-np.pi, np.pi) for _ in range(len(q_goal))]

# Get Euclidean distance between two configurations
def get_euclidean_distance(q1, q2):
    """Calculate the Euclidean distance between two configurations."""
    return math.sqrt(sum((q2[i] - q1[i]) ** 2 for i in range(len(q1))))

# Find the nearest configuration in the current tree
def nearest(V, q_rand):
    """Find the nearest configuration in the tree to the given random configuration."""
    return min(V, key=lambda v: get_euclidean_distance(v, q_rand))

# Steer from q_nearest towards q_rand by a given step size
def steer(q_nearest, q_rand, delta_q):
    """Steer from the nearest configuration towards the random configuration."""
    if get_euclidean_distance(q_rand, q_nearest) <= delta_q:
        return q_rand
    else:
        q_hat = [(q_rand[i] - q_nearest[i]) / get_euclidean_distance(q_rand, q_nearest) for i in range(len(q_rand))]
        return [q_nearest[i] + q_hat[i] * delta_q for i in range(len(q_hat))]

# Check for collisions
def check_collision(env, config):
    """Check if the given configuration results in a collision."""
    set_joint_positions(env, config)
    points = p.getContactPoints(bodyA=env.robot_id)
    for point in points:
        if point[2] != env.robot_id:  # Ensure it's not self-collision
            return True
    return False

# Trajectory Planning using Cubic Spline
def plan_trajectory(path, duration=5.0, timestep=0.05):
    """Plan a trajectory using cubic spline interpolation."""
    num_points = len(path)
    times = np.linspace(0, duration, num_points)
    cs = [CubicSpline(times, [p[i] for p in path]) for i in range(len(path[0]))]
    t_vals = np.arange(0, duration, timestep)
    trajectory = [[cs_i(t) for cs_i in cs] for t in t_vals]
    return trajectory

# Inverse Dynamics Control
def inverse_dynamics_control(env, target_positions):
    """Use inverse dynamics to control the robot arm to follow target positions."""
    qdot = [0.0] * len(env.joint_ids)
    for target in target_positions:
        torques = p.calculateInverseDynamics(env.robot_id, target, qdot, [0.0] * len(qdot))
        # Add check for maximum joint torque limits
        max_torque = 200.0  # Example maximum torque limit
        torques = [max(min(torque, max_torque), -max_torque) for torque in torques]
        # Check if the target positions are feasible to avoid collisions or unachievable configurations
        if not check_collision(env, target):
            p.setJointMotorControlArray(
                bodyIndex=env.robot_id,
                jointIndices=env.joint_ids,
                controlMode=p.TORQUE_CONTROL,
                forces=torques
            )
            p.stepSimulation()
            time.sleep(1 / 240)

# Move the robot to pick up and place the chess piece
def pick_and_place(env, start_coords, end_coords):
    """Move the robot to pick up and place the chess piece."""
    # aabb_min, aabb_max = p.getAABB(chesspiece_id)
    # piece_height = aabb_max[2] - aabb_min[2]  # Height along the Z-axis
    # print(f"Chess piece height: {piece_height} meters")
    piece_height = 0.0454 # from blend * 0.2 scaled
    safe_height = 1.2  # Increased safe height to avoid collision with table or pieces
    fine_tune_height = 0.1  # Increased fine-tune height for better clearance

    # Step 1: Move to the start position and grasp the chess piece
    hover_start = np.array(start_coords)
    hover_start[2] = safe_height
    start_coords_lower = np.array(start_coords)
    start_coords_lower[2] -= fine_tune_height
    start_config_lower = p.calculateInverseKinematics(env.robot_id, env.tip_link_id, start_coords_lower)
    if not check_collision(env, start_config_lower):
        set_joint_positions(env, start_config_lower)
        time.sleep(0.2)
        env.gripper.activate()
        time.sleep(0.5)

    # Step 2: Detach the gripper and leave it at the start position
    env.gripper.release()
    env.gripper.detach()
    time.sleep(0.5)

    # Step 3: Perform RRT path planning without the gripper
    start_config_hover = p.calculateInverseKinematics(env.robot_id, env.tip_link_id, hover_start)
    hover_end = np.array(end_coords)
    hover_end[2] = safe_height
    goal_config_hover = p.calculateInverseKinematics(env.robot_id, env.tip_link_id, hover_end)
    # Ensure the goal configuration is feasible
    if not check_collision(env, start_config_hover) and not check_collision(env, goal_config_hover):
        path = rrt(start_config_hover, goal_config_hover, MAX_ITERS, DELTA_Q, STEER_GOAL_P, env)
    else:
        path = None

    if path is None:
        print("No collision-free path found to target position.")
        return

    # Step 4: Backtrace to the start position and reattach the gripper
    trajectory = plan_trajectory(path[::-1])  # Reverse the path to backtrace
    inverse_dynamics_control(env, trajectory)
    env.gripper.attach(tool_position=[0, 0, 0])  # Reattach gripper at the appropriate position
    env.gripper.activate()
    time.sleep(0.5)

    # Step 5: Follow the path to the goal position with the chess piece
    trajectory = plan_trajectory(path)
    inverse_dynamics_control(env, trajectory)

    # Step 6: Lower the chess piece and release it
    end_coords_lower = np.array(end_coords)
    end_coords_lower[2] -= fine_tune_height
    goal_config_lower = p.calculateInverseKinematics(env.robot_id, env.tip_link_id, end_coords_lower)
    if not check_collision(env, goal_config_lower):
        set_joint_positions(env, goal_config_lower)
        env.gripper.release()
        time.sleep(0.5)

    # Move the arm back to a safe hover position
    set_joint_positions(env, goal_config_hover)

# Follow the path by moving through each configuration
def follow_path(path, env):
    """Follow a given path by moving through each configuration."""
    trajectory = plan_trajectory(path)
    inverse_dynamics_control(env, trajectory)

# Set joint positions
def set_joint_positions(env, joint_positions):
    """Set the joint positions of the robot arm."""
    p.setJointMotorControlArray(
        bodyIndex=env.robot_id,
        jointIndices=env.joint_ids,
        controlMode=p.POSITION_CONTROL,
        targetPositions=joint_positions,
        forces=[500] * len(env.joint_ids)
    )
    for _ in range(50):
        p.stepSimulation()
        time.sleep(1 / 240)

# Main Functionality (Asynchronous)
async def main():
    """Main function to initialize the simulation and run the pick and place operations."""
    initialize_pybullet()

    env = PickPlaceEnv(render=True, high_res=False, high_frame_rate=False)
    env.reset([])

    engine = Pikafish()

    while True:
        # Get the best move from the Pikafish engine
        move = await engine.get_best_move(env.board_to_fen(env.board, True))

        if not move or len(move) != 4:
            print("Invalid move received. Ending simulation.")
            break

        print(f"Best move: {move}")  # Print the best move
        start_pos = move[:2]
        end_pos = move[2:]

        # Convert chess notation to simulation coordinates
        start_coords = chess_notation_to_coordinates(start_pos, env)
        end_coords = chess_notation_to_coordinates(end_pos, env)

        # Perform the pick and place operation
        pick_and_place(env, start_coords, end_coords)

        # Update the environment observations
        env.update_observations()

        # Wait before the next move
        time.sleep(2)

    print("Simulation complete. Press Ctrl+C to exit.")
    while True:
        p.stepSimulation()
        time.sleep(1 / 240)

# Run Main Function (Asynchronous)
if __name__ == "__main__":
    asyncio.run(main())
