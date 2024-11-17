import time

import numpy as np
import pybullet  # type: ignore

from simulation.robotiq import *

#from engine.pikafish import Pikafish

pybullet.connect(pybullet.GUI)
pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 1)
pybullet.setGravity(0, 0, -9.81)
pybullet.setRealTimeSimulation(1)

# ################ DEBUG CAMERA SETTINGS

camera_distance = 1.2  # Closer to the chessboard
camera_yaw = 0  # Rotate around the Z-axis
camera_pitch = -80  # Tilt the camera downward
camera_target_position = [0.08, -0.05, 0.6]  # Center of the chessboard

# Apply camera settings
pybullet.resetDebugVisualizerCamera(
    cameraDistance=camera_distance,
    cameraYaw=camera_yaw,
    cameraPitch=camera_pitch,
    cameraTargetPosition=camera_target_position
)
########################


def initAxis(center, quater):
    rot_mat = pybullet.getMatrixFromQuaternion(quater)
    rotmat = np.array(rot_mat).reshape((3, 3))
    pybullet.addUserDebugLine(lineFromXYZ=center, lineToXYZ=center + rotmat[:3, 0] * 0.1, lineColorRGB=[1, 0, 0],
                            lineWidth=10)
    pybullet.addUserDebugLine(lineFromXYZ=center, lineToXYZ=center + rotmat[:3, 1] * 0.1, lineColorRGB=[0, 1, 0],
                            lineWidth=10)
    pybullet.addUserDebugLine(lineFromXYZ=center, lineToXYZ=center + rotmat[:3, 2] * 0.1, lineColorRGB=[0, 0, 1],
                            lineWidth=10)





forward=0
speed=10
turn=0
up=0
stretch=0

env = PickPlaceEnv(render=True, high_res=False, high_frame_rate=False)
env.reset([])

def arm_control(env, pybullet, up, stretch):
    joints = pybullet.getJointStates(env.robot_id, env.joint_ids)
    joints = [j[0] for j in joints]
    joints[0] += up * 0.01
    joints[1] += stretch * 0.01
    env.servoj(joints)

pybullet.setRealTimeSimulation(1)

for _ in range(30):
    pybullet.stepSimulation()
    
#is_our_turn = True
while True:
    time.sleep(1/240)
    speed = 20
    
    for keycode, keystate in pybullet.getKeyboardEvents().items():
        # gripper
        # open gripper when 'o' is pressed
        if (keycode == ord('o') and (keystate & pybullet.KEY_WAS_TRIGGERED)):
            env.gripper.release()
        # close gripper when 'c' is pressed
        if (keycode == ord('c') and (keystate & pybullet.KEY_WAS_TRIGGERED)):
            env.gripper.activate()

    print(env.make_move())
    # is_our_turn = not is_our_turn # like this? idk
    
    # test pick and place of h2e2
    #env.step(action={'pick': np.array([0.237, -0.206, 0.66]), 'place': np.array([0.065, -0.206, 0.66])})
    #arm_control(env, pybullet, up, stretch)

    # base_control(mobot, pybullet, forward, turn)
    # arm_control(mobot, pybullet, up, stretch, roll, yaw)

    # mobot.update_observations()

    # current_position, _, _ = mobot.get_robot_base_pose(mobot.robot_id)
    # total_driving_distance += np.linalg.norm(np.array(current_position) - np.array(previous_position))
    # previous_position = current_position

    # mobot.update_observations()
    # mobot.make_move()

    # ee_position, _ = mobot.get_robot_ee_pose(mobot.robot_id)
    # # print("End-effector position: ", ee_position)
