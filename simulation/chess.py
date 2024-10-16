import os
import time

import numpy as np
import pybullet
from stretch import Robot

pybullet.connect(pybullet.GUI)
pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 1)

pybullet.setGravity(0, 0, -1.81)

def initAxis(center, quater):
    rot_mat = pybullet.getMatrixFromQuaternion(quater)
    rotmat = np.array(rot_mat).reshape((3, 3))
    pybullet.addUserDebugLine(lineFromXYZ=center, lineToXYZ=center + rotmat[:3, 0] * 0.1, lineColorRGB=[1, 0, 0],
                            lineWidth=10)
    pybullet.addUserDebugLine(lineFromXYZ=center, lineToXYZ=center + rotmat[:3, 1] * 0.1, lineColorRGB=[0, 1, 0],
                            lineWidth=10)
    pybullet.addUserDebugLine(lineFromXYZ=center, lineToXYZ=center + rotmat[:3, 2] * 0.1, lineColorRGB=[0, 0, 1],
                            lineWidth=10)


root_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)),"../")

################ Plane Environment
plane_id = pybullet.loadURDF(os.path.join(root_dir,"resource/urdf/plane.urdf"), [0, 0, 0])
plane_texture_id = pybullet.loadTexture(os.path.join(root_dir,"resource/texture/texture1.jpg"))
pybullet.changeVisualShape(0,-1,textureUniqueId=plane_texture_id)

################ Robot
mobot_urdf_file = os.path.join(root_dir, "resource/urdf/stretch/stretch.urdf")
mobot = Robot(start_pos=[0.0,0.8,0.05], urdf_file=mobot_urdf_file)

for _ in range(30):
    pybullet.stepSimulation()


urdf_dir = os.path.join(root_dir,"resource/urdf")

# table initialization

table_position = [0, 0, 0]
table_scaling = 1.0
table_orientation = pybullet.getQuaternionFromEuler([0, 0, np.pi])
table_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"table.urdf"),\
                                   useFixedBase=True,
                                   basePosition=table_position,\
                                   baseOrientation=table_orientation,\
                                   globalScaling=table_scaling)
table_texture_id = pybullet.loadTexture(os.path.join(root_dir,"resource/texture/table.png"))
pybullet.changeVisualShape(table_id,0,textureUniqueId=table_texture_id)


for _ in range(20):
    pybullet.stepSimulation()

pybullet.changeVisualShape(mobot.robot_id,0,rgbaColor=[1,0,0,1])
pybullet.changeVisualShape(mobot.robot_id,1,rgbaColor=[0,1,0,1])

pybullet.setRealTimeSimulation(1)

# board simulation
board_position = [table_position[0] + 0.08, table_position[1] - 0.05, 0.6]
board_scaling = 0.5
board_orientation = pybullet.getQuaternionFromEuler([0, 0, np.pi])
board_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chessboard/chessboard.urdf"),\
                                   useFixedBase=True,
                                   basePosition=board_position,\
                                   baseOrientation=board_orientation,\
                                   globalScaling=board_scaling)
pybullet.changeVisualShape(board_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])

# CHESS PIECES #############
# Black pieces
b_orientation = pybullet.getQuaternionFromEuler([0, 0, np.pi / 2])
cp_scaling = 0.23
obj_friction_ceof = 4000.0

#b1
b1_position = [table_position[0] - 0.015, table_position[1] - 0.37, 0.7]
b1_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b1/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b1_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b1_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b1_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b1_id, -1, mass=0.01)

#b2
b2_position = [table_position[0] + 0.07, table_position[1] - 0.37, 0.7]
b2_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b2/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b2_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b2_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b2_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b2_id, -1, mass=0.01)

#b3
b3_position = [table_position[0] - 0.10, table_position[1] - 0.37, 0.7]
b3_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b3/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b3_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b3_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b3_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b3_id, -1, mass=0.01)

#b4
b4_position = [table_position[0] + 0.155, table_position[1] - 0.37, 0.7]
b4_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b4/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b4_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b4_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b4_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b4_id, -1, mass=0.01)

#b5
b5_position = [table_position[0] - 0.185, table_position[1] - 0.37, 0.7]
b5_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b5/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b5_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b5_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b5_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b5_id, -1, mass=0.01)

#b6
b6_position = [table_position[0] + 0.242, table_position[1] - 0.37, 0.7]
b6_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b6/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b6_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b6_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b6_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b6_id, -1, mass=0.01)

#b7
b7_position = [table_position[0] - 0.27, table_position[1] - 0.37, 0.7]
b7_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b7/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b7_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b7_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b7_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b7_id, -1, mass=0.01)

#b8
b8_position = [table_position[0] + 0.327, table_position[1] - 0.37, 0.7]
b8_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b8/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b8_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b8_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b8_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b8_id, -1, mass=0.01)

#b9
b9_position = [table_position[0] - 0.355, table_position[1] - 0.37, 0.7]
b9_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b9/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b9_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b9_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b9_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b9_id, -1, mass=0.01)

#b10
b10_position = [table_position[0] + 0.242, table_position[1] - 0.2, 0.7]
b10_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b10/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b10_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b10_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b10_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b10_id, -1, mass=0.01)

#b11
b11_position = [table_position[0] - 0.27, table_position[1] - 0.2, 0.7]
b11_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b11/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b11_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b11_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b11_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b11_id, -1, mass=0.01)

# black soldiers
#b12
b12_position = [table_position[0] + 0.327, table_position[1] - 0.12, 0.7]
b12_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b12/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b12_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b12_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b12_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b12_id, -1, mass=0.01)

#b13
b13_position = [table_position[0] + 0.155, table_position[1] - 0.12, 0.7]
b13_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b13/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b13_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b13_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b13_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b13_id, -1, mass=0.01)

#b14
b14_position = [table_position[0] - 0.015, table_position[1] - 0.12, 0.7]
b14_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b14/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b14_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b14_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b14_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b14_id, -1, mass=0.01)

#b15
b15_position = [table_position[0] - 0.185, table_position[1] - 0.12, 0.7]
b15_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b15/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b15_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b15_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b15_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b15_id, -1, mass=0.01)

#b16
b16_position = [table_position[0] - 0.355, table_position[1] - 0.12, 0.7]
b16_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b16/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b16_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b16_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b16_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b16_id, -1, mass=0.01)
#############################

# RED PIECES

r_orientation = pybullet.getQuaternionFromEuler([0, 0, np.pi * 3 / 2])
cp_scaling = 0.23
obj_friction_ceof = 4000.0

#r1
r1_position = [table_position[0] - 0.015, table_position[1] + 0.4, 0.7]
r1_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r1/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r1_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r1_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r1_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r1_id, -1, mass=0.01)

#r2
r2_position = [table_position[0] + 0.07, table_position[1] + 0.4, 0.7]
r2_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r2/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r2_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r2_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r2_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r2_id, -1, mass=0.01)

#r3
r3_position = [table_position[0] - 0.10, table_position[1] + 0.4, 0.7]
r3_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r3/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r3_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r3_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r3_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r3_id, -1, mass=0.01)

#r4
r4_position = [table_position[0] + 0.155, table_position[1] + 0.4, 0.7]
r4_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r4/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r4_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r4_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r4_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r4_id, -1, mass=0.01)

#r5
r5_position = [table_position[0] - 0.185, table_position[1] + 0.4, 0.7]
r5_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r5/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r5_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r5_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r5_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r5_id, -1, mass=0.01)

#r6
r6_position = [table_position[0] + 0.242, table_position[1] + 0.4, 0.7]
r6_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r6/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r6_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r6_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r6_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r6_id, -1, mass=0.01)

#r7
r7_position = [table_position[0] - 0.27, table_position[1] + 0.4, 0.7]
r7_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r7/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r7_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r7_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r7_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r7_id, -1, mass=0.01)

#r8
r8_position = [table_position[0] + 0.327, table_position[1] + 0.4, 0.7]
r8_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r8/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r8_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r8_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r8_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r8_id, -1, mass=0.01)

#r9
r9_position = [table_position[0] - 0.355, table_position[1] + 0.4, 0.7]
r9_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r9/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r9_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r9_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r9_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r9_id, -1, mass=0.01)

#r10
r10_position = [table_position[0] + 0.242, table_position[1] + 0.22, 0.7]
r10_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r10/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r10_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r10_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r10_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r10_id, -1, mass=0.01)

#r11
r11_position = [table_position[0] - 0.27, table_position[1] + 0.22, 0.7]
r11_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r11/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r11_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r11_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r11_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r11_id, -1, mass=0.01)

# red soldiers
#r12
r12_position = [table_position[0] + 0.327, table_position[1] + 0.14, 0.7]
r12_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r12/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r12_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r12_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r12_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r12_id, -1, mass=0.01)

#r13
r13_position = [table_position[0] + 0.155, table_position[1] + 0.14, 0.7]
r13_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r13/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r13_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r13_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r13_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r13_id, -1, mass=0.01)

#r14
r14_position = [table_position[0] - 0.015, table_position[1] + 0.14, 0.7]
r14_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r14/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r14_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r14_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r14_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r14_id, -1, mass=0.01)

#r15
r15_position = [table_position[0] - 0.185, table_position[1] + 0.14, 0.7]
r15_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r15/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r15_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r15_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r15_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r15_id, -1, mass=0.01)

#r16
r16_position = [table_position[0] - 0.355, table_position[1] + 0.14, 0.7]
r16_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r16/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r16_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r16_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r16_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r16_id, -1, mass=0.01)
#############################


# for j in range(pybullet.getNumJoints(mobot.robotId)):
#     print(pybullet.getJointInfo(mobot.robotId,j))
forward=0
turn=0
speed=10

for _ in range(30):
    pybullet.stepSimulation()


mobot.get_observation()

while (1):
    time.sleep(1./240.)
    keys = pybullet.getKeyboardEvents()
    leftWheelVelocity=0
    rightWheelVelocity=0

    for k,v in keys.items():

        if (k == pybullet.B3G_RIGHT_ARROW and (v&pybullet.KEY_WAS_TRIGGERED)):
                turn = -0.8
        if (k == pybullet.B3G_RIGHT_ARROW and (v&pybullet.KEY_WAS_RELEASED)):
                turn = 0
        if (k == pybullet.B3G_LEFT_ARROW and (v&pybullet.KEY_WAS_TRIGGERED)):
                turn = 0.8
        if (k == pybullet.B3G_LEFT_ARROW and (v&pybullet.KEY_WAS_RELEASED)):
                turn = 0

        if (k == pybullet.B3G_UP_ARROW and (v&pybullet.KEY_WAS_TRIGGERED)):
                forward=1
        if (k == pybullet.B3G_UP_ARROW and (v&pybullet.KEY_WAS_RELEASED)):
                forward=0
        if (k == pybullet.B3G_DOWN_ARROW and (v&pybullet.KEY_WAS_TRIGGERED)):
                forward=-1
        if (k == pybullet.B3G_DOWN_ARROW and (v&pybullet.KEY_WAS_RELEASED)):
                forward=0

    rightWheelVelocity+= (forward+turn)*speed
    leftWheelVelocity += (forward-turn)*speed
    
    pybullet.setJointMotorControl2(mobot.robot_id,0,pybullet.VELOCITY_CONTROL,targetVelocity=leftWheelVelocity,force=1000)
    pybullet.setJointMotorControl2(mobot.robot_id,1,pybullet.VELOCITY_CONTROL,targetVelocity=rightWheelVelocity,force=1000)

    mobot.get_observation()
    mobot.make_move()
