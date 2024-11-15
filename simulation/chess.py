import os
import time
from collections import defaultdict

import numpy as np
import pybullet # type: ignore

from simulation.robot import *

pybullet.connect(pybullet.GUI)
pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 1)
# pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_WIREFRAME, 1) # for debugging

# pybullet.setGravity(0, 0, -1.81)
pybullet.setGravity(0, 0, -9.81)
pybullet.setRealTimeSimulation(1)


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
# plane_id = pybullet.loadURDF(os.path.join(root_dir,"resource/urdf/plane.urdf"), [0, 0, 0])
# plane_texture_id = pybullet.loadTexture(os.path.join(root_dir,"resource/texture/texture1.jpg"))
plane_id = pybullet.loadURDF("resource/urdf/plane.urdf", [0, 0, 0])
plane_texture_id = pybullet.loadTexture("resource/texture/texture1.jpg")
pybullet.changeVisualShape(0, -1, textureUniqueId=plane_texture_id)


################ Table
# urdf_dir = os.path.join(root_dir,"resource/urdf")
urdf_dir = "resource/urdf"

table_position = [0, 0, 0]
table_scaling = 1.0
table_orientation = pybullet.getQuaternionFromEuler([0, 0, np.pi])
table_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"table.urdf"),\
                                   useFixedBase=True,
                                   basePosition=table_position,\
                                   baseOrientation=table_orientation,\
                                   globalScaling=table_scaling)
# table_texture_id = pybullet.loadTexture(os.path.join(root_dir,"resource/texture/table.png"))
table_texture_id = pybullet.loadTexture("resource/texture/table.png")
pybullet.changeVisualShape(table_id,0,textureUniqueId=table_texture_id)

################ Board
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
PIECE_MASS = 0.01

# Black pieces
b_orientation = pybullet.getQuaternionFromEuler([0, 0, -np.pi / 2])
cp_scaling = 0.20
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
pybullet.changeDynamics(b1_id, -1, mass=PIECE_MASS)

#b2
b2_position = [table_position[0] + 0.07, table_position[1] - 0.37, 0.7]
b2_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b2/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b2_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b2_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b2_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b2_id, -1, mass=PIECE_MASS)

#b3
b3_position = [table_position[0] - 0.10, table_position[1] - 0.37, 0.7]
b3_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b3/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b3_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b3_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b3_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b3_id, -1, mass=PIECE_MASS)

#b4
b4_position = [table_position[0] + 0.155, table_position[1] - 0.37, 0.7]
b4_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b4/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b4_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b4_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b4_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b4_id, -1, mass=PIECE_MASS)

#b5
b5_position = [table_position[0] - 0.185, table_position[1] - 0.37, 0.7]
b5_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b5/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b5_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b5_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b5_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b5_id, -1, mass=PIECE_MASS)

#b6
b6_position = [table_position[0] + 0.242, table_position[1] - 0.37, 0.7]
b6_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b6/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b6_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b6_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b6_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b6_id, -1, mass=PIECE_MASS)

#b7
b7_position = [table_position[0] - 0.27, table_position[1] - 0.37, 0.7]
b7_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b7/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b7_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b7_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b7_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b7_id, -1, mass=PIECE_MASS)

#b8
b8_position = [table_position[0] + 0.327, table_position[1] - 0.37, 0.7]
b8_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b8/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b8_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b8_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b8_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b8_id, -1, mass=PIECE_MASS)

#b9
b9_position = [table_position[0] - 0.355, table_position[1] - 0.37, 0.7]
b9_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b9/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b9_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b9_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b9_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b9_id, -1, mass=PIECE_MASS)

#b10
b10_position = [table_position[0] + 0.242, table_position[1] - 0.2, 0.7]
b10_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b10/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b10_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b10_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b10_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b10_id, -1, mass=PIECE_MASS)

#b11
b11_position = [table_position[0] - 0.27, table_position[1] - 0.2, 0.7]
b11_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b11/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b11_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b11_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b11_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b11_id, -1, mass=PIECE_MASS)

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
pybullet.changeDynamics(b12_id, -1, mass=PIECE_MASS)

#b13
b13_position = [table_position[0] + 0.155, table_position[1] - 0.12, 0.7]
b13_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b13/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b13_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b13_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b13_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b13_id, -1, mass=PIECE_MASS)

#b14
b14_position = [table_position[0] - 0.015, table_position[1] - 0.12, 0.7]
b14_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b14/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b14_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b14_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b14_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b14_id, -1, mass=PIECE_MASS)

#b15
b15_position = [table_position[0] - 0.185, table_position[1] - 0.12, 0.7]
b15_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b15/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b15_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b15_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b15_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b15_id, -1, mass=PIECE_MASS)

#b16
b16_position = [table_position[0] - 0.355, table_position[1] - 0.12, 0.7]
b16_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/b16/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=b16_position,
                                  baseOrientation=b_orientation)
pybullet.changeVisualShape(b16_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(b16_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(b16_id, -1, mass=PIECE_MASS)
#############################

# RED PIECES

r_orientation = pybullet.getQuaternionFromEuler([0, 0, np.pi / 2])
cp_scaling = 0.20
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
pybullet.changeDynamics(r1_id, -1, mass=PIECE_MASS)

#r2
r2_position = [table_position[0] + 0.07, table_position[1] + 0.4, 0.7]
r2_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r2/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r2_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r2_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r2_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r2_id, -1, mass=PIECE_MASS)

#r3
r3_position = [table_position[0] - 0.10, table_position[1] + 0.4, 0.7]
r3_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r3/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r3_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r3_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r3_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r3_id, -1, mass=PIECE_MASS)

#r4
r4_position = [table_position[0] + 0.155, table_position[1] + 0.4, 0.7]
r4_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r4/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r4_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r4_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r4_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r4_id, -1, mass=PIECE_MASS)

#r5
r5_position = [table_position[0] - 0.185, table_position[1] + 0.4, 0.7]
r5_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r5/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r5_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r5_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r5_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r5_id, -1, mass=PIECE_MASS)

#r6
r6_position = [table_position[0] + 0.242, table_position[1] + 0.4, 0.7]
r6_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r6/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r6_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r6_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r6_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r6_id, -1, mass=PIECE_MASS)

#r7
r7_position = [table_position[0] - 0.27, table_position[1] + 0.4, 0.7]
r7_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r7/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r7_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r7_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r7_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r7_id, -1, mass=PIECE_MASS)

#r8
r8_position = [table_position[0] + 0.327, table_position[1] + 0.4, 0.7]
r8_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r8/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r8_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r8_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r8_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r8_id, -1, mass=PIECE_MASS)

#r9
r9_position = [table_position[0] - 0.355, table_position[1] + 0.4, 0.7]
r9_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r9/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r9_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r9_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r9_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r9_id, -1, mass=PIECE_MASS)

#r10
r10_position = [table_position[0] + 0.242, table_position[1] + 0.22, 0.7]
r10_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r10/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r10_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r10_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r10_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r10_id, -1, mass=PIECE_MASS)

#r11
r11_position = [table_position[0] - 0.27, table_position[1] + 0.22, 0.7]
r11_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r11/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r11_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r11_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r11_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r11_id, -1, mass=PIECE_MASS)

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
pybullet.changeDynamics(r12_id, -1, mass=PIECE_MASS)

#r13
r13_position = [table_position[0] + 0.155, table_position[1] + 0.14, 0.7]
r13_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r13/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r13_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r13_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r13_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r13_id, -1, mass=PIECE_MASS)

#r14
r14_position = [table_position[0] - 0.015, table_position[1] + 0.14, 0.7]
r14_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r14/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r14_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r14_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r14_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r14_id, -1, mass=PIECE_MASS)

#r15
r15_position = [table_position[0] - 0.185, table_position[1] + 0.14, 0.7]
r15_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r15/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r15_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r15_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r15_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r15_id, -1, mass=PIECE_MASS)

#r16
r16_position = [table_position[0] - 0.355, table_position[1] + 0.14, 0.7]
r16_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/r16/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=cp_scaling,
                                  basePosition=r16_position,
                                  baseOrientation=r_orientation)
pybullet.changeVisualShape(r16_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
pybullet.changeDynamics(r16_id, -1, lateralFriction=obj_friction_ceof)
pybullet.changeDynamics(r16_id, -1, mass=PIECE_MASS)
#############################

piece_id_to_char = defaultdict(lambda: "  ", {
    # board_id: "  ",

    b1_id:  "将",
    b2_id:  "士",
    b3_id:  "士",
    b4_id:  "象",
    b5_id:  "象",
    b6_id:  "馬",
    b7_id:  "馬",
    b8_id:  "車",
    b9_id:  "車",
    b10_id: "砲",
    b11_id: "砲",
    b12_id: "卒",
    b13_id: "卒",
    b14_id: "卒",
    b15_id: "卒",
    b16_id: "卒",

    r1_id:  "帥",
    r2_id:  "仕",
    r3_id:  "仕",
    r4_id:  "相",
    r5_id:  "相",
    r6_id:  "傌",
    r7_id:  "傌",
    r8_id:  "俥",
    r9_id:  "俥",
    r10_id: "炮",
    r11_id: "炮",
    r12_id: "兵",
    r13_id: "兵",
    r14_id: "兵",
    r15_id: "兵",
    r16_id: "兵",
})

piece_id_to_fen = defaultdict(lambda: ".", {
    # board_id: "  ",

    b1_id:  "k",
    b2_id:  "a",
    b3_id:  "a",
    b4_id:  "b",
    b5_id:  "b",
    b6_id:  "n",
    b7_id:  "n",
    b8_id:  "r",
    b9_id:  "r",
    b10_id: "c",
    b11_id: "c",
    b12_id: "p",
    b13_id: "p",
    b14_id: "p",
    b15_id: "p",
    b16_id: "p",

    r1_id:  "K",
    r2_id:  "A",
    r3_id:  "A",
    r4_id:  "B",
    r5_id:  "B",
    r6_id:  "N",
    r7_id:  "N",
    r8_id:  "R",
    r9_id:  "R",
    r10_id: "C",
    r11_id: "C",
    r12_id: "P",
    r13_id: "P",
    r14_id: "P",
    r15_id: "P",
    r16_id: "P",
})


################ Robot
# mobot_urdf_file = os.path.join(root_dir,"resource/urdf/robot/robot.urdf")
mobot_urdf_file = "resource/urdf/robot/robot.urdf"

obj_indices = [board_id]
mobot = Robot([0.22,0.7,0], obj_indices, piece_id_to_char, piece_id_to_fen, urdf_file=mobot_urdf_file)

# for j in range(pybullet.getNumJoints(mobot.robotId)):
#     print(pybullet.getJointInfo(mobot.robotId,j))

pybullet.changeVisualShape(mobot.robot_id,0,rgbaColor=[1,0,0,1])
pybullet.changeVisualShape(mobot.robot_id,1,rgbaColor=[0,1,0,1])

pybullet.setRealTimeSimulation(1)

for _ in range(30):
    pybullet.stepSimulation()

forward=0
speed=10
turn=0
up=0
stretch=0
gripper_open=0
roll=0
yaw=0


mobot.update_observations()

total_driving_distance = 0
previous_position, _, _ = mobot.get_robot_base_pose(mobot.robot_id)
current_position = previous_position

constraint = None

navi_flag = False
grasp_flag = False

while True:
    time.sleep(1/240)
    speed = 20
    
    for keycode, keystate in pybullet.getKeyboardEvents().items():
        # moving - only left (robot moving forward) and right (robot moving backward) arrows are used
        if (keycode == pybullet.B3G_RIGHT_ARROW and (keystate & pybullet.KEY_WAS_TRIGGERED)):
            turn = -1
        if (keycode == pybullet.B3G_RIGHT_ARROW and (keystate & pybullet.KEY_WAS_RELEASED)):
            turn = 0
        if (keycode == pybullet.B3G_LEFT_ARROW and (keystate & pybullet.KEY_WAS_TRIGGERED)):
            turn = 1
        if (keycode == pybullet.B3G_LEFT_ARROW and (keystate & pybullet.KEY_WAS_RELEASED)):
            turn = 0
        if (keycode == pybullet.B3G_UP_ARROW and (keystate & pybullet.KEY_WAS_TRIGGERED)):
            forward=100
        if (keycode == pybullet.B3G_UP_ARROW and (keystate & pybullet.KEY_WAS_RELEASED)):
            forward=0
        if (keycode == pybullet.B3G_DOWN_ARROW and (keystate & pybullet.KEY_WAS_TRIGGERED)):
            forward=-100
        if (keycode == pybullet.B3G_DOWN_ARROW and (keystate & pybullet.KEY_WAS_RELEASED)):
            forward=0

        # lifting
        if (keycode == ord('u') and (keystate & pybullet.KEY_WAS_TRIGGERED)):
            up = 1
        if (keycode == ord('u') and (keystate & pybullet.KEY_WAS_RELEASED)):
            up = 0
        if (keycode == ord('d') and (keystate & pybullet.KEY_WAS_TRIGGERED)):
            up = -1
        if (keycode == ord('d') and (keystate & pybullet.KEY_WAS_RELEASED)):
            up = 0

        # stretching -- forward/backward
        if (keycode == ord('b') and (keystate & pybullet.KEY_WAS_TRIGGERED)):
            stretch = -1
        if (keycode == ord('b') and (keystate & pybullet.KEY_WAS_RELEASED)):
            stretch = 0
        if (keycode == ord('f') and (keystate & pybullet.KEY_WAS_TRIGGERED)):
            stretch = 1
        if (keycode == ord('f') and (keystate & pybullet.KEY_WAS_RELEASED)):
            stretch = 0

        # gripper
        # open gripper when 'o' is pressed
        if (keycode == ord('o') and (keystate & pybullet.KEY_WAS_TRIGGERED)):
            gripper_control(mobot, pybullet, cmd=1)
        # if (keycode == ord('o') and (keystate & pybullet.KEY_WAS_RELEASED)):
        #     gripper_open = False
        # close gripper when 'c' is pressed
        if (keycode == ord('c') and (keystate & pybullet.KEY_WAS_TRIGGERED)):
           gripper_control(mobot, pybullet, cmd=0)
        # if (keycode == ord('c') and (keystate & pybullet.KEY_WAS_RELEASED)):
        #     gripper_open = True

    base_control(mobot, pybullet, forward, turn)
    arm_control(mobot, pybullet, up, stretch, roll, yaw)

    mobot.update_observations()

    current_position, _, _ = mobot.get_robot_base_pose(mobot.robot_id)
    total_driving_distance += np.linalg.norm(np.array(current_position) - np.array(previous_position))
    previous_position = current_position

    # mobot.update_observations()
    # mobot.make_move()

    ee_position, _, _ = mobot.get_robot_ee_pose(mobot.robot_id)
    #print("End-effector position: ", ee_position)
