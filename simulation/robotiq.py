import asyncio
import os
import threading
import time
from collections import defaultdict

import cv2
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pybullet
import pybullet_data
from PIL import Image

from engine.pikafish import Pikafish

BOUNDS = np.float32([[-0.3, 0.3], [-0.8, -0.2], [0, 0.15]])
PIXEL_SIZE = 0.00267857
urdf_dir = "resource/urdf"

class Robotiq2F85:
    """Gripper handling for Robotiq 2F85."""

    def __init__(self, robot, tool, position):
        self.robot = robot
        self.tool = tool
        pos = [position[0] + 0.1339999999999999, position[1] - 0.49199999999872496, position[2] + 0.5]
        rot = pybullet.getQuaternionFromEuler([np.pi, 0, np.pi])
        urdf = 'resource/urdf/robotiq_2f_85/robotiq_2f_85.urdf'
        self.body_id = pybullet.loadURDF(urdf, pos, rot)
        self.n_joints = pybullet.getNumJoints(self.body_id)
        self.activated = False

        # Connect gripper base to robot tool.
        pybullet.createConstraint(self.robot, tool, self.body_id, 0, jointType=pybullet.JOINT_FIXED, jointAxis=[0, 0, 0], parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, -0.07], childFrameOrientation=pybullet.getQuaternionFromEuler([0, 0, np.pi / 2]))

        # Set friction coefficients for gripper fingers.
        for i in range(pybullet.getNumJoints(self.body_id)):
            pybullet.changeDynamics(self.body_id, i, lateralFriction=10.0, spinningFriction=1.0, rollingFriction=1.0, frictionAnchor=True)

        self.motor_joint = 1
        self.camera_id = 2

        # Start thread to handle additional gripper constraints.
        self.constraints_thread = threading.Thread(target=self.gripper_step)
        self.constraints_thread.daemon = True
        self.constraints_thread.start()

    def gripper_step(self):
        """
        Control joint positions by enforcing hard contraints on gripper behavior.
        Set one joint as the open/close motor joint (other joints should mimic).
        """
        while True:
            try:
                currj = [pybullet.getJointState(self.body_id, i)[0] for i in range(self.n_joints)]
                indj = [6, 3, 8, 5, 10]
                targj = [currj[1], -currj[1], -currj[1], currj[1], currj[1]]
                pybullet.setJointMotorControlArray(self.body_id, indj, pybullet.POSITION_CONTROL, targj, positionGains=np.ones(5))
            except:
                return
            time.sleep(0.01)

    def activate(self):
        """Close gripper fingers"""
        pybullet.setJointMotorControl2(self.body_id, self.motor_joint, pybullet.VELOCITY_CONTROL, targetVelocity=1.5, force=50)
        self.activated = True

    def release(self):
        """Open gripper fingers"""
        pybullet.setJointMotorControl2(self.body_id, self.motor_joint, pybullet.VELOCITY_CONTROL, targetVelocity=-0.8, force=50)
        self.activated = False

    def detect_contact(self):
        """
        If activated and object in gripper: check object contact.
        If activated and nothing in gripper: check gripper contact.
        If released: check proximity to surface (disabled).
        """
        obj, _, ray_frac = self.check_proximity()
        if self.activated:
            empty = self.grasp_width() < 0.01
            cbody = self.body_id if empty else obj
            if obj == self.body_id or obj == 0:
                return False
            return self.external_contact(cbody)

    def external_contact(self, body=None):
        """Returns true if body is in contact with something other than gripper"""
        if body is None:
            body = self.body_id
        pts = pybullet.getContactPoints(bodyA=body)
        pts = [pt for pt in pts if pt[2] != self.body_id]
        return len(pts) > 0  # pylint: disable=g-explicit-length-test

    def is_grasp_successful(self):
        return self.grasp_width() <= 0.063

    def grasp_width(self):
        lpad = np.array(pybullet.getLinkState(self.body_id, 4)[0])
        rpad = np.array(pybullet.getLinkState(self.body_id, 9)[0])
        dist = np.linalg.norm(lpad - rpad) - 0.047813
        return dist

    def check_proximity(self):
        ee_pos = np.array(pybullet.getLinkState(self.robot, self.tool)[0])
        tool_pos = np.array(pybullet.getLinkState(self.body_id, 0)[0])
        vec = (tool_pos - ee_pos) / np.linalg.norm((tool_pos - ee_pos))
        ee_targ = ee_pos + vec
        ray_data = pybullet.rayTest(ee_pos, ee_targ)[0]
        obj, link, ray_frac = ray_data[0], ray_data[1], ray_data[2]
        return obj, link, ray_frac


class SimulationEnv:
    def __init__(self):
        self.dt = 1/480

        assets_path = os.path.dirname(os.path.abspath(""))
        pybullet.setAdditionalSearchPath(assets_path)
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        pybullet.setTimeStep(self.dt)

        self.home_joints = (np.pi / 2, -np.pi / 2, np.pi / 2, -np.pi / 2, 3 * np.pi / 2, 0)  # Joint angles: (J0, J1, J2, J3, J4, J5).
        self.home_ee_euler = (np.pi, 0, np.pi)  # (RX, RY, RZ) rotation in Euler angles.
        self.ee_link_id = 9  # Link ID of UR5 end effector
        self.tip_link_id = 10  # Link ID of gripper finger tips
        self.gripper = None

        self.table_position = [0, 0, 0]
        self.robot_position = [0.0, 0.64, 0.6]
        self.box_position = [0.5, 0.0, 0.67]
        self.box_drop_position = [0.5, 0.0, 0.8]
        self.board_position = [0.09, -0.05, 0.6]

        image_aspect_ratio = 1.0
        self.image_height = 320
        self.image_width = int(image_aspect_ratio * self.image_height)
        self.board_id = None
        self.piece_id_to_char = None
        self.piece_id_to_fen = None
        self.board = None

        self.engine = Pikafish()

        # Default position of robot arm's end effector. to get out of the camera's view
        self.default_position = [0, 0.0, 0.8]
        self.board_positions = [[0.0, 0.0, 0.0] * 9 for _ in range(10)]

    def reset(self):
        urdf_dir = "resource/urdf"

        ################ Plane
        plane_id = pybullet.loadURDF(os.path.join(urdf_dir,"/plane.urdf"), [0, 0, 0])
        plane_texture_id = pybullet.loadTexture("resource/texture/texture1.jpg")
        pybullet.changeVisualShape(0, -1, textureUniqueId=plane_texture_id)

        ################ Table
        table_scaling = 1.0
        table_orientation = pybullet.getQuaternionFromEuler([0, 0, np.pi/2])
        table_id = pybullet.loadURDF(
            fileName=os.path.join(urdf_dir,"table.urdf"),
            useFixedBase=True,
            basePosition=self.table_position,
            baseOrientation=table_orientation,
            globalScaling=table_scaling
        )
        # table_texture_id = pybullet.loadTexture(os.path.join(root_dir,"resource/texture/table.png"))
        table_texture_id = pybullet.loadTexture("resource/texture/table.png")
        pybullet.changeVisualShape(table_id,0,textureUniqueId=table_texture_id)
        
        ################ Robot
        self.robot_id = pybullet.loadURDF(
            "resource/urdf/ur5e/ur5e.urdf",
            self.robot_position,
            globalScaling=1.2,
            flags=pybullet.URDF_USE_MATERIAL_COLORS_FROM_MTL
        )
        self.joint_ids = [pybullet.getJointInfo(self.robot_id, i) for i in range(pybullet.getNumJoints(self.robot_id))]
        self.joint_ids = [j[0] for j in self.joint_ids if j[2] == pybullet.JOINT_REVOLUTE]

        # Move robot to home configuration
        for i in range(len(self.joint_ids)):
            pybullet.resetJointState(self.robot_id, self.joint_ids[i], self.home_joints[i])

        # Add gripper
        if self.gripper is not None:
            while self.gripper.constraints_thread.is_alive():
                self.constraints_thread_active = False
        self.gripper = Robotiq2F85(self.robot_id, self.ee_link_id, self.robot_position)
        self.gripper.release()

        ################ Box
        box_scaling = 0.08
        box_orientation = pybullet.getQuaternionFromEuler([0, 0, np.pi])
        box_id = pybullet.loadURDF(
            fileName=os.path.join(urdf_dir,"obj_libs/box/box.urdf"),
            useFixedBase=True,
            basePosition=self.box_position,
            baseOrientation=box_orientation,
            globalScaling=box_scaling
        )
        # change colour to light brown
        pybullet.changeVisualShape(box_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
        pybullet.changeDynamics(box_id, -1, mass=0.2)

        ################ Board
        board_scaling = 0.5
        board_orientation = pybullet.getQuaternionFromEuler([0, 0, np.pi])
        board_id = pybullet.loadURDF(
            fileName=os.path.join(urdf_dir,"obj_libs/chessboard/chessboard.urdf"),
            useFixedBase=True,
            basePosition=self.board_position,
            baseOrientation=board_orientation,
            globalScaling=board_scaling
        )
        self.board_id = board_id
        pybullet.changeVisualShape(board_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
        # pybullet.changeDynamics(board_id, -1, mass=1.0)


        ################ Chess Pieces
        self.piece_id_to_char, self.piece_id_to_fen = self.initialize_chess_pieces(self.board_position)

        # Go back to default position
        self.move_and_step(self.default_position)

        # Add collision filter to avoid gripper-board interaction
        pybullet.setCollisionFilterPair(
            self.gripper.body_id,  # Gripper ID
            board_id,           # board ID
            linkIndexA=-1,      # Entire gripper
            linkIndexB=-1,      # Entire table
            enableCollision=False
        )

        # for _ in range(30):
        #     self.step_sim_and_update_obs()

    def move_joints_to_pos(self, joint_positions):
        pybullet.setJointMotorControlArray(
            bodyIndex=self.robot_id,
            jointIndices=self.joint_ids,
            controlMode=pybullet.POSITION_CONTROL,
            targetPositions=joint_positions,
            positionGains=[0.01]*6
        )

    def movep(self, position, multiplier=64.0):
        joints = pybullet.calculateInverseKinematics(
            bodyUniqueId=self.robot_id,
            endEffectorLinkIndex=self.tip_link_id,
            targetPosition=position,
            targetOrientation=pybullet.getQuaternionFromEuler(self.home_ee_euler),
            maxNumIterations=100
        )
        self.move_joints_to_pos(joints)
        self.step_sim_and_update_obs()
        # position = np.array(position)
        # current_position = self.get_end_effector_pos()
        # dist = np.sqrt(np.sum((current_position - position) ** 2.0))
        # steps = max(1, round(dist * multiplier))  # Increase steps for smoother and slower movement
        # # print(f"Dist: {dist:.3f}, Steps: {steps}")
        # for i in range(1, steps + 1):
        #     interpolated_position = current_position + (position - current_position) * (i / steps)
        #     joints = pybullet.calculateInverseKinematics(
        #         bodyUniqueId=self.robot_id,
        #         endEffectorLinkIndex=self.tip_link_id,
        #         targetPosition=interpolated_position,
        #         targetOrientation=pybullet.getQuaternionFromEuler(self.home_ee_euler),
        #         maxNumIterations=100
        #     )
        #     self.move_joints_to_pos(joints)
        #     self.step_sim_and_update_obs()

    def get_end_effector_pos(self):
        return np.array(pybullet.getLinkState(self.robot_id, self.tip_link_id)[0])

    def move_and_step(self, goal, target_delta = 0.01):
        ee_xyz = self.get_end_effector_pos()
        while np.linalg.norm(goal - ee_xyz) > target_delta:
            self.movep(goal)
            self.step_sim_and_update_obs()
            ee_xyz = self.get_end_effector_pos()

    def move_object(self, start_xyz, end_xyz):
        start_xyz = np.array(start_xyz)
        end_xyz = np.array(end_xyz)

        # Compute hover xyz, which is slightly above the source and target coords
        hover_height = 0.1
        hover_start_xyz = start_xyz.copy()
        hover_end_xyz = end_xyz.copy()
        hover_start_xyz[2] += hover_height
        hover_end_xyz[2] += hover_height

        self.move_and_step(hover_start_xyz)
        self.move_and_step(start_xyz)
        self.gripper.activate()
        prev_grasp_width = self.gripper.grasp_width()
        time.sleep(0.01)
        while prev_grasp_width > self.gripper.grasp_width():
            prev_grasp_width = self.gripper.grasp_width()
            time.sleep(0.01)
        self.move_and_step(hover_start_xyz)
        self.move_and_step(hover_end_xyz)
        self.move_and_step(end_xyz)
        self.gripper.release()
        prev_grasp_width = self.gripper.grasp_width()
        time.sleep(0.01)
        while prev_grasp_width < self.gripper.grasp_width():
            prev_grasp_width = self.gripper.grasp_width()
            time.sleep(0.01)

        self.move_and_step(hover_end_xyz)
        self.move_and_step(self.default_position)

    def step_sim_and_update_obs(self):
        pybullet.stepSimulation()
        self.update_counter = getattr(self, 'update_counter', 0) + 1
        if self.update_counter % 16 == 0:
            self.update_observations(fast=True)
            self.update_counter = 0

    def update_observations(self, fast=False) -> None:
        # For simplicity, let's fix the camera position to directly above the board
        # camera_view_matrix = pybullet.computeViewMatrix(
        #     cameraEyePosition=[0, -0.1, 1.8],
        #     cameraTargetPosition=[0.0, 0.0, 0.0],
        #     cameraUpVector=[0.0, 0.0, 1.0]
        # )
        camera_position = np.array(pybullet.getLinkState(self.gripper.body_id, self.gripper.camera_id)[4])
        target_position = camera_position.copy()
        target_position[0] += 0.00001 # Look directly below
        target_position[2] -= 1.0 # Look directly below
        camera_view_matrix = pybullet.computeViewMatrix(
            cameraEyePosition=camera_position,
            cameraTargetPosition=target_position, # [0, 0, 0]
            cameraUpVector=[0.0, 0.0, 1.0]
        )

        # pybullet.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, camera_target_position)
        camera_proj_matrix = pybullet.computeProjectionMatrixFOV(fov=125.0, aspect=1.0, nearVal=0.1, farVal=10)
        # $initAxis(camera_link_pos, camera_link_ori)
        cameraImage = pybullet.getCameraImage(
            width=self.image_width,
            height=self.image_height,
            viewMatrix=camera_view_matrix,
            projectionMatrix=camera_proj_matrix,
            renderer=pybullet.ER_BULLET_HARDWARE_OPENGL
        )

        if fast:
            return

        width, height, rgbPixels, depthPixels, segmentationMaskBuffer = cameraImage

        # plt.imsave('11.png', segmentationMaskBuffer)
        segmentationMaskBuffer = np.rot90(segmentationMaskBuffer, axes=(1, 0))
        indices = np.argwhere(segmentationMaskBuffer == self.board_id)
        src_board_coords = self.get_board_coordinates(indices)
        dst_board_coords = np.array([
            [0, 0],
            [0, self.image_width-1],
            [self.image_height-1, 0],
            [self.image_height-1, self.image_width-1]
        ], dtype=np.float32) # y,x format
        transform_matrix = cv2.getPerspectiveTransform(src_board_coords[:, ::-1], dst_board_coords[:, ::-1]) # cast to x,y format
        dst_size = (self.image_width, self.image_height)
        transformed_board = cv2.warpPerspective(
            segmentationMaskBuffer.astype(np.float64),
            transform_matrix,
            dst_size,
            flags=cv2.INTER_NEAREST # Keep all pixels integers
        )
        # plt.imsave('22.png', transformed_board)

        x_vals = np.linspace(width*5/320, width*314/320, num=10)
        y_vals = np.linspace(height*7/320, height*312/320, num=11)
        x_coords = (x_vals[:-1] + x_vals[1:]) / 2
        y_coords = (y_vals[:-1] + y_vals[1:]) / 2
        x_coords = x_coords.round().astype(int)
        y_coords = y_coords.round().astype(int)

        # plt.imsave('33.png', transformed_board)

        # maxi = transformed_board.max() + 1
        # WHITE = (maxi, maxi, maxi)
        # for x in x_vals:
        #     x = round(x)
        #     cv2.line(transformed_board, (x, 0), (x, height), WHITE, 1)
        # for y in y_vals:
        #     y = round(y)
        #     cv2.line(transformed_board, (0, y), (width, y), WHITE, 1)
        # for x in x_coords:
        #     x = round(x)
        #     for y in y_coords:
        #         y = round(y)
        #         cv2.circle(transformed_board, (x,y), radius=0, color=WHITE, thickness=-1)
        # plt.imsave('44.png', transformed_board)
        # exit()

        board = transformed_board[np.ix_(y_coords, x_coords)]
        self.board = board.astype(int)
        
    def get_piece_at_position(self, position):
        # Get the ID of the piece at a specific chessboard position.
        # returns The segmentation mask ID of the piece, or None if empty.
        row, col = self.pos_to_idx(position)
        piece_id = self.board[9 - row, col]

        if piece_id == self.board_id or piece_id == 0:
            return None  # No piece at this position

        return piece_id  # Return the piece ID
    
    def get_board_coordinates(self, indices: np.ndarray):
        rect = np.zeros((4, 2), dtype=np.float32)
        s = indices.sum(axis=1)
        diff = np.diff(indices, axis=1)
        rect[0] = indices[np.argmin(s)] # top left
        rect[1] = indices[np.argmax(diff)] # top right
        rect[2] = indices[np.argmin(diff)] # bottom left
        rect[3] = indices[np.argmax(s)] # bottom right
        return rect
    
    def print_board(self):
        board = np.vectorize(self.piece_id_to_char.__getitem__)(self.board)
        board = np.array(['|'.join(row) for row in board])
        print()
        print('-' * 28)
        for row in board:
            print('|' + row + '|')
            print('-' * 28)
        print()

    def board_to_fen(self, board, turn):
        # Initialize the FEN string
        fen_rows = []

        # Iterate over each row in the board matrix
        for row in board:
            fen_row = []
            empty_count = 0

            for piece in row:
                fen_piece = self.piece_id_to_fen[piece]
                if fen_piece == '.':
                    empty_count += 1
                else:
                    if empty_count > 0:
                        fen_row.append(str(empty_count))
                        empty_count = 0
                    fen_row.append(fen_piece)

            # If there were empty spaces at the end of the row
            if empty_count > 0:
                fen_row.append(str(empty_count))

            fen_rows.append(''.join(fen_row))

        # Join the rows with '/' to create the full FEN string
        fen = '/'.join(fen_rows)
        fen_turn = "w" if turn else "b"
        fen += f" {fen_turn} - - 0 1" # TODO: implement halfmoves and fullmoves
        return fen

    def make_move(self, is_red_turn=True, move=None, print_evals=False) -> bool:
        self.update_observations()
        if self.board is None or self.board.shape != (10, 9):
            print("Camera image cannot be processed.")
            return

        if move is None:
            # if self.i == 0:
            #     self.print_board()
            # self.i = (self.i+1) % 30

            fen = self.board_to_fen(self.board, is_red_turn)
            # print(fen)
            move = asyncio.run(self.engine.get_best_move(fen, print_evals=print_evals))
            print("Pikafish thinks the best move is", move, flush=True)
            if move == '(none)':
                # Game over
                print("Game over!")

        # get xyz coordinates of targets
        start_pos = move[:2]
        end_pos = move[2:]

        if self.get_piece_at_position(start_pos) is None:
            # Invalid move
            return False

        # print(f"row: {row}, col: {col}")
        # return self.board_positions[row][col]

        piece_grip_depth = 0.048 # 0.0452
        piece_drop_depth = 0.037
        # check if there is a piece at the location
        if self.get_piece_at_position(end_pos) is not None:
            # print(self.get_piece_at_position(end_pos))
            # move the piece into the box
            r0, c0 = self.pos_to_idx(end_pos)
            start_id = self.board[9-r0, c0]
            start_coords = list(pybullet.getBasePositionAndOrientation(start_id)[0])
            start_coords[2] -= piece_grip_depth
            self.move_object(start_coords, self.box_drop_position)

        r0, c0 = self.pos_to_idx(start_pos)
        start_id = self.board[9-r0, c0]

        start_coords = list(pybullet.getBasePositionAndOrientation(start_id)[0])
        end_coords = self.pos_to_coordinates(end_pos)

        start_coords[2] -= piece_grip_depth
        end_coords[2] -= piece_drop_depth
        self.move_object(start_coords, end_coords)
        return True

    def pos_to_idx(self, pos):
        col = ord(pos[0]) - ord('a')
        row = int(pos[1])
        return row, col

    def pos_to_coordinates(self, pos):
        # pos is a string like "a1"
        # returns the coordinates in the board_positions matrix
        # a1 = [0,8]
        # h10 = [9,0]
        row, col = self.pos_to_idx(pos)
        return self.board_positions[row][col].copy()

    def initialize_chess_pieces(self, board_center_pos):
        # all chess pieces initial position
        # chinese chess
        chess_pieces = {    
            "b1" : "e9", # black king
            "b2" : "d9", "b3" : "f9", # black advisor
            "b4" : "c9", "b5" : "g9", # black elephant
            "b6" : "b9", "b7" : "h9", # black horse
            "b8" : "a9", "b9" : "i9", # black chariot
            "b10" : "b7", "b11" : "h7", # black cannon
            "b12" : "a6", "b13" : "c6", "b14" : "e6", "b15" : "g6", "b16" : "i6", # black soldier

            "r1" : "e0", # red king
            "r2" : "d0", "r3" : "f0", # red advisor
            "r4" : "c0", "r5" : "g0", # red elephant
            "r6" : "b0", "r7" : "h0", # red horse
            "r8" : "a0", "r9" : "i0", # red chariot
            "r10" : "b2", "r11" : "h2", # red cannon
            "r12" : "a3", "r13" : "c3", "r14" : "e3", "r15" : "g3", "r16" : "i3" # red soldier
        }
        # Dynamically create variables and store their IDs in a dictionary
        piece_id_map = {}

        gap = 0.086 # chess spacing apart
        chess_bottom_left_corner = [board_center_pos[0]- 5 * gap - 0.015, board_center_pos[1] - 4 * gap + 0.016, board_center_pos[2] + 0.066]
        for row in range(10):
            for col in range(9):
                self.board_positions[row][col] = [
                    chess_bottom_left_corner[0] + col * gap,  # X position
                    chess_bottom_left_corner[1] + row * gap,  # Y position (invert rows for bottom-left start)
                    chess_bottom_left_corner[2]  # Z position (same for all)
                ]
                
        # ################ CHESS PIECES

        PIECE_MASS = 0.01
        cp_scaling = 0.20 # chess piece scaling
        obj_friction_ceof = 100.0
        b_orientation = pybullet.getQuaternionFromEuler([0, 0, np.pi / 2]) # black pieces orientation
        r_orientation = pybullet.getQuaternionFromEuler([0, 0, -np.pi / 2]) # red pieces orientation

        # Red pieces
        red = ["r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15", "r16"]
        for r in red:
            base_position = self.pos_to_coordinates(chess_pieces[r])
            piece_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/"+r+"/model.urdf"),
                                            useFixedBase=False,
                                            globalScaling=cp_scaling,
                                            basePosition=base_position,
                                            baseOrientation=r_orientation)
            pybullet.changeVisualShape(piece_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
            pybullet.changeDynamics(
                piece_id,  # ID of the chess piece
                -1,        # For the entire object
                lateralFriction=obj_friction_ceof,  # Increased friction
                spinningFriction=1.0,  # Improved rotation stability
                    mass=PIECE_MASS  # Light mass for chess piece
            )
            # Store IDs in a dictionary for easier access
            piece_id_map[r] = piece_id

        # Black pieces
        black = ["b1", "b2", "b3", "b4", "b5", "b6", "b7", "b8", "b9", "b10", "b11", "b12", "b13", "b14", "b15", "b16"]
        for b in black:
            piece_position = self.pos_to_coordinates(chess_pieces[b])
            piece_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/"+b+"/model.urdf"),
                                            useFixedBase=False,
                                            globalScaling=cp_scaling,
                                            basePosition=piece_position,
                                            baseOrientation=b_orientation)
            pybullet.changeVisualShape(piece_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
            pybullet.changeDynamics(
                piece_id,  # ID of the chess piece
                -1,        # For the entire object
                lateralFriction=obj_friction_ceof,  # Increased friction
                spinningFriction=1.0,  # Improved rotation stability
                    mass=PIECE_MASS  # Light mass for chess piece
                )
            # Store IDs in a dictionary for easier access
            piece_id_map[b] = piece_id

        print("Chess pieces initialized")
        
        piece_id_to_char = defaultdict(lambda: "  ", {
            # board_id: "  ",

            piece_id_map["b1"]:  "将",
            piece_id_map["b2"]:  "士",
            piece_id_map["b3"]:  "士",
            piece_id_map["b4"]:  "象",
            piece_id_map["b5"]:  "象",
            piece_id_map["b6"]:  "馬",
            piece_id_map["b7"]:  "馬",
            piece_id_map["b8"]:  "車",
            piece_id_map["b9"]:  "車",
            piece_id_map["b10"]: "砲",
            piece_id_map["b11"]: "砲",
            piece_id_map["b12"]: "卒",
            piece_id_map["b13"]: "卒",
            piece_id_map["b14"]: "卒",
            piece_id_map["b15"]: "卒",
            piece_id_map["b16"]: "卒",

            piece_id_map["r1"]:  "帥",
            piece_id_map["r2"]:  "仕",
            piece_id_map["r3"]:  "仕",
            piece_id_map["r4"]:  "相",
            piece_id_map["r5"]:  "相",
            piece_id_map["r6"]:  "傌",
            piece_id_map["r7"]:  "傌",
            piece_id_map["r8"]:  "俥",
            piece_id_map["r9"]:  "俥",
            piece_id_map["r10"]: "炮",
            piece_id_map["r11"]: "炮",
            piece_id_map["r12"]: "兵",
            piece_id_map["r13"]: "兵",
            piece_id_map["r14"]: "兵",
            piece_id_map["r15"]: "兵",
            piece_id_map["r16"]: "兵",
        })

        piece_id_to_fen = defaultdict(lambda: ".", {
            # board_id: "  ",

            piece_id_map["b1"]:  "k",
            piece_id_map["b2"]:  "a",
            piece_id_map["b3"]:  "a",
            piece_id_map["b4"]:  "b",
            piece_id_map["b5"]:  "b",
            piece_id_map["b6"]:  "n",
            piece_id_map["b7"]:  "n",
            piece_id_map["b8"]:  "r",
            piece_id_map["b9"]:  "r",
            piece_id_map["b10"]: "c",
            piece_id_map["b11"]: "c",
            piece_id_map["b12"]: "p",
            piece_id_map["b13"]: "p",
            piece_id_map["b14"]: "p",
            piece_id_map["b15"]: "p",
            piece_id_map["b16"]: "p",
            
            piece_id_map["r1"]:  "K",
            piece_id_map["r2"]:  "A",
            piece_id_map["r3"]:  "A",
            piece_id_map["r4"]:  "B",
            piece_id_map["r5"]:  "B",
            piece_id_map["r6"]:  "N",
            piece_id_map["r7"]:  "N",
            piece_id_map["r8"]:  "R",
            piece_id_map["r9"]:  "R",
            piece_id_map["r10"]: "C",
            piece_id_map["r11"]: "C",
            piece_id_map["r12"]: "P",
            piece_id_map["r13"]: "P",
            piece_id_map["r14"]: "P",
            piece_id_map["r15"]: "P",
            piece_id_map["r16"]: "P",        
        })
        
        return piece_id_to_char, piece_id_to_fen
