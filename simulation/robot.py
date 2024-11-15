import asyncio

import cv2
import numpy as np
import pybullet

from engine.pikafish import Pikafish


def initAxis(center, quater):
    rot_mat = pybullet.getMatrixFromQuaternion(quater)
    rotmat = np.array(rot_mat).reshape((3, 3))
    pybullet.addUserDebugLine(lineFromXYZ=center, lineToXYZ=center + rotmat[:3, 0] * 0.1, lineColorRGB=[1, 0, 0],
                            lineWidth=10)
    pybullet.addUserDebugLine(lineFromXYZ=center, lineToXYZ=center + rotmat[:3, 1] * 0.1, lineColorRGB=[0, 1, 0],
                            lineWidth=10)
    pybullet.addUserDebugLine(lineFromXYZ=center, lineToXYZ=center + rotmat[:3, 2] * 0.1, lineColorRGB=[0, 0, 1],
                            lineWidth=10)

def get_global_action_from_local(robot, delta_forward):
    # Get the current joint angle of joint 2 (rotation around z-axis)
    joint2_state = pybullet.getJointState(robot, 3)
    current_yaw = joint2_state[0]  # Get the current rotation (yaw angle)
    
    # Calculate the delta in world coordinates using the yaw angle (rotation around z-axis)
    delta_x = delta_forward * np.cos(current_yaw)  # Change along world x-axis
    delta_y = delta_forward * np.sin(current_yaw)  # Change along world y-axis
    
    return delta_x, delta_y

def base_control(robot, p, forward=0, turn=0):
    x_forward, y_forward = get_global_action_from_local(robot.robot_id, forward)
    p.setJointMotorControl2(robot.robot_id,3,p.VELOCITY_CONTROL,targetVelocity=turn,force=1000)
    p.setJointMotorControl2(robot.robot_id,1,p.VELOCITY_CONTROL,targetVelocity=x_forward,force=1000)
    p.setJointMotorControl2(robot.robot_id,2,p.VELOCITY_CONTROL,targetVelocity=y_forward,force=1000)

def arm_control(robot, p, up=0, stretch=0, roll=0, yaw=0):
    # up and down
    p.setJointMotorControl2(robot.robot_id,8,p.VELOCITY_CONTROL,targetVelocity=0.2*up,force=1000)

    # stretch and shrink
    p.setJointMotorControl2(robot.robot_id,10,p.VELOCITY_CONTROL,targetVelocity=0.1*stretch,force=100)
    p.setJointMotorControl2(robot.robot_id,11,p.VELOCITY_CONTROL,targetVelocity=0.1*stretch,force=100)
    p.setJointMotorControl2(robot.robot_id,12,p.VELOCITY_CONTROL,targetVelocity=0.1*stretch,force=100)
    p.setJointMotorControl2(robot.robot_id,13,p.VELOCITY_CONTROL,targetVelocity=0.1*stretch,force=100)
    
    # rotate
    p.setJointMotorControl2(robot.robot_id,14,p.VELOCITY_CONTROL,targetVelocity=roll,force=1000)
    p.setJointMotorControl2(robot.robot_id,16,p.VELOCITY_CONTROL,targetVelocity=yaw,force=1000)



def gripper_control(mobot, p, cmd=0):
    # Configuration for centered and gentle grip
    open_position = 0.015  # Slightly open for holding
    close_position = 0.001  # Close to center without overlap
    symmetric_force = 1.5  # Gentle force to prevent sliding

    if cmd == 1:  # Open slightly
        p.setJointMotorControl2(mobot.robot_id, 18, p.POSITION_CONTROL,
                                targetPosition=open_position,
                                force=symmetric_force)

        p.setJointMotorControl2(mobot.robot_id, 19, p.POSITION_CONTROL,
                                targetPosition=-open_position,
                                force=symmetric_force)

    elif cmd == 0:  # Close gently around the chess piece
        # Close fingers to a gentle contact without pushing inward
        p.setJointMotorControl2(mobot.robot_id, 18, p.POSITION_CONTROL,
                                targetPosition=-close_position,
                                force=symmetric_force)

        p.setJointMotorControl2(mobot.robot_id, 19, p.POSITION_CONTROL,
                                targetPosition=close_position,
                                force=symmetric_force)

    # Optionally, after closing, switch to torque control to hold without squeezing further
    p.setJointMotorControl2(mobot.robot_id, 18, p.TORQUE_CONTROL, force=0.1)
    p.setJointMotorControl2(mobot.robot_id, 19, p.TORQUE_CONTROL, force=0.1)


class Robot:
    def __init__(self, start_pos, obj_indices, piece_id_to_char, piece_id_to_fen, urdf_file=None):
        self.gripperMaxForce = 1000.0
        self.armMaxForce = 200.0

        self.obj_indices = obj_indices
        self.board_id = obj_indices[0]
        self.piece_id_to_char = piece_id_to_char
        self.piece_id_to_fen = piece_id_to_fen
        self.urdf_file = urdf_file

        self.robot_id = pybullet.loadURDF(urdf_file, start_pos, globalScaling=1.2)

        # Create a mapping from joint/link name to id
        self.joint_to_int = {
            pybullet.getBodyInfo(self.robot_id)[0].decode('UTF-8') : -1 # base link doesn't have an id, just set -1
        }
        for idx in range(pybullet.getNumJoints(self.robot_id)):
            joint_info = pybullet.getJointInfo(self.robot_id, idx)
            joint_name = joint_info[1].decode('UTF-8')
            link_name = joint_info[12].decode('UTF-8')
            self.joint_to_int[joint_name] = idx
            self.joint_to_int[link_name] = idx
        
        self.camera_idx = self.get_joint_index("link_head_tilt")

        # Set initial positions and orientations
        pybullet.resetBasePositionAndOrientation(self.robot_id, start_pos, [0, 0, 0, 1])
        pybullet.resetJointState(self.robot_id, self.camera_idx, -0.3)
        pybullet.resetJointState(self.robot_id, 4, 0.5)

        image_aspect_ratio = 1.0
        self.image_height = 320
        self.image_width = int(image_aspect_ratio * self.image_height)
        # self.board_image = None # shape is (height, width, 4), RGBA format
        # self.board_seg_mask = None
        self.board = None
        self.i = 0

        # Set friction coefficients for gripper fingers.
        for i in range(pybullet.getNumJoints(self.robot_id)):
            pybullet.changeDynamics(self.robot_id, i, lateralFriction=10.0, spinningFriction=1.0, rollingFriction=1.0, frictionAnchor=True)


        self.engine = Pikafish()

    def get_joint_index(self, name: str) -> int:
        """
        Get the index of the joint by its name. Works for links as well, assuming joints and links have different names.
        """
        assert name in self.joint_to_int
        return self.joint_to_int[name]

    def update_observations(self) -> None:
        # camera_link_pos, camera_link_ori = pybullet.getLinkState(self.robot_id, self.camera_idx)[:2]
        # camera_link_rotmat = pybullet.getMatrixFromQuaternion(camera_link_ori)
        # camera_link_rotmat = np.array(camera_link_rotmat).reshape((3, 3))
        # camera_target_link_pos = np.array(camera_link_pos)
        # camera_target_link_pos = camera_target_link_pos + camera_link_rotmat[:,0]
        # camera_mesh_color = [0.0, 1.0, 0.0, 1.0] # GREEN
        # pybullet.changeVisualShape(self.robot_id, self.camera_idx, rgbaColor=camera_mesh_color)

        # camera_view_matrix = pybullet.computeViewMatrix(
        #     cameraEyePosition=[camera_link_pos[0], camera_link_pos[1], camera_link_pos[2]],
        #     cameraTargetPosition=[camera_target_link_pos[0], camera_target_link_pos[1], camera_target_link_pos[2]],
        #     cameraUpVector=camera_link_rotmat[:,1]
        # )
        # For simplicity, let's fix the camera position to directly above the board
        camera_view_matrix = pybullet.computeViewMatrix(
            cameraEyePosition=[0, 0.1, 1.8],
            cameraTargetPosition=[0.0, 0.0, 0.0],
            cameraUpVector=[0.0, 0.0, 1.0]
        )

        # pybullet.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, camera_target_position)
        camera_proj_matrix = pybullet.computeProjectionMatrixFOV(fov=45.0, aspect=1.0, nearVal=0.1, farVal=10)
        # $initAxis(camera_link_pos, camera_link_ori)
        cameraImage = pybullet.getCameraImage(
            width=self.image_width,
            height=self.image_height,
            viewMatrix=camera_view_matrix,
            projectionMatrix=camera_proj_matrix,
            renderer=pybullet.ER_BULLET_HARDWARE_OPENGL
        )

        width, height, rgbPixels, depthPixels, segmentationMaskBuffer = cameraImage

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

        # plt.imsave('transformed_board.png', transformed_board)
        # self.board_seg_mask = transformed_board
        x_vals = np.linspace(width*5/320, width*314/320, num=10)
        y_vals = np.linspace(height*7/320, height*312/320, num=11)
        x_coords = (x_vals[:-1] + x_vals[1:]) / 2
        y_coords = (y_vals[:-1] + y_vals[1:]) / 2
        x_coords = x_coords.round().astype(int)
        y_coords = y_coords.round().astype(int)
        board = transformed_board[np.ix_(y_coords, x_coords)]
        self.board = board.astype(int)

        # For debugging
        # WHITE: tuple[Any | floating[Any]] = (np.max(transformed_board)+1,)
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
        # plt.imsave('transformed_board.png', transformed_board, cmap='inferno')

    def get_board_coordinates(self, indices: np.ndarray):
        rect = np.zeros((4, 2), dtype=np.float32)
        s = indices.sum(axis=1)
        diff = np.diff(indices, axis=1)
        rect[0] = indices[np.argmin(s)] # top left
        rect[1] = indices[np.argmax(diff)] # top right
        rect[2] = indices[np.argmin(diff)] # bottom left
        rect[3] = indices[np.argmax(s)] # bottom right
        return rect

    def adjust_lift_height(self, deltaY: float) -> None:
        """
        Extend arm (prismatic joint) by deltaY units.
        """
        lift_id = self.get_joint_index("joint_lift")
        # lift_info = pybullet.getJointInfo(self.robot_id, lift_id)
        # jointType = lift_info[2]
        # assert jointType == 1 # Prismatic

        lift_state = pybullet.getJointState(self.robot_id, lift_id)
        lift_pos = lift_state[0]
        # print("current lift pos:", lift_pos)
        target_lift_pos = lift_pos + deltaY
        # print("target lift pos:", target_lift_pos)

        pybullet.setJointMotorControl2(
            bodyIndex=self.robot_id,
            jointIndex=lift_id,
            controlMode=pybullet.POSITION_CONTROL,
            targetPosition=target_lift_pos
        )

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
        assert board.shape == (10, 9)
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

    def make_move(self, is_our_turn=True):
        if not is_our_turn:
            return

        # if self.i == 0:
        #     self.print_board()
        # self.i = (self.i+1) % 30

        fen = self.board_to_fen(self.board, is_our_turn)
        # print(fen)
        bestmove = asyncio.run(self.engine.get_best_move(fen))
        print("bestmove:", bestmove, flush=True)

    def get_position(self):
        return pybullet.getBasePositionAndOrientation(self.robot_id)[0]
    
    def get_robot_base_pose(self, robot_id):
        """
        Gets the base pose of the robot.

        Args:
            p: The PyBullet instance.
            robot_id: The ID of the robot in the PyBullet simulation.

        Returns:
            position: A tuple of (x, y, z) coordinates of the robot's base.
            orientation: A tuple of quaternion (x, y, z, w) representing the orientation.
            euler_orientation: A tuple of Euler angles (roll, pitch, yaw) derived from the quaternion.
        """
        # Get the base position and orientation as a quaternion
        position, orientation = pybullet.getBasePositionAndOrientation(robot_id)
        
        # Convert quaternion to Euler angles for easier interpretation if needed
        euler_orientation = pybullet.getEulerFromQuaternion(orientation)
        
        return position, orientation, euler_orientation
    

    def get_robot_ee_pose(self, robot_id):
        """
        Get the position and orientation of the end-effector.

        Args:
            p: PyBullet instance.
            robot_id: ID of the robot in the simulation.
            ee_link_index: Index of the end-effector link.

        Returns:
            position: (x, y, z) position of the end-effector.
            orientation: Quaternion (x, y, z, w) representing orientation.
            euler_orientation: Euler angles (roll, pitch, yaw) derived from the quaternion.
        """
        # index of the end-effector link
        ee_link_index = 17

        # Get the world position and orientation of the end-effector link
        position, orientation = pybullet.getLinkState(robot_id, ee_link_index)[:2]
        
        # Convert orientation quaternion to Euler angles
        euler_orientation = pybullet.getEulerFromQuaternion(orientation)
        
        return position, orientation, euler_orientation
