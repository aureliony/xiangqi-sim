import asyncio
import os
import cv2
import numpy as np
import pybullet
import pybullet_data
import time
import threading
import cv2
from collections import defaultdict


from engine.pikafish import Pikafish

BOUNDS = np.float32([[-0.3, 0.3], [-0.8, -0.2], [0, 0.15]])
PIXEL_SIZE = 0.00267857

class Robotiq2F85:
    """Gripper handling for Robotiq 2F85."""

    def __init__(self, robot, tool, position):
        self.robot = robot
        self.tool = tool
        pos = [position[0] + 0.1339999999999999, position[1] - 0.49199999999872496, position[2] + 0.5]
        rot = pybullet.getQuaternionFromEuler([np.pi, 0, np.pi])
        urdf = 'resource/urdf/robotiq_2f_85/robotiq_2f_85.urdf'
        self.body = pybullet.loadURDF(urdf, pos, rot)
        self.n_joints = pybullet.getNumJoints(self.body)
        self.activated = False

        # Connect gripper base to robot tool.
        pybullet.createConstraint(self.robot, tool, self.body, 0, jointType=pybullet.JOINT_FIXED, jointAxis=[0, 0, 0], parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, -0.07], childFrameOrientation=pybullet.getQuaternionFromEuler([0, 0, np.pi / 2]))

        # Set friction coefficients for gripper fingers.
        for i in range(pybullet.getNumJoints(self.body)):
            pybullet.changeDynamics(self.body, i, lateralFriction=10.0, spinningFriction=1.0, rollingFriction=1.0, frictionAnchor=True)

        # Start thread to handle additional gripper constraints.
        self.motor_joint = 1
        self.constraints_thread = threading.Thread(target=self.step)
        self.constraints_thread.daemon = True
        self.constraints_thread.start()


    # Control joint positions by enforcing hard contraints on gripper behavior.
    # Set one joint as the open/close motor joint (other joints should mimic).
    def step(self):
        while True:
            try:
                currj = [pybullet.getJointState(self.body, i)[0] for i in range(self.n_joints)]
                indj = [6, 3, 8, 5, 10]
                targj = [currj[1], -currj[1], -currj[1], currj[1], currj[1]]
                pybullet.setJointMotorControlArray(self.body, indj, pybullet.POSITION_CONTROL, targj, positionGains=np.ones(5))
            except:
                return
            time.sleep(0.001)
            
    # Close gripper fingers.
    def activate(self):
        pybullet.setJointMotorControl2(self.body, self.motor_joint, pybullet.VELOCITY_CONTROL, targetVelocity=1, force=10)
        self.activated = True

    # Open gripper fingers.
    def release(self):
        pybullet.setJointMotorControl2(self.body, self.motor_joint, pybullet.VELOCITY_CONTROL, targetVelocity=-1, force=10)
        self.activated = False

    # If activated and object in gripper: check object contact.
    # If activated and nothing in gripper: check gripper contact.
    # If released: check proximity to surface (disabled).
    def detect_contact(self):
        obj, _, ray_frac = self.check_proximity()
        if self.activated:
            empty = self.grasp_width() < 0.01
            cbody = self.body if empty else obj
            if obj == self.body or obj == 0:
                return False
            return self.external_contact(cbody)
    #   else:
    #     return ray_frac < 0.14 or self.external_contact()

    # Return if body is in contact with something other than gripper
    def external_contact(self, body=None):
        if body is None:
            body = self.body
        pts = pybullet.getContactPoints(bodyA=body)
        pts = [pt for pt in pts if pt[2] != self.body]
        return len(pts) > 0  # pylint: disable=g-explicit-length-test

    def check_grasp(self):
        while self.moving():
            time.sleep(0.001)
        success = self.grasp_width() > 0.01
        return success
    
    def grasp_width(self):
        lpad = np.array(pybullet.getLinkState(self.body, 4)[0])
        rpad = np.array(pybullet.getLinkState(self.body, 9)[0])
        dist = np.linalg.norm(lpad - rpad) - 0.047813
        return dist

    def check_proximity(self):
        ee_pos = np.array(pybullet.getLinkState(self.robot, self.tool)[0])
        tool_pos = np.array(pybullet.getLinkState(self.body, 0)[0])
        vec = (tool_pos - ee_pos) / np.linalg.norm((tool_pos - ee_pos))
        ee_targ = ee_pos + vec
        ray_data = pybullet.rayTest(ee_pos, ee_targ)[0]
        obj, link, ray_frac = ray_data[0], ray_data[1], ray_data[2]
        return obj, link, ray_frac
  
# Gym-style environment code

class PickPlaceEnv():

    def __init__(self, render=False, high_res=False, high_frame_rate=False):
        self.dt = 1/480
        self.sim_step = 0

        # Configure and start PyBullet.
        # python3 -m pybullet_utils.runServer
        # pybullet.connect(pybullet.SHARED_MEMORY)  # pybullet.GUI for local GUI.
        pybullet.connect(pybullet.DIRECT)  # pybullet.GUI for local GUI.
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 1)
        pybullet.setPhysicsEngineParameter(enableFileCaching=0)
        assets_path = os.path.dirname(os.path.abspath(""))
        pybullet.setAdditionalSearchPath(assets_path)
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        pybullet.setTimeStep(self.dt)

        self.home_joints = (np.pi / 2, -np.pi / 2, np.pi / 2, -np.pi / 2, 3 * np.pi / 2, 0)  # Joint angles: (J0, J1, J2, J3, J4, J5).
        self.home_ee_euler = (np.pi, 0, np.pi)  # (RX, RY, RZ) rotation in Euler angles.
        self.ee_link_id = 9  # Link ID of UR5 end effector.
        self.tip_link_id = 10  # Link ID of gripper finger tips.
        self.gripper = None

        self.render = render
        self.high_res = high_res
        self.high_frame_rate = high_frame_rate
        self.position = [0, 0.62, 0.62]
        
        image_aspect_ratio = 1.0
        self.image_height = 320
        self.image_width = int(image_aspect_ratio * self.image_height)
        self.board_id = None
        self.piece_id_to_char = None
        self.piece_id_to_fen = None
        self.board = None
        self.i = 0
        
        self.engine = Pikafish()


    def reset(self, object_list):
        pybullet.resetSimulation(pybullet.RESET_USE_DEFORMABLE_WORLD)
        pybullet.setGravity(0, 0, -9.8)
        self.cache_video = []

        # Temporarily disable rendering to load URDFs faster.
        # pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 0)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1) # nah
        

        urdf_dir = "resource/urdf"
        # Add robot.
        #pybullet.loadURDF("resource/urdf/plane.urdf", [0, 0, -0.001])
        
        ################ Plane

        plane_id = pybullet.loadURDF(os.path.join(urdf_dir,"/plane.urdf"), [0, 0, 0])
        plane_texture_id = pybullet.loadTexture("resource/texture/texture1.jpg")
        pybullet.changeVisualShape(0, -1, textureUniqueId=plane_texture_id)
                
        ################ Table

        table_position = [0, 0, 0]
        table_scaling = 1.0
        table_orientation = pybullet.getQuaternionFromEuler([0, 0, np.pi/2])
        table_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"table.urdf"),\
                                        useFixedBase=True,
                                        basePosition=table_position,\
                                        baseOrientation=table_orientation,\
                                        globalScaling=table_scaling)
        # table_texture_id = pybullet.loadTexture(os.path.join(root_dir,"resource/texture/table.png"))
        table_texture_id = pybullet.loadTexture("resource/texture/table.png")
        pybullet.changeVisualShape(table_id,0,textureUniqueId=table_texture_id)

        ################ Board
        
        board_position = [0.08, -0.05, 0.6]
        board_scaling = 0.5
        board_orientation = pybullet.getQuaternionFromEuler([0, 0, np.pi])
        board_id = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chessboard/chessboard.urdf"),\
                                        useFixedBase=True,
                                        basePosition=board_position,\
                                        baseOrientation=board_orientation,\
                                        globalScaling=board_scaling)
        self.board_id = board_id
        pybullet.changeVisualShape(board_id, -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
        
        ################ Chess Pieces

        self.piece_id_to_char, self.piece_id_to_fen = initialize_chess_pieces(board_position)

        ################ Robot
        
        self.robot_id = pybullet.loadURDF("resource/urdf/ur5e/ur5e.urdf", self.position, flags=pybullet.URDF_USE_MATERIAL_COLORS_FROM_MTL)
        self.ghost_id = pybullet.loadURDF("resource/urdf/ur5e/ur5e.urdf", [self.position[0], self.position[1], self.position[2] - 10])  # For forward kinematics.
        self.joint_ids = [pybullet.getJointInfo(self.robot_id, i) for i in range(pybullet.getNumJoints(self.robot_id))]
        self.joint_ids = [j[0] for j in self.joint_ids if j[2] == pybullet.JOINT_REVOLUTE]

        # Move robot to home configuration.
        for i in range(len(self.joint_ids)):
            pybullet.resetJointState(self.robot_id, self.joint_ids[i], self.home_joints[i])

        # Add gripper.
        if self.gripper is not None:
            while self.gripper.constraints_thread.is_alive():
                self.constraints_thread_active = False
        self.gripper = Robotiq2F85(self.robot_id, self.ee_link_id, self.position)
        self.gripper.release()

        for _ in range(200):
            pybullet.stepSimulation()

        # # record object positions at reset
        # self.init_pos = {name: self.get_obj_pos(name) for name in object_list}

        return self.update_observations() #get_observation()

    def servoj(self, joints):
        """Move to target joint positions with position control."""
        pybullet.setJointMotorControlArray(
        bodyIndex=self.robot_id,
        jointIndices=self.joint_ids,
        controlMode=pybullet.POSITION_CONTROL,
        targetPositions=joints,
        positionGains=[0.01]*6)

    def movep(self, position):
        """Move to target end effector position."""
        joints = pybullet.calculateInverseKinematics(
            bodyUniqueId=self.robot_id,
            endEffectorLinkIndex=self.tip_link_id,
            targetPosition=position,
            targetOrientation=pybullet.getQuaternionFromEuler(self.home_ee_euler),
            maxNumIterations=100)
        self.servoj(joints)

    def get_ee_pos(self):
        ee_xyz = np.float32(pybullet.getLinkState(self.robot_id, self.tip_link_id)[0])
        return ee_xyz

    def step(self, action=None):
        """Do pick and place motion primitive."""
        pick_xyz, place_xyz = action['pick'].copy(), action['place'].copy()

        # Set fixed primitive z-heights.
        hover_xyz = np.float32([pick_xyz[0], place_xyz[1], 0.8])
        # if pick_pos.shape[-1] == 2:
        #     pick_xyz = np.append(pick_pos, 0.025)
        # else:
        #     pick_xyz = pick_pos
        #     pick_xyz[2] = 0.025
        # if place_pos.shape[-1] == 2:
        #     place_xyz = np.append(place_pos, 0.15)
        # else:
        #     place_xyz = place_pos
        #     place_xyz[2] = 0.15

        # Move to object.
        ee_xyz = self.get_ee_pos()
        while np.linalg.norm(hover_xyz - ee_xyz) > 0.01:
            self.movep(hover_xyz)
            self.step_sim_and_render()
            ee_xyz = self.get_ee_pos()

        while np.linalg.norm(pick_xyz - ee_xyz) > 0.01:
            self.movep(pick_xyz)
            self.step_sim_and_render()
            ee_xyz = self.get_ee_pos()

        # Pick up object.
        self.gripper.activate()
        for _ in range(240):
            self.step_sim_and_render()
        while np.linalg.norm(hover_xyz - ee_xyz) > 0.01:
            self.movep(hover_xyz)
            self.step_sim_and_render()
            ee_xyz = self.get_ee_pos()

        for _ in range(50):
            self.step_sim_and_render()

        # Move to place location.
        while np.linalg.norm(place_xyz - ee_xyz) > 0.01:
            self.movep(place_xyz)
            self.step_sim_and_render()
            ee_xyz = self.get_ee_pos()

        # Place down object.
        while (not self.gripper.detect_contact()) and (place_xyz[2] > 0.66):
            place_xyz[2] -= 0.001
            self.movep(place_xyz)
            for _ in range(3):
                self.step_sim_and_render()
        self.gripper.release()
        for _ in range(240):
            self.step_sim_and_render()
            
        place_xyz[2] = 0.8
        ee_xyz = self.get_ee_pos()
        while np.linalg.norm(place_xyz - ee_xyz) > 0.01:
            self.movep(place_xyz)
            self.step_sim_and_render()
            ee_xyz = self.get_ee_pos()

        observation = self.update_observations() # get_observation()
        reward = self.get_reward()
        done = False
        info = {}
        return observation, reward, done, info

    # def set_alpha_transparency(self, alpha: float) -> None:
    #     for id in range(20):
    #         visual_shape_data = pybullet.getVisualShapeData(id)
    #         for i in range(len(visual_shape_data)):
    #             object_id, link_index, _, _, _, _, _, rgba_color = visual_shape_data[i]
    #             rgba_color = list(rgba_color[0:3]) +  [alpha]
    #             pybullet.changeVisualShape(
    #                 self.robot_id, linkIndex=i, rgbaColor=rgba_color)
    #             pybullet.changeVisualShape(
    #                 self.gripper.body, linkIndex=i, rgbaColor=rgba_color)

    def step_sim_and_render(self):
        pybullet.stepSimulation()
        self.sim_step += 1

        interval = 40 if self.high_frame_rate else 60
        # Render current image at 8 FPS.
        # if self.sim_step % interval == 0 and self.render:
        #     self.cache_video.append(self.get_camera_image())

    # def get_camera_image(self):
    #     if not self.high_res:
    #         image_size = (240, 240)
    #         intrinsics = (120., 0, 120., 0, 120., 120., 0, 0, 1)
    #     else:
    #         image_size=(360, 360)
    #         intrinsics=(180., 0, 180., 0, 180., 180., 0, 0, 1)
    #         color, _, _, _, _ = env.render_image(image_size, intrinsics)
    #     return color

    def get_reward(self):
        return None

    # def get_observation(self):
    #     observation = {}

    #     # Render current image.
    #     color, depth, position, orientation, intrinsics = self.render_image()

    #     # Get heightmaps and colormaps.
    #     points = self.get_pointcloud(depth, intrinsics)
    #     position = np.float32(position).reshape(3, 1)
    #     rotation = pybullet.getMatrixFromQuaternion(orientation)
    #     rotation = np.float32(rotation).reshape(3, 3)
    #     transform = np.eye(4)
    #     transform[:3, :] = np.hstack((rotation, position))
    #     points = self.transform_pointcloud(points, transform)
    #     heightmap, colormap, xyzmap = self.get_heightmap(points, color, BOUNDS, PIXEL_SIZE)

    #     observation["image"] = colormap
    #     observation["xyzmap"] = xyzmap

    #     return observation

    # def render_image(self, image_size=(720, 720), intrinsics=(360., 0, 360., 0, 360., 360., 0, 0, 1)):

    #     # Camera parameters.
    #     position = (0, -0.85, 0.4)
    #     orientation = (np.pi / 4 + np.pi / 48, np.pi, np.pi)
    #     orientation = pybullet.getQuaternionFromEuler(orientation)
    #     zrange = (0.01, 10.)
    #     noise=True

    #     # OpenGL camera settings.
    #     lookdir = np.float32([0, 0, 1]).reshape(3, 1)
    #     updir = np.float32([0, -1, 0]).reshape(3, 1)
    #     rotation = pybullet.getMatrixFromQuaternion(orientation)
    #     rotm = np.float32(rotation).reshape(3, 3)
    #     lookdir = (rotm @ lookdir).reshape(-1)
    #     updir = (rotm @ updir).reshape(-1)
    #     lookat = position + lookdir
    #     focal_len = intrinsics[0]
    #     znear, zfar = (0.01, 10.)
    #     viewm = pybullet.computeViewMatrix(position, lookat, updir)
    #     fovh = (image_size[0] / 2) / focal_len
    #     fovh = 180 * np.arctan(fovh) * 2 / np.pi

    #     # Notes: 1) FOV is vertical FOV 2) aspect must be float
    #     aspect_ratio = image_size[1] / image_size[0]
    #     projm = pybullet.computeProjectionMatrixFOV(fovh, aspect_ratio, znear, zfar)

    #     # Render with OpenGL camera settings.
    #     _, _, color, depth, segm = pybullet.getCameraImage(
    #         width=image_size[1],
    #         height=image_size[0],
    #         viewMatrix=viewm,
    #         projectionMatrix=projm,
    #         shadow=1,
    #         flags=pybullet.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
    #         renderer=pybullet.ER_BULLET_HARDWARE_OPENGL)

    #     # Get color image.
    #     color_image_size = (image_size[0], image_size[1], 4)
    #     color = np.array(color, dtype=np.uint8).reshape(color_image_size)
    #     color = color[:, :, :3]  # remove alpha channel
    #     if noise:
    #         color = np.int32(color)
    #         color += np.int32(np.random.normal(0, 3, color.shape))
    #         color = np.uint8(np.clip(color, 0, 255))

    #     # Get depth image.
    #     depth_image_size = (image_size[0], image_size[1])
    #     zbuffer = np.float32(depth).reshape(depth_image_size)
    #     depth = (zfar + znear - (2 * zbuffer - 1) * (zfar - znear))
    #     depth = (2 * znear * zfar) / depth
    #     if noise:
    #         depth += np.random.normal(0, 0.003, depth.shape)

    #     intrinsics = np.float32(intrinsics).reshape(3, 3)
    #     return color, depth, position, orientation, intrinsics

    # def get_pointcloud(self, depth, intrinsics):
    #     """Get 3D pointcloud from perspective depth image.
    #     Args:
    #     depth: HxW float array of perspective depth in meters.
    #     intrinsics: 3x3 float array of camera intrinsics matrix.
    #     Returns:
    #     points: HxWx3 float array of 3D points in camera coordinates.
    #     """
    #     height, width = depth.shape
    #     xlin = np.linspace(0, width - 1, width)
    #     ylin = np.linspace(0, height - 1, height)
    #     px, py = np.meshgrid(xlin, ylin)
    #     px = (px - intrinsics[0, 2]) * (depth / intrinsics[0, 0])
    #     py = (py - intrinsics[1, 2]) * (depth / intrinsics[1, 1])
    #     points = np.float32([px, py, depth]).transpose(1, 2, 0)
    #     return points

    # def transform_pointcloud(self, points, transform):
    #     """Apply rigid transformation to 3D pointcloud.
    #     Args:
    #     points: HxWx3 float array of 3D points in camera coordinates.
    #     transform: 4x4 float array representing a rigid transformation matrix.
    #     Returns:
    #     points: HxWx3 float array of transformed 3D points.
    #     """
    #     padding = ((0, 0), (0, 0), (0, 1))
    #     homogen_points = np.pad(points.copy(), padding,
    #                             'constant', constant_values=1)
    #     for i in range(3):
    #         points[Ellipsis, i] = np.sum(transform[i, :] * homogen_points, axis=-1)
    #     return points

    # def get_heightmap(self, points, colors, bounds, pixel_size):
    #     """Get top-down (z-axis) orthographic heightmap image from 3D pointcloud.
    #     Args:
    #     points: HxWx3 float array of 3D points in world coordinates.
    #     colors: HxWx3 uint8 array of values in range 0-255 aligned with points.
    #     bounds: 3x2 float array of values (rows: X,Y,Z; columns: min,max) defining
    #         region in 3D space to generate heightmap in world coordinates.
    #     pixel_size: float defining size of each pixel in meters.
    #     Returns:
    #     heightmap: HxW float array of height (from lower z-bound) in meters.
    #     colormap: HxWx3 uint8 array of backprojected color aligned with heightmap.
    #     xyzmap: HxWx3 float array of XYZ points in world coordinates.
    #     """
    #     width = int(np.round((bounds[0, 1] - bounds[0, 0]) / pixel_size))
    #     height = int(np.round((bounds[1, 1] - bounds[1, 0]) / pixel_size))
    #     heightmap = np.zeros((height, width), dtype=np.float32)
    #     colormap = np.zeros((height, width, colors.shape[-1]), dtype=np.uint8)
    #     xyzmap = np.zeros((height, width, 3), dtype=np.float32)

    #     # Filter out 3D points that are outside of the predefined bounds.
    #     ix = (points[Ellipsis, 0] >= bounds[0, 0]) & (points[Ellipsis, 0] < bounds[0, 1])
    #     iy = (points[Ellipsis, 1] >= bounds[1, 0]) & (points[Ellipsis, 1] < bounds[1, 1])
    #     iz = (points[Ellipsis, 2] >= bounds[2, 0]) & (points[Ellipsis, 2] < bounds[2, 1])
    #     valid = ix & iy & iz
    #     points = points[valid]
    #     colors = colors[valid]

    #     # Sort 3D points by z-value, which works with array assignment to simulate
    #     # z-buffering for rendering the heightmap image.
    #     iz = np.argsort(points[:, -1])
    #     points, colors = points[iz], colors[iz]
    #     px = np.int32(np.floor((points[:, 0] - bounds[0, 0]) / pixel_size))
    #     py = np.int32(np.floor((points[:, 1] - bounds[1, 0]) / pixel_size))
    #     px = np.clip(px, 0, width - 1)
    #     py = np.clip(py, 0, height - 1)
    #     heightmap[py, px] = points[:, 2] - bounds[2, 0]
    #     for c in range(colors.shape[-1]):
    #         colormap[py, px, c] = colors[:, c]
    #         xyzmap[py, px, c] = points[:, c]
    #     colormap = colormap[::-1, :, :]  # Flip up-down.
    #     xv, yv = np.meshgrid(np.linspace(BOUNDS[0, 0], BOUNDS[0, 1], height),
    #                         np.linspace(BOUNDS[1, 0], BOUNDS[1, 1], width))
    #     xyzmap[:, :, 0] = xv
    #     xyzmap[:, :, 1] = yv
    #     xyzmap = xyzmap[::-1, :, :]  # Flip up-down.
    #     heightmap = heightmap[::-1, :]  # Flip up-down.
    #     return heightmap, colormap, xyzmap

    # def get_obj_id(self, obj_name):
    #     try:
    #         if obj_name in self.obj_name_to_id:
    #             obj_id = self.obj_name_to_id[obj_name]
    #         else:
    #             obj_name = obj_name.replace('circle', 'bowl').replace('square', 'block').replace('small', '').strip()
    #             obj_id = self.obj_name_to_id[obj_name]
    #     except:
    #         print(f'requested_name="{obj_name}"')
    #         print(f'available_objects_and_id="{self.obj_name_to_id}')
    #     return obj_id

    # def get_obj_pos(self, obj_name):
    #     obj_name = obj_name.replace('the', '').replace('_', ' ').strip()
    #     pick_id = self.get_obj_id(obj_name)
    #     pose = pybullet.getBasePositionAndOrientation(pick_id)
    #     position = np.float32(pose[0])
    #     return position

    def get_bounding_box(self, obj_name):
        obj_id = self.get_obj_id(obj_name)
        return pybullet.getAABB(obj_id)
    
    def update_observations(self) -> None:
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
    

urdf_dir = "resource/urdf"

        
def pos_to_coordinates(pos):
    # pos is a string like "a1"
    # returns the coordinates in the chessboard_positions matrix
    # a1 = [0,8]
    # h10 = [9,0]
    col = ord(pos[0]) - ord('a')
    row = 9 - int(pos[1])
    return row, col

def coordinates_to_pos(row, col):
    # row, col are integers
    # returns the position string
    # [0,8] = a1
    # [9,0] = h10
    return chr(col + ord('a')) + str(9 - row)
      


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

def initialize_chess_pieces(chessboard_position):
    
    # Dynamically create variables and store their IDs in a dictionary
    piece_id_map = {}
    
    gap = 0.086 # chess spacing apart
    chess_bottom_left_corner = [chessboard_position[0]- 5 * gap - 0.015, chessboard_position[1] - 4 * gap + 0.016, chessboard_position[2] + 0.066]
    chessboard_positions = [ [[0,0,0] for i in range(9)] for j in range(10)]
    for row in range(10):
        for col in range(9):
            chessboard_positions[row][col] = [
                chess_bottom_left_corner[0] + col * gap,  # X position
                chess_bottom_left_corner[1] + (9 - row) * gap,  # Y position (invert rows for bottom-left start)
                chess_bottom_left_corner[2]  # Z position (same for all)
            ]
            
    # ################ FOR TESTING
    for i in range(10):
        print(chessboard_positions[i])
        
    # # pretty print the chessboard with using coordinates_to_pos
    # for i in range(10):
    #     print([coordinates_to_pos(i,j) for j in range(9)])

    # ################ CHESS PIECES
    
            
    PIECE_MASS = 0.01
    cp_scaling = 0.20 # chess piece scaling
    obj_friction_ceof = 4000.0
    b_orientation = pybullet.getQuaternionFromEuler([0, 0, -np.pi / 2]) # black pieces orientation
    r_orientation = pybullet.getQuaternionFromEuler([0, 0, 0]) # red pieces orientation

    # Black pieces
    black = ["b1", "b2", "b3", "b4", "b5", "b6", "b7", "b8", "b9", "b10", "b11", "b12", "b13", "b14", "b15", "b16"]
    
    
    for b in black:
        id_name = f"{b}_id"
        position_name = f"{b}_position"
        coords = pos_to_coordinates(chess_pieces[b])
        globals()[position_name] = chessboard_positions[coords[0]][coords[1]]
        globals()[id_name] = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/"+b+"/model.urdf"),
                                        useFixedBase=False,
                                        globalScaling=cp_scaling,
                                        basePosition=[globals()[position_name][0], globals()[position_name][1], globals()[position_name][2] + 0.04],
                                        baseOrientation=b_orientation)
        pybullet.changeVisualShape(globals()[id_name], -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
        pybullet.changeDynamics(globals()[id_name], -1, lateralFriction=obj_friction_ceof)
        pybullet.changeDynamics(globals()[id_name], -1, mass=PIECE_MASS)
        # Store IDs in a dictionary for easier access
        piece_id_map[b] = globals()[id_name]
    
    # Red pieces
    red = ["r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15", "r16"]
    for r in red:
        id_name = f"{r}_id"
        position_name = f"{r}_position"
        coords = pos_to_coordinates(chess_pieces[r])
        globals()[position_name] = chessboard_positions[coords[0]][coords[1]]
        globals()[id_name] = pybullet.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/chesspieces/"+r+"/model.urdf"),
                                        useFixedBase=False,
                                        globalScaling=cp_scaling,
                                        basePosition=[globals()[position_name][0], globals()[position_name][1], globals()[position_name][2] + 0.04],
                                        baseOrientation=r_orientation)
        pybullet.changeVisualShape(globals()[id_name], -1, rgbaColor=[0.824, 0.706, 0.549, 1.0])
        pybullet.changeDynamics(globals()[id_name], -1, lateralFriction=obj_friction_ceof)
        pybullet.changeDynamics(globals()[id_name], -1, mass=PIECE_MASS)
        # Store IDs in a dictionary for easier access
        piece_id_map[r] = globals()[id_name]
        
    
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
    

