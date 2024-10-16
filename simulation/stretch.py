import numpy as np
import pybullet
from PIL import Image


def initAxis(center, quater):
    rot_mat = pybullet.getMatrixFromQuaternion(quater)
    rotmat = np.array(rot_mat).reshape((3, 3))
    pybullet.addUserDebugLine(lineFromXYZ=center, lineToXYZ=center + rotmat[:3, 0] * 0.1, lineColorRGB=[1, 0, 0],
                            lineWidth=10)
    pybullet.addUserDebugLine(lineFromXYZ=center, lineToXYZ=center + rotmat[:3, 1] * 0.1, lineColorRGB=[0, 1, 0],
                            lineWidth=10)
    pybullet.addUserDebugLine(lineFromXYZ=center, lineToXYZ=center + rotmat[:3, 2] * 0.1, lineColorRGB=[0, 0, 1],
                            lineWidth=10)


class Robot:
    def __init__(self, start_pos=[0.4,0.3,0.4], urdf_file=None, resource_dir=None, project_root_dir=None):
        self.gripperMaxForce = 1000.0
        self.armMaxForce = 200.0

        # Unused variables for now
        # self.start_pos = start_pos
        # self.urdf_file = urdf_file
        # self.project_dir = project_root_dir
        # self.resource_dir = resource_dir

        self.robot_id = pybullet.loadURDF(urdf_file, start_pos)

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

        image_aspect_ratio = 1.5
        self.image_height = 640
        self.image_width = int(image_aspect_ratio * self.image_height)
        self.board_image = None # shape is (height, width, 4), RGBA format
        self.board_seg_mask = None

    def get_joint_index(self, name: str) -> int:
        """
        Get the index of the joint by its name. Works for links as well, assuming joints and links have different names.
        """
        assert name in self.joint_to_int
        return self.joint_to_int[name]

    def get_observation(self) -> None:
        camera_link_pos, camera_link_ori = pybullet.getLinkState(self.robot_id, self.camera_idx)[:2]
        camera_link_rotmat = pybullet.getMatrixFromQuaternion(camera_link_ori)
        camera_link_rotmat = np.array(camera_link_rotmat).reshape((3, 3))
        camera_target_link_pos = np.array(camera_link_pos)
        camera_target_link_pos = camera_target_link_pos + camera_link_rotmat[:,0]
        camera_mesh_color = [0.0, 1.0, 0.0, 1.0] # GREEN
        pybullet.changeVisualShape(self.robot_id, self.camera_idx, rgbaColor=camera_mesh_color)

        camera_view_matrix = pybullet.computeViewMatrix(
            cameraEyePosition=[camera_link_pos[0], camera_link_pos[1], camera_link_pos[2]],
            cameraTargetPosition=[camera_target_link_pos[0], camera_target_link_pos[1], camera_target_link_pos[2]],
            cameraUpVector=camera_link_rotmat[:,1]
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
        self.board_image = rgbPixels
        self.board_seg_mask = segmentationMaskBuffer
    
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
        print("current lift pos:", lift_pos)
        target_lift_pos = lift_pos + deltaY
        print("target lift pos:", target_lift_pos)

        pybullet.setJointMotorControl2(
            bodyIndex=self.robot_id,
            jointIndex=lift_id,
            controlMode=pybullet.POSITION_CONTROL,
            targetPosition=target_lift_pos
        )


    def make_move(self):
        assert self.board_image is not None
        assert self.board_seg_mask is not None
        self.adjust_lift_height(0.1)
        # image = Image.fromarray(self.board_image, 'RGBA')
        # image.save('board.png')
        # exit()
