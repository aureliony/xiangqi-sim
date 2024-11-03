import numpy as np
import time
import pybullet as p
import pybullet_data
from stretch import StretchRobot
from path_planning import AStarPlanner, RRTPlanner
from object_detection import PieceDetector
from advanced_techniques import SliderControl, ObstacleAvoidance
from chinese_chess_engine_alpha_zero import Game, Board

# Setup the pybullet simulation
def setup_simulation():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)  # Set gravity for realistic simulation
    p.loadURDF("plane.urdf")
    table_id = p.loadURDF("table.urdf", basePosition=[0, 0, 0])
    cube_id = p.loadURDF("cube.urdf", basePosition=[0.5, 0.5, 0.1])
    robot = StretchRobot(urdf_path="stretch.urdf")

    # Improve visualization settings
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)
    p.resetDebugVisualizerCamera(
        cameraDistance=1.5,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0.5, 0.5, 0]
    )
    return robot, table_id, cube_id

# Motion planning for robot to move to a target position
def move_to_position(robot, target_pos):
    # Initialize path planning algorithm
    path_planner = AStarPlanner()  # Use A* planner for precise motion planning
    obstacle_avoidance = ObstacleAvoidance()  # Module to handle dynamic obstacles
    path = path_planner.plan(robot.get_current_position(), target_pos)

    # Move through the planned path smoothly
    for waypoint in path:
        if obstacle_avoidance.is_obstacle_nearby(waypoint):
            print("Obstacle detected, recalculating path...")
            path = path_planner.plan(robot.get_current_position(), target_pos)
            continue

        if not robot.check_singularity(waypoint):  # Avoid singularities
            robot.move_to(waypoint)
            time.sleep(0.02)  # Reduced delay for smoother and faster movement
        else:
            print("Singularity detected, recalculating path...")
            path = path_planner.plan(robot.get_current_position(), target_pos)

# Motion planning for robot to move and grasp chess pieces
def move_and_grasp(robot, start_pos, target_pos):
    move_to_position(robot, start_pos)  # Move to the start position

    # Perform grasping action
    if robot.is_reachable(start_pos):
        robot.adjust_for_grasp(start_pos)  # Adjust arm for optimal grasp
        robot.grasp(start_pos)  # Grasp the piece
        if not robot.check_grasp_stability():
            print("Grasp failed, retrying...")
            robot.grasp(start_pos)
    else:
        print("Start position is not reachable.")

    # Move the piece to the new target position
    move_to_position(robot, target_pos)
    robot.release()

    # Move to a safe position after releasing
    safe_pos = [target_pos[0], target_pos[1], target_pos[2] + 0.1]
    move_to_position(robot, safe_pos)

# Main game loop to handle the gameplay and interactions
def play_chinese_chess():
    # Setup simulation environment
    robot, table_id, cube_id = setup_simulation()

    # Initialize the chess engine and game logic from ChineseChess-AlphaZero
    game = Game()
    board = Board()
    piece_detector = PieceDetector()
    slider_control = SliderControl()  # Advanced technique to speed up arm movement

    while not game.is_game_over():
        # Get the current board state
        board_state = board.get_board_state()
        detected_pieces = piece_detector.detect(board_state)

        # Evaluate and make the best move using the AlphaZero model
        best_move = game.get_best_move(board_state)
        start_pos, target_pos = best_move['from'], best_move['to']

        # Move robot to grasp and move the chess piece
        if piece_detector.verify_piece_presence(start_pos):
            slider_control.optimize_path(robot)  # Optimize movement using slider control
            move_and_grasp(robot, start_pos, target_pos)
        else:
            print("Piece at start position not detected, recalculating...")
            continue

        # Update board state
        board.update(best_move)
        game.update(board)

        # Detect fallen pieces or unexpected situations
        fallen_pieces = piece_detector.detect_fallen_pieces()
        if fallen_pieces:
            print(f"Handling fallen pieces: {fallen_pieces}")
            for piece in fallen_pieces:
                recovery_pos = piece['recovery_position']
                move_and_grasp(robot, piece['current_position'], recovery_pos)

    print("Game Over!")

# Test motion planning for the arm
def test_motion_planning():
    # Setup simulation environment
    robot, _, _ = setup_simulation()

    # Define a series of target positions for testing
    test_positions = [
        [0.4, 0.3, 0.5],
        [0.6, 0.3, 0.5],
        [0.5, 0.5, 0.3],
        [0.4, 0.4, 0.6]
    ]

    # Move the robot arm to each target position
    for target_pos in test_positions:
        print(f"Moving to position: {target_pos}")
        move_to_position(robot, target_pos)
        time.sleep(1)  # Pause for a moment to observe the movement

if __name__ == "__main__":
    # Uncomment the function you want to run
    # play_chinese_chess()
    test_motion_planning()

# Updating the `adjust_lift_height` method and incorporating it into motion planning
def adjust_lift_height(self, deltaY: float) -> None:
    """
    Extend arm (prismatic joint) by deltaY units.
    """
    lift_id = self.get_joint_index("joint_lift")
    lift_state = p.getJointState(self.robot_id, lift_id)
    lift_pos = lift_state[0]
    target_lift_pos = lift_pos + deltaY
    p.setJointMotorControl2(
        bodyIndex=self.robot_id,
        jointIndex=lift_id,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target_lift_pos
    )

# Integrating motion planning with `make_move` function
def make_move(self, start_pos, target_pos):
    assert self.board_image is not None
    assert self.board_seg_mask is not None
    
    # Adjust the lift height to prepare for grasping
    self.adjust_lift_height(0.1)
    
    # Use motion planning to move and grasp the piece
    move_and_grasp(self, start_pos, target_pos)
    
    # Lower the lift after completing the move
    self.adjust_lift_height(-0.1)
