import heapq
import numpy as np
import pybullet as p

class ChineseChessRobot(Robot):
    def __init__(self, start_pos, obj_indices, piece_id_to_char, urdf_file=None):
        super().__init__(start_pos, obj_indices, piece_id_to_char, urdf_file)
        self.grid_size = (9, 10)  # Standard Chinese chess board size
        self.cell_size = 0.1  # Define the physical size of each cell on the board (in meters)
        self.board_grid = np.zeros(self.grid_size)  # 0: free, 1: occupied

    def set_piece_positions(self):
        """Update board grid with current piece positions."""
        for piece_id in self.obj_indices:
            pos, _ = p.getBasePositionAndOrientation(piece_id)
            grid_pos = self.world_to_grid(pos)
            self.board_grid[grid_pos] = 1  # Mark cell as occupied

    def world_to_grid(self, pos):
        """Convert world coordinates to grid coordinates."""
        x, y, _ = pos
        col = int((x + 0.45) // self.cell_size)
        row = int((y + 0.45) // self.cell_size)
        return (row, col)

    def grid_to_world(self, grid_pos):
        """Convert grid coordinates to world coordinates."""
        row, col = grid_pos
        x = col * self.cell_size - 0.45
        y = row * self.cell_size - 0.45
        return [x, y, 0.7]  # Set the height (z) above the board

    def a_star_search(self, start, goal):
        """A* search for finding a path from start to goal on the grid."""
        rows, cols = self.grid_size
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        cost_so_far = {start: 0}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                if self.board_grid[neighbor] == 1:  # Skip occupied cells
                    continue
                new_cost = cost_so_far[current] + 1
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (priority, neighbor))
                    came_from[neighbor] = current
        return None  # No path found

    def heuristic(self, a, b):
        """Heuristic function for A* (Manhattan distance)."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, pos):
        """Get valid neighbors for a position on the grid."""
        row, col = pos
        neighbors = [(row + dr, col + dc) for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]]
        return [(r, c) for r, c in neighbors if 0 <= r < self.grid_size[0] and 0 <= c < self.grid_size[1]]

    def reconstruct_path(self, came_from, current):
        """Reconstruct the path from A* search."""
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path

    def execute_path(self, path):
        """Execute the robot movement along a planned path."""
        for grid_pos in path:
            target_pos = self.grid_to_world(grid_pos)
            self.move_arm_to_position(target_pos)
            time.sleep(1/240)  # Sleep to allow motion completion

    def pick_and_place(self, piece_id, start, end):
        """High-level pick-and-place with collision-avoiding path planning."""
        self.set_piece_positions()  # Update board grid

        # Convert board coordinates to grid positions
        start_grid = self.world_to_grid(start)
        end_grid = self.world_to_grid(end)

        # Plan path with A* avoiding obstacles
        path = self.a_star_search(start_grid, end_grid)
        if path is None:
            print("No path found!")
            return

        # Execute path to pick the piece
        self.execute_path(path)

        # Pick the piece
        gripper_control(self, p, cmd=0)  # Close gripper
        time.sleep(1/240)

        # Plan path from start to end position
        path_to_place = self.a_star_search(start_grid, end_grid)
        if path_to_place is None:
            print("No path found to place the piece!")
            return

        # Execute path to place the piece
        self.execute_path(path_to_place)

        # Release the piece
        gripper_control(self, p, cmd=1)  # Open gripper
        time.sleep(1/240)

start_position = [0.22, 0.8, 0]
urdf_file = "path_to_robot_urdf.urdf"
obj_indices = [board_id]  # List of pieces on the board
piece_id_to_char = {board_id: " "}  # Dictionary to identify pieces
chess_robot = ChineseChessRobot(start_position, obj_indices, piece_id_to_char, urdf_file=urdf_file)

# Move a piece from (0, 0) to (0, 1)
piece_to_move = chess_robot.board_id  # Example piece ID
start_pos = BOARD_COORDS[(0, 0)]
end_pos = BOARD_COORDS[(0, 1)]
chess_robot.pick_and_place(piece_to_move, start_pos, end_pos)
