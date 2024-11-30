import time
import numpy as np
import pybullet

from simulation.robotiq import SimulationEnv
from scipy.spatial import KDTree

# Tree representation
class Node:
    def __init__(self, position, parent=None):
        assert isinstance(position, np.ndarray)
        assert parent is None or isinstance(parent, Node)
        self.position = np.array(position)
        self.parent = parent


class SimulationEnvRRT(SimulationEnv):
    def __init__(self):
        super().__init__()

    def move_object(self, start_xyz, end_xyz):
        start_xyz = np.array(start_xyz)
        end_xyz = np.array(end_xyz)
        hover_height = 0.1
        hover_start_xyz = start_xyz.copy()
        hover_end_xyz = end_xyz.copy()
        hover_start_xyz[2] += hover_height
        hover_end_xyz[2] += hover_height

        def sample_random_point(bounds):
            return np.random.uniform(low=bounds[0], high=bounds[1])

        def find_nearest(tree, point):
            # # TODO: make this faster than O(n)
            # return min(tree, key=lambda node: np.linalg.norm(node.position - point))
        
            kdtree = KDTree([node.position for node in tree])
            _, idx = kdtree.query(point)
            return tree[idx]

        def is_collision_free(p1, p2):
            steps = int(np.linalg.norm(p2 - p1) / 0.1)  # Break path into small steps
            for step in range(steps):
                pos = p1 + step * (p2 - p1) / steps

                # Expand the collision radius by checking within a margin
                ray_results = pybullet.rayTest(pos, pos + [0, 0, -0.1])
                safety_margin = 0.05 # Adjust safety margin here
                for hit in ray_results:
                    if hit[0] != -1:  # Collision detected
                        distance = np.linalg.norm(np.array(hit[3]) - np.array(pos))
                        if distance < safety_margin:  
                            return False
            return True
        
        def smooth_path(path, max_checks=10):
            smooth_path = [path[0]]  # Always include the start
            for i in range(len(path) - 2):
                start = path[i]
                end = path[i + 2]
                if is_collision_free(start, end):  # Skip intermediate points if straight line works
                    continue
                smooth_path.append(path[i + 1])
            smooth_path.append(path[-1])  # Always include the goal
            return smooth_path

        def clamp_bounds(
            node: np.ndarray,
            prev_bounds = np.array([[0.0, 0.0, 0.8], [0.0, 0.0, 0.8]]),
            radius = 0.1,
            workspace_bounds = np.array([[-0.4, -0.4, 0.6], [0.4, 0.65, 1.1]])
        ):
            bounds = np.array([
                node - np.array([radius, radius, radius]),
                node + np.array([radius, radius, radius])
            ])
            new_bounds = np.array([
                np.minimum(bounds[0], prev_bounds[0]),
                np.maximum(bounds[1], prev_bounds[1])
            ])
            return np.maximum(workspace_bounds[0], 
                              np.minimum(new_bounds, workspace_bounds[1])
            )

        def rrt(
            goal_pos,
            max_iterations = 50000,
            step_size = 0.07
        ):
            ee_pos = np.array(self.get_ee_pos())
            tree: list[Node] = [Node(ee_pos)]
            tree_bounds = clamp_bounds(ee_pos)
            path_found = False
            for _ in range(max_iterations):
                random_point = sample_random_point(tree_bounds)
                nearest_node = find_nearest(tree, random_point)

                direction = random_point - nearest_node.position
                direction = direction / np.linalg.norm(direction) * step_size
                new_position = nearest_node.position + direction

                if is_collision_free(nearest_node.position, new_position):
                    new_node = Node(new_position, parent=nearest_node)
                    tree.append(new_node)
                    tree_bounds = clamp_bounds(new_position, tree_bounds)

                    if np.linalg.norm(new_position - goal_pos) < step_size:
                        path_found = True
                        goal_node = Node(goal_pos, parent=new_node)
                        tree.append(goal_node)
                        break

            if not path_found:
                print("RRT failed to find a path!")
                return

            # Reconstruct path
            path = []
            current_node = tree[-1]
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            path.reverse()
            path = smooth_path(path)

            # Execute movement
            for waypoint in path:
                self.movep(waypoint)
                for _ in range(5):
                    self.step_sim_and_render()

            time.sleep(0.5)

        rrt(hover_start_xyz)
        rrt(start_xyz)

        self.gripper.activate()
        for _ in range(240):
            self.step_sim_and_render()

        rrt(hover_end_xyz)
        rrt(end_xyz)

        self.gripper.release()
        for _ in range(240):
            self.step_sim_and_render()

        default_pos = np.array(self.default_position)
        rrt(default_pos)
