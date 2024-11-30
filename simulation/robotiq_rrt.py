import time
import numpy as np
import pybullet

from simulation.robotiq import SimulationEnv


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

        def sample_random_point(bounds):
            return np.random.uniform(low=bounds[0], high=bounds[1])

        def find_nearest(tree, point):
            # TODO: make this faster than O(n)
            return min(tree, key=lambda node: np.linalg.norm(node.position - point))

        def is_collision_free(p1, p2):
            # ray_result = pybullet.rayTest(p1, p2)
            # return all(hit[0] == -1 for hit in ray_result)
            steps = int(np.linalg.norm(p2 - p1) / 0.1)  # Break path into small steps
            for step in range(steps):
                pos = p1 + step * (p2 - p1) / steps
                if pybullet.rayTest(pos, pos + [0, 0, -0.1])[0][0] != -1:  # Ray-test for collisions
                    return False
            return True

        def clamp_bounds(
            node: np.ndarray,
            prev_bounds = np.array([[0.0, 0.0, 0.8], [0.0, 0.0, 0.8]]),
            radius = 0.1,
            workspace_bounds = np.array([[-0.4, -0.4, 0.6], [0.4, 0.4, 1.1]])
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
            step_size = 0.05
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

            # Reconstruct the path
            path = []
            current_node = tree[-1]
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            path.reverse()

            # Execute the path
            for waypoint in path:
                self.movep(waypoint, 1000.0)
                for _ in range(5):
                    self.step_sim_and_render()

            time.sleep(0.5)

        goal_pos = np.array(start_xyz)
        rrt(goal_pos)

        # Gripper picks up the object
        self.gripper.activate()
        for _ in range(240):
            self.step_sim_and_render()

        # Move to end pos
        goal_pos = np.array(end_xyz)
        rrt(goal_pos)

        # Release the object
        self.gripper.release()
        for _ in range(240):
            self.step_sim_and_render()

        # Move back to the default position
        default_pos = np.array(self.default_position)
        rrt(default_pos)
