from tree4tamp.tamp import *
import kdtree
from tree4tamp.planner.reachability_tree import *

class RRTNode(Config):
    def __init__(self, q_dict):
        super().__init__(q_dict)
        self.parent = None

    @classmethod
    def from_config(cls, q:Config):
        return cls(q.q)
    
    def as_config(self):
        return Config(self.q)

    @property
    def as_array(self):
        return np.hstack([self.q[robot_name] for robot_name in self.robot_names])

    def __getitem__(self, i):
        return self.as_array[i]
    
    def __add__(self, other):
        result = deepcopy(self)
        for robot_name in self.robot_names:
            result.q[robot_name] += other.q[robot_name]
        return result

    def __sub__(self, other)->"RRTNode":
        result = deepcopy(self)
        for robot_name in self.robot_names:
            result.q[robot_name] -= other.q[robot_name]
        return result

    def __mul__(self, other: float):
        result = deepcopy(self)
        for robot_name in self.q:
            result.q[robot_name] *= other
        return result

    def __truediv__(self, other: float):
        result = deepcopy(self)
        for robot_name in self.q:
            result.q[robot_name] /= other
        return result
    
    def __eq__(self, other):
        return np.array_equal(self.as_array, other.as_array)

    def __len__(self):
        return len(self.as_array)

class MotionPlanner:
    def __init__(self, problem:TAMPProblem, max_iter=100):
        self.problem = problem
        self.max_iter = max_iter
        self.q_delta_max = 0.1

    def connect(self, state:RTNode, state_new:RTNode, rt:RT):
        path = self.rrt_connect(state.q, state_new.q, state.mode)
        if path:
            rt.add_edge(state, state_new, a, path, state.mode)

    def rrt_connect(self, q_init:Config, q_goal:Config, mode:Mode):
        node_init = RRTNode.from_config(q_init)
        node_goal = RRTNode.from_config(q_goal)

        if self.problem.is_collision(mode, node_init): return None
        if self.problem.is_collision(mode, node_goal): return None
        
        # first, check direct path
        path = self.get_direct_path(node_init, node_goal, mode, partial=False)
        if path is not None:    return path
        
        # else, rrt-connect
        tree1 = kdtree.create(dimensions=len(node_init))
        tree2 = kdtree.create(dimensions=len(node_goal))

        tree_a, tree_b = tree1, tree2
        is_fwd = True
        for i in range(self.max_iter):
            node_rand = RRTNode.from_config(self.problem.sample_random_config())
            node_near_a = tree_a.search_nn(node_rand)[0].data
            path = self.get_direct_path(node_near_a, node_rand, mode, partial=True)
            for node in path:
                tree_a.add(node)
            
            node_a = deepcopy(path[-1])
            node_near_b = tree_b.search_nn(node_a)[0].data
            path = self.get_direct_path(node_near_b, node_a)
            for node in path:
                tree_b.add(node)
            if path[-1] == node_a: #connected
                node_fwd, node_bwd = node_a, path[-1]
                if not is_fwd:  node_fwd, node_bwd = node_bwd, node_fwd
                path_fwd = self.backtrack(node_fwd)
                path_bwd = self.backtrack(node_bwd).reverse()
                return [node.as_config() for node in (*path_fwd, *path_bwd)] 
            tree_a, tree_b = tree_b, tree_a
            is_fwd = not is_fwd
        return None

    def backtrack(self, node:RRTNode)->List[RRTNode]:
        path = [node]
        while True:
            if node.parent is None: break
            node = node.parent
            path.append(node)
        return path.reverse()
        
    def get_direct_path(self, node1:RRTNode, node2:RRTNode, mode:Mode, partial=False):
        q_diff = node2 - node1
        num_interp = np.ceil(np.linalg.norm(q_diff.as_array) / self.q_delta_max).astype(int)
        ratio = np.linspace(0,1, num_interp, endpoint=True)
        path = [node1 + q_diff*r for r in ratio]
        i = 0
        for i, node in enumerate(path):
            if self.problem.is_collision(mode, node):
                if not partial: return None
                break
        return path[:i]

