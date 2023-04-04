from tree4tamp.tamp import *
import kdtree
from tree4tamp.planner.reachability_tree import *

def distance_ts(
    T1: Pose, 
    T2: Pose, 
    rot_weight=0.3
) -> float:
    linear = np.linalg.norm(T1.trans - T2.trans)
    angular = T1.rot.angle_between(T2.rot)
    return linear + rot_weight * angular

class RRTNode(Config):
    def __init__(self, q_dict):
        super().__init__(q_dict)
        self.T = {name:None for name in Config.robot_names}
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
        self.q_delta_max = 0.2
        self.ts_eps = 0.02,

    def rrt_connect(self, q_init:Config, q_goal:Config, mode:Mode):
        node_init = RRTNode.from_config(q_init)
        node_goal = RRTNode.from_config(q_goal)
        
        # first, check direct path
        path = self.get_direct_path(node_init, node_goal, mode, partial=False)
        if path is not None:    return path
        
        # else, rrt-connect
        tree1 = kdtree.create(dimensions=len(node_init))
        tree2 = kdtree.create(dimensions=len(node_goal))
        tree1.add(node_init)
        tree2.add(node_goal)

        tree_a, tree_b = tree1, tree2
        is_fwd = True
        for i in range(self.max_iter):
            node_rand = RRTNode.from_config(self.problem.sample_random_config())
            node_near_a = tree_a.search_nn(node_rand)[0].data
            path = self.get_direct_path(node_near_a, node_rand, mode, partial=True)
            if path is None: continue
            
            parent = node_near_a
            for node in path:
                tree_a.add(node)
                node.parent = parent
                parent = node
            
            node_a = deepcopy(path[-1])
            node_near_b = tree_b.search_nn(node_a)[0].data
            path = self.get_direct_path(node_near_b, node_a, mode, partial=True)
            if path is None: continue
            
            parent = node_near_b
            for node in path:
                tree_b.add(node)
                node.parent = parent
                parent = node
            
            if path[-1] == node_a: #connected
                node_fwd, node_bwd = node_a, path[-1]
                if not is_fwd:  node_fwd, node_bwd = node_bwd, node_fwd
                path_fwd = self.backtrack(node_fwd)
                path_bwd = self.backtrack(node_bwd)[::-1]
                path = [node.as_config() for node in (*path_fwd, *path_bwd)] 

                for config in path:
                    self.problem.geometry_assign(mode, config)
                return path
            tree_a, tree_b = tree_b, tree_a
            is_fwd = not is_fwd
        
        self.problem.geometry_assign(mode, node_init)
        self.problem.geometry_assign(mode, node_goal)
        return None

    def backtrack(self, node:RRTNode)->List[RRTNode]:
        path = [node]
        while True:
            if node.parent is None: break
            node = node.parent
            path.append(node)
        return path[::-1]
        
    def get_direct_path(self, node1:RRTNode, node2:RRTNode, mode:Mode, partial=False
    ) -> List[RRTNode]:
        q_diff = node2 - node1
        num_interp = np.ceil(np.linalg.norm(q_diff.as_array) / self.q_delta_max).astype(int)
        ratio = np.linspace(0,1, num_interp, endpoint=True)
        path = [node1 + q_diff*r for r in ratio][1:]
        i = 0
        for node in path:
            if self.problem.is_collision(mode, node):
                if not partial: return None
                break
            i += 1
        if i == 0:  return None
        return path[:i+1]
    
    def check_approach_traj(
        self,
        rt_node: RTNode,
        grasp_pose: Pose
    ):
        node_curr = RRTNode.from_config(rt_node.q)
        robot_name = rt_node.grasp_robot_name #list(rt_node.grasp_pose.keys())[0]
        pose_target = grasp_pose
        mode = rt_node.mode
        robot = self.problem.robots[robot_name]

        traj = []
        for _ in range(10):
            node_curr.T[robot_name] = robot.forward_kinematics(node_curr.q[robot_name])
            node_new = self.move_tspace(robot_name, node_curr, pose_target)
            #q_new = node_new.q[robot_name]
            if self.problem.is_collision(mode, node_new):
                return None
            node_new.T[robot_name] = robot.forward_kinematics(node_new.q[robot_name])
            traj.append(node_new)
            if distance_ts(node_new.T[robot_name], pose_target) < self.ts_eps:
                result = [node.as_config() for node in traj]
                return result
            node_curr = node_new
        return None

    def move_tspace(
        self, 
        robot_name: str,
        node_curr: RRTNode, 
        pose_target: Pose,
    ):
        damping = 0.1
        q_curr: np.ndarray = node_curr.q[robot_name]
        pose_curr: Pose = node_curr.T[robot_name]

        pos_err = pose_target.trans - pose_curr.trans
        orn_err = orn_error(pose_target.rot.as_quat(), pose_curr.rot.as_quat())
        err = np.hstack([pos_err, orn_err*2])

        jac = self.problem.robots[robot_name].get_jacobian(q_curr)
        lmbda = np.eye(6) * damping ** 2
        jac_pinv = jac.T @ np.linalg.inv(jac @ jac.T + lmbda)
        node_delta = deepcopy(node_curr)
        node_delta.q[robot_name] = jac_pinv @ err
        node_delta = self.limit_step_size(node_delta, q_delta_max=0.05)
        node_new = node_curr + node_delta
        return node_new
    
    def limit_step_size(self, node_delta: RRTNode, q_delta_max: Optional[np.ndarray]=None):
        if q_delta_max is None:
            q_delta_max = self.q_delta_max
        mag = np.linalg.norm(node_delta.as_array, np.inf) # node_delta.norm(np.inf) # np.linalg.norm(q_delta.q_numpy, np.inf)
        if mag > q_delta_max:
            node_delta = node_delta / mag * q_delta_max
        return node_delta