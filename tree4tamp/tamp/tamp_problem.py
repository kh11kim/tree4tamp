from pybullet_suite import *
from copy import deepcopy
from itertools import product
from .tamp_object import *
from .tamp_action import *
sys.path.append('./pyperplan/')
from pyperplan.pddl.parser import Parser
from pyperplan.planner import _ground

class TAMPProblem:
    """TAMP world interface"""
    def __init__(
        self,
        domain_name,
        prob_name,
        gui,
        domain_pddl_path,
    ):
        self.domain_name = domain_name
        self.prob_name = prob_name
        self.movables: Dict[str, Movable] = {}
        self.regions: Dict[str, Region] = {}
        self.robots: Dict[str, Robot] = {}
        self.envs: Dict[str, Body] = {}
        self.q_goal = None
        self.atts_goal: Dict[str, Attachment] = {}

        # geometry world
        self.world = BulletWorld(gui=gui)
        self.sm = BulletSceneMaker(self.world)
        # self.hand = Gripper(self.world, self.sm)

        # init
        self.set_task_scene()
        self.set_tamp_object_config(self.movables, self.regions, self.robots)
        self.set_init_goal()
        #self.set_init_geometry_from_scene()
        self.set_task_domain_problem(domain_pddl_path)

        Config.set_robot_names(list(self.robots.keys()))
        Mode.set_movable_names(list(self.movables.keys()))
        
        #self.set_objects()

        # #self.checker = Checker(not gui)
        
        # self.init_state = None
        #self.init_state = self.save()
        # self.set_tamp_objects(movables, regions, robots)
                
        # with open(self.domain_pddl_path, "r") as f:
        #     self.domain_pddl_string = f.read()

    @property
    def objects(self)->Dict[str, TAMPObject]:
        return {**self.movables, **self.regions, **self.robots}

    @property
    def fixed(self):
        return {**self.envs, **self.regions}
    
    @property
    def all_instances(self):
        return {**self.objects, **self.envs}

    # Initialization
    def set_task_scene(self):
        # this should save self.init_state
        # this should output movables, regions, robots
        #raise NotImplementedError()
        pass
    
    def set_tamp_object_config(self):
        #raise NotImplementedError()
        pass

    def set_init_goal(self):
        self.init = None
        self.goal = None
        #raise NotImplementedError()
        pass
    
    def set_init_geometry_from_scene(self):
        pass

    def set_task_domain_problem(self, domain_pddl_path):
        # abstract world
        parser = Parser(domain_pddl_path)
        parser.probInput = self.generate_pddl_problem_as_string()
        self.pddl_domain = parser.parse_domain()
        self.pddl_problem = parser.parse_problem(self.pddl_domain, read_from_file=False)
        self.task = _ground(self.pddl_problem)
        self.abs_state_init = deepcopy(self.task.initial_state)
    
    #------------------------------------------
    # Geometry Assignment and Collision Check
    #------------------------------------------
    def get_config(self):
        q = {}
        for robot_name, robot in self.robots.items():
            q[robot_name] = robot.get_joint_angles()
        return Config(q)

    def set_config(self, config: Config):
        for robot in config.q.keys():
            q = config.q[robot]
            self.robots[robot].set_joint_angles(q)
    
    def assign_obj(
        self,
        obj: str, 
        parents:Dict[str, str], 
        atts: Dict[str, Attachment],
        assigned: List[str],
    ):
        if obj in assigned:
            return self.objects[obj].get_base_pose()
        parent_name = parents[obj]
        parent_type = type(self.objects[parent_name])
        if parent_type is Panda:
            parent_pose = self.robots[parent_name].get_ee_pose()
        elif parent_type is Region:
            parent_pose = self.regions[parent_name].get_base_pose()
        elif parent_type is Movable:
            parent_pose = self.assign_obj(parent_name, parents, atts, assigned)
        
        obj_pose = parent_pose * atts[obj].tf.inverse()
        self.movables[obj].set_base_pose(obj_pose)
        assigned.append(obj)
        return obj_pose

    def mode_assign(self, mode:Mode):
        def has_cycle(parents):
            def dfs(node):
                if node in visited: return True
                
                if node not in parents.keys(): return False
                visited.add(node)
                parent = parents[node]
                # if parent in visited:
                #     return True
                if parent is not None: return dfs(parent)
                return False

            for node in parents.keys():
                visited = set()
                if node not in visited:
                    if dfs(node):
                        return True
            return False
        
        # make graph and check cycle.
        parents = {}
        for obj, att in mode.atts.items():
            parents[obj] = att.parent_name
        if has_cycle(parents):
            raise ValueError("A cycle is detected in kinematic tree..!")
        
        # assign the mode
        assigned = []
        for obj, att in mode.atts.items():
            if obj not in assigned:
                self.assign_obj(obj, parents, mode.atts, assigned)

    def geometry_assign(self, mode:Mode, config:Optional[Config]=None):
        self.set_config(config)
        self.mode_assign(mode)
        
        # assign the gripper width
        is_grasp = False
        for att in mode.atts.values():
            if isinstance(att, Grasp):
                self.robots[att.parent_name].open(att.width)
                is_grasp = True
            
        if is_grasp is False:
            for robot in self.robots.values():
                robot.open()
    
    @contextmanager
    def no_assign(self):
        with self.world.no_rendering():
            configs = {robot_name:robot.get_joint_angles() for robot_name, robot in self.robots.items()}
            movable_poses = {movable_name: movable.get_base_pose() for movable_name, movable in self.movables.items()}
            yield
            for robot_name, joints in configs.items():
                self.robots[robot_name].set_joint_angles(joints)
            for movable_name, pose in movable_poses.items():
                self.movables[movable_name].set_base_pose(pose)

    def is_collision(
        self, mode:Mode, config: Config,  #ignore_movables=False  #, only_fixed=False
    ):
        self.geometry_assign(mode, config)

        #self collision
        for robot in self.robots:
            if self.world.is_self_collision(robot):
                return True
        
        #collision between robots
        for robot1, robot2 in combinations(self.robots, 2):
            if self.world.is_body_pairwise_collision(
                body=robot1, obstacles=[robot2]
            ):
                return True
        
        # robot-fixed(region+env)
        for robot, fixed in product(self.robots, self.fixed):
            for link in range(1, len(self.robots[robot].info)): #except base
                if self.world.is_link_pairwise_collision(
                        body1=robot, body2=fixed, link1=link, link2 =-1):
                    return True
        
        # if only_fixed: return False

        # robot-movable
        movables = list(mode.atts.keys())
        for robot, obs in product(self.robots, movables):
            if robot == mode.atts[obs].parent_name: continue
            if self.world.is_body_pairwise_collision(
                body=robot, obstacles=[obs]):
                return True
        
        # if ignore_movables: return False
        
        #movable-movable collision
        for movable1, movable2 in product(movables, movables):
            if movable1 == movable2: continue
            if movable2 == mode.atts[movable1].parent_name: continue
            if movable1 == mode.atts[movable2].parent_name: continue
            if self.world.is_body_pairwise_collision(
                body=movable1, obstacles=[movable2]):
                return True
        return False

    def is_placed(self, movable:Movable, region: Region):
        if movable == region: False
        eps = 0.01
        lower1, upper1 = movable.get_AABB()
        lower2, upper2 = region.get_AABB()
        center = movable.get_base_pose().trans
        is_z_contact = upper2[2] - eps <= lower1[2] <= upper2[2] + eps
        is_in_area = np.less_equal(lower2[:2], center[:2]).all() and \
                    np.less_equal(center[:2], upper2[:2]).all()
        return is_z_contact and is_in_area


    #------------------------------------------
    # Geometry Calculation and Sampling
    #------------------------------------------
    def sample_attachment(self, obj_name: str, parent_name: str):
        def sample_grasp(obj_name: str, parent_name: str):
            movable: Movable = self.objects[obj_name]
            grasp = movable.sample_grasp()
            grasp.parent_name = parent_name
            return grasp
        def sample_placement(obj_name: str, parent_name: str, yaw=None):
            movable: Movable = self.objects[obj_name]
            placeable: TAMPObject = self.objects[parent_name]
            sssp = placeable.sssp
            point = np.random.uniform(sssp.lower, sssp.upper)
            point = placeable.get_base_pose().transform_point(point)
            sop = movable.sample_sop()
            placement = Placement.from_point_and_sop(
                obj_name, parent_name, 
                placeable.get_base_pose(),
                point,sop,
                yaw=yaw)
            return placement

        if "robot" in parent_name:
            return sample_grasp(obj_name, parent_name)
        else:
            return sample_placement(obj_name, parent_name)
    
    def sample_attachment_by_action(self, action:Operator):
        return self.sample_attachment(action.get_target_movable(), action.get_parent_to())
    
    def sample_random_config(self):
        q_dict = {}
        for robot_name in self.robots.keys():
            ll = self.robots[robot_name].joint_lower_limit
            ul = self.robots[robot_name].joint_upper_limit
            q_dict[robot_name] = np.random.uniform(ll, ul)
        return Config(q_dict)

    def calculate_current_placement_from_scene(self, obj_name: str, parent_name: str, sop:SOP):
        movable: Movable = self.objects[obj_name]
        placeable: TAMPObject = self.objects[parent_name]
        
        point = movable.get_base_pose().trans
        z = placeable.get_base_pose().trans[-1] + placeable.sssp.upper[-1]
        point[-1] = z
        placement = Placement.from_point_and_sop(
            obj_name, parent_name, 
            placeable.get_base_pose(),
            point, sop, yaw=0
        )
        #placement.assigned_tf = placeable.get_base_pose() * placement.tf.inverse()
        return placement

    def sample_transition(self, mode:Mode, q:Config, mode_new:Mode):
        q_pre = deepcopy(q)
        q_grasp = deepcopy(q)
        for m in self.movables.keys():
            if mode.atts[m].att_type != mode_new.atts[m].att_type: #TODO: considering handover
                if isinstance(mode.atts[m], Grasp):
                    grasp, placement = mode.atts[m], mode_new.atts[m]
                else:
                    placement, grasp = mode.atts[m], mode_new.atts[m]
        q_pre_ik, q_ik = self.get_ik(grasp, placement, grasp.parent_name, mode)
        q_pre.set_joints(grasp.parent_name, q_pre_ik)
        q_grasp.set_joints(grasp.parent_name, q_ik)
        return q_pre, q_grasp, grasp.parent_name

    def get_ik(
        self, grasp: Grasp, placement: Placement, robot_name:str, mode:Mode, 
        robot_base_pose: Pose=Pose.identity()
    ) -> bool:
        self.mode_assign(mode)
        parent_pose = self.objects[placement.parent_name].get_base_pose()
        obj_pose = parent_pose * placement.tf.inverse()
        ee_grasp_pose = obj_pose * grasp.tf # wrt world
        
        grasp_pose = robot_base_pose.inverse() * ee_grasp_pose #wrt robot
        grasp_pose_pre = grasp.get_pre_pose(grasp_pose)
        
        robot = self.robots[robot_name]
        q_pre_ik = robot.inverse_kinematics(pose=grasp_pose_pre)
        if q_pre_ik is None: return None
        q_ik = robot.inverse_kinematics(pose=grasp_pose)
        if q_ik is None: return None
        return q_pre_ik, q_ik
    
    def generate_pddl_problem_as_string(self):
        def addline(pddl, fact):
            if isinstance(fact, str):
                pddl += f"    ({fact})\n"
            if isinstance(fact, tuple):
                pddl += f"    ({' '.join(fact)})\n"
            return pddl
        
        # Generate the PDDL problem file as a string
        pddl = f"(define (problem {self.domain_name}_PROB1)\n"
        pddl += f"  (:domain {self.domain_name})\n"
        pddl += "  (:objects\n"
        for obj_type, obj in self.object_types.items():
            objs_string = ' '.join(obj)
            pddl += f"    {objs_string} - {obj_type}\n"
        pddl += "  )\n"
        pddl += "  (:init\n"
        for fact in self.init:
            pddl = addline(pddl, fact) #pddl +=     #f"    {fact}\n"
        pddl += "  )\n"
        pddl += "  (:goal  (and\n"
        for fact in self.goal:
            pddl = addline(pddl, fact) #pddl += f"    {fact}\n"
        pddl += "  ))\n"
        pddl += ")\n"
        return pddl

    # def is_collision_between_pgp(
    #     self, p1:Placement, g1:Grasp, p2:Placement):
    #     # def get_obj_pose_by_placement(p:Placement):
    #     #     parent_name = p.parent_name
    #     #     assert parent_name in self.regions
    #     #     parent_pose = self.regions[parent_name].get_base_pose()
    #     #     return parent_pose * p.tf.inverse()
    #     obj1 = self.movables[p1.obj_name]
    #     obj2 = self.movables[p2.obj_name]
    #     # obj_pose1 = get_obj_pose_by_placement(p1)
    #     # obj_pose2 = get_obj_pose_by_placement(p2)
    #     grasp_pose = p1.obj_pose*g1.tf
    #     obj1.set_base_pose(p1.obj_pose)
    #     obj2.set_base_pose(p2.obj_pose)
    #     self.hand.reset(grasp_pose)
    #     if self.world.is_body_pairwise_collision(
    #         body="hand", obstacles=[obj2]):
    #         self.hand.remove()
    #         return True
    #     self.hand.remove()
    #     return False

    # def is_collision_between_two_placement(
    #     self, p1:Placement, p2:Placement, robot_name=None):
    #     # def get_obj_pose_by_placement(p:Placement):
    #     #     parent_name = p.parent_name
    #     #     assert parent_name in self.regions
    #     #     if parent_name in self.regions:
    #     #         parent_pose = self.regions[parent_name].get_base_pose()
    #     #     else:
    #     #         parent_pose = get_obj_pose_by_placement()
    #     #     return parent_pose * p.tf.inverse()
        
    #     obj1 = self.movables[p1.obj_name]
    #     obj2 = self.movables[p2.obj_name]
    #     obj_pose1 = p1.obj_pose
    #     obj_pose2 = p2.obj_pose
    #     obj1.set_base_pose(obj_pose1)
    #     obj2.set_base_pose(obj_pose2)

    #     if self.world.is_body_pairwise_collision(
    #         body=obj1, obstacles=[obj2]):
    #         return True
    #     if robot_name is not None:
    #         robot = self.robots[robot_name]
    #         for obj in [obj1, obj2]:
    #             if self.world.is_body_pairwise_collision(
    #                 body=robot, obstacles=[obj]):
    #                 return True
    #     return False

    # def heuristic_distance(self, att1:Attachment, att2:Attachment):
    #     if isinstance(att1, Grasp) and isinstance(att2, Grasp):
    #         p1 = (att1.tf * Pose(trans=[0,0,-1])).trans
    #         p2 = (att2.tf * Pose(trans=[0,0,-1])).trans
    #         return np.linalg.norm(p2 - p1)/2
    #     elif isinstance(att1, Placement) and isinstance(att2, Placement):
    #         support = self.objects[att1.parent_name]
    #         delta = att2.point - att1.point
    #         sssp = support.sssp
    #         x_scale, y_scale = (sssp.upper - sssp.lower)[:2]
    #         return np.linalg.norm([delta[0]/x_scale, delta[1]/y_scale])
    
    

# class TAMPProblem:
    # def __init__(self, prob_name:str, scene: TAMPDomain):
    #     self.scene = scene
    #     self.prob_name = prob_name
    #     self.set_objects()
    #     self.set_init_mode_config()
    #     self.set_init_goal()

    
    
    # def set_init_mode_config(self):
    #     # set initial mode of the problem
    #     self.init_mode = None
    #     raise NotImplementedError()
    
    # def set_objects(self):
    #     self.objects = None
    #     raise NotImplementedError()
    


    # def set_init_mode_config(self):
    #     raise NotImplementedError()

    

    # def get_prob_pddl(self, shuffle=False):
    #     n = "\n"
    #     object_dict = deepcopy(self.objects)
    #     init = deepcopy(self.init)
    #     goal = deepcopy(self.goal)
    #     if shuffle:
    #         for obj_list in object_dict.values():
    #             np.random.shuffle(obj_list)
    #         np.random.shuffle(init)
    #         np.random.shuffle(goal)
            
    #     obj_string = "\n".join([" ".join(objs) + f" - {t}" for t, objs in object_dict.items()])
    #     init_string = "\n".join(["("+" ".join(pred)+")" if type(pred) is tuple else "("+pred+")" for pred in init])
    #     goal_string = "\n".join(["("+" ".join(pred)+")" if type(pred) is tuple else "("+pred+")" for pred in goal])
    #     pddl_string = f"(define {n}(problem {self.prob_name}){n}(:domain {self.scene.domain_name}){n}(:objects {n}{obj_string}){n}(:init {n}{init_string}){n}(:goal {n}(and {n}{goal_string})))"
    #     return pddl_string
    
    # def make_temp_pddl_files(self, path="./temp_pddl", shuffle=False):
    #     if os.path.exists(path):
    #         shutil.rmtree(path)
    #     os.mkdir(path)
    #     domain_file_path = path + "/domain.pddl"
    #     problem_file_path = path + "/problem.pddl"
    #     with open(domain_file_path, "w") as f:
    #         f.write(self.domain.domain_pddl_string)
    #     with open(path+"/problem.pddl", "w") as f:
    #         f.write(self.get_prob_pddl(shuffle=shuffle))
    #     return domain_file_path, problem_file_path

    