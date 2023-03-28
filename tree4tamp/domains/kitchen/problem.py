from pathlib import Path

from pybullet_suite import *
from ...tamp import *


##############################
class ProblemKitchen(TAMPProblem):
    def __init__(self, gui, num_box=3, goal_box_list=None):
        domain_pddl_path = Path(__file__).parent / Path("domain_abstract.pddl")
        self.num_box = num_box
        if goal_box_list== "all":
            self.goal_box_list = [*range(1, num_box+1)]
        else:
            self.goal_box_list = goal_box_list
        self.domain_name = "KITCHEN"
        self.problem_name = f"KITCHEN{num_box}"
        super().__init__(
            self.domain_name,
            self.problem_name,
            gui,
            domain_pddl_path=domain_pddl_path
        )
                
        # not mandatory
        # self.action_info = {
        #     "geometric":["pick", "place"],
        #     "non-geometric":["wash", "cook"]
        # }
        # self.region_types = ["sink", "oven", "dish"]
        # self.movable_types = ["food"]
        # self.predicates = ["on", "handempty", "holding", "clear", "cleaned", "coocked"]

    def set_task_scene(self):
        with self.world.no_rendering():
            sm = BulletSceneMaker(self.world)
            plane = sm.create_plane(z_offset=-0.4)
            #hand = Gripper(self.world)

            # set environment
            self.envs = {}
            self.envs["table"] = sm.create_table(
                "table", 0.4, 1.2, 0.2, 
                x_offset=0.4, y_offset=0, z_offset=0.2)
            self.envs["ground"] = sm.create_table("ground", 2, 2, 0.4) #ground
            
            # set regions
            self.regions = {}
            names = ["dish1", "sink1", "oven1"]
            w, h = 0.22, 0.02
            w1 = w
            half_extents = [
                [w/2, w/2, h/2],
                [w1/2, w1/2, h/2],
                [w1/2, w1/2, h/2],
            ]
            positions = [
                [0.4, 0, 0.2+0.01],
                [0.4, +0.3, 0.2+0.01],
                [0.4, -0.3, 0.2+0.01]
            ]
            colors = [
                [0.6, 0.6, 0.6, 1],
                [0, 0, 1, 1],
                [1, 0, 0, 1]
            ]
            for idx, name in enumerate(names):
                self.regions[name] = sm.create_box(
                    body_name=name, 
                    half_extents=half_extents[idx], 
                    mass=1., 
                    pose=Pose(trans=positions[idx]),
                    rgba_color=colors[idx])
            
            # set movables
            self.movables = {}
            w, h = 0.06, 0.07
            half_extents = [w/2, w/2, h/2]
            gap = w+0.035
            names = [f"box{i+1}" for i in range(self.num_box)]
            positions = [
                [0.4, 0, 0.2+0.02+0.05],
                [0.4, -gap, 0.2+0.02+0.05],
                [0.4, +gap, 0.2+0.02+0.05],
                [0.4-gap, 0, 0.2+0.02+0.05],
                [0.4+gap, 0, 0.2+0.02+0.05]
            ]
            colors = [
                [1, 0, 0, 1],
                [0, 1, 0, 1],
                [0, 0, 1, 1],
                [0.7, 0.7, 0, 1],
                [0.7, 0, 0.7, 1],
            ]
            for idx, name in enumerate(names[:self.num_box]):
                self.movables[name] = sm.create_box(
                    body_name=name, 
                    half_extents=half_extents, 
                    mass=1., 
                    pose=Pose(trans=positions[idx]),
                    rgba_color=colors[idx])

            # set robot
            self.robots = {"robot":self.world.load_robot(name="robot", robot_class=Panda)}
            self.object_types = {
                "robot": ["robot"],
                "sink": ["sink1"],
                "dish": ["dish1"],
                "oven": ["oven1"],
                "food": list(self.movables.keys()),
            }            
        self.world.set_view(eye_point=[1.2,-0.2,0.7])
        self.world.wait_for_rest()

    def set_tamp_object_config(self, movables:Dict, regions:Dict, robots:Dict):
        def get_top_grasps(body: Body):
            max_grasp_width = 0.15
            grasp_depth = 0.015
            _, (w, l, h) = body.get_AABB_wrt_obj_frame()
            rot = Rotation.from_euler("xyz",[np.pi,0,0])
            trans = [0,0,h/2-grasp_depth]
            grasp_poses = []
            grasp_widths = []
            if w <= max_grasp_width:
                grasp_poses += [
                    Pose(rot=rot, trans=trans),
                    Pose(rot=Rotation.from_euler("xyz",[0,0,np.pi])*rot,trans=trans)
                ]
                grasp_widths += [w, w]
            if l <= max_grasp_width:
                grasp_poses += [
                    Pose(rot=Rotation.from_euler("xyz",[0,0,np.pi/2])*rot,trans=trans),
                    Pose(rot=Rotation.from_euler("xyz",[0,0,np.pi*3/2])*rot,trans=trans)
                ]
                grasp_widths += [l, l]
            grasps = [Grasp(name, tf=tf, width=width) 
                for tf, width in zip(grasp_poses, grasp_widths)]
            return grasps
        def get_sops(body: Body):
            _, (w, l, h) = body.get_AABB_wrt_obj_frame()
            sop = SOP(Rotation.identity(), np.array([0,0,1]), h/2)
            sops = [sop]
            return sops
        def get_sssp(body: Body):
            lower, upper = body.get_AABB_wrt_obj_frame(output_center_extent=False)
            lower[-1] = upper[-1]
            return SSSP(lower, upper)

        self.movables = {}
        for name, body in movables.items():
            grasps = get_top_grasps(body)
            sops = get_sops(body)
            sssp = get_sssp(body)
            self.movables[name] = Movable.from_body(body, name, sops, sssp)
            self.movables[name].set_grasps(grasps)
        
        self.regions = {}
        for name, body in regions.items():
            sssp = get_sssp(body)
            self.regions[name] = Region.from_body(body, name, sssp)
        
        self.placeables = self.regions
        
        self.robots = {}
        for name, robot in robots.items():
            self.robots[name] = robot

    def set_init_goal(self):
        # set by parent class
        foods = list(self.movables.keys())
        #hand_clean = [("clear", box) for box in foods]
        hand_clean = [("handempty")]
        attached_dish = [("attached", box, "dish1") for box in foods]
        self.init = [
            *hand_clean,
            *attached_dish
        ]
        # if self.goal_box_list is None:
        #     #all cooked
        #     cooked = [("cooked", box) for box in foods]
        # else:
        cooked = []
        for box_num in self.goal_box_list:
            assert f"box{box_num}" in foods
            cooked.append(("cooked", f"box{box_num}"))
        self.goal = [ #and
            *cooked,
            *hand_clean
        ]
        self.mode_goal = {}

    def set_init_geometry_from_scene(self):
        # mode_init: all movables are placed to the dish
        parent_obj = "dish1"    #TODO: automatically detect the attachment between objects
        att_list = []
        for movable_name, movable in self.movables.items():
            sop = movable.sops[0]
            placement = self.get_current_placement_from_scene(movable_name, parent_obj, sop)
            att_list.append(placement)
        self.mode_init = Mode.from_list(att_list)
        
        # q_init: robot joint home position
        q = {}
        for robot_name, robot in self.robots.items():
            q[robot_name] = robot.get_joint_angles()
        self.q_init = Config(q)
        #self.geometry_assign(self.mode_init, self.config_init)

# class ProblemKitchen(TAMPProblem):
#     def __init__(self, domain: DomainKitchen, cooked_list=None):
#         self.num_block = domain.num_box
#         if cooked_list == "all":
#             self.cooked_list = [*domain.movables.keys()]
#         else:
#             assert type(cooked_list) == list
#             self.cooked_list = cooked_list
#         super().__init__(
#             prob_name="kitchen_prob1",
#             scene=domain
#         )
        
    
#     def set_objects(self):
#         blocks = [f"box{i+1}" for i in range(self.num_block)]
#         # set by parent class
#         self.object_dict = {
#             "sink1":"sink",
#             "dish1":"dish",
#             "oven1":"oven"
#         }
#         self.object_dict.update({name:"food" for name in blocks})
    
#     # def set_init_goal(self):
#     #     # set by parent class
#     #     foods = [name for name, obj_type in self.object_dict.items()\
#     #              if obj_type == "food"]
#     #     hand_clean = [("clear", box) for box in foods]
#     #     hand_clean += [("handempty")]
#     #     on_dish = [("on", box, "dish1") for box in foods]
#     #     self.init = [
#     #         *hand_clean,
#     #         *on_dish
#     #     ]
#     #     if self.cooked_list is None:
#     #         #all cooked
#     #         cooked = [("cooked", box) for box in foods]
#     #     else:
#     #         cooked = []
#     #         for cooked_box in self.cooked_list:
#     #             assert cooked_box in foods
#     #             cooked.append(("cooked", cooked_box))
#     #     self.goal = [ #and
#     #         *cooked,
#     #         *hand_clean
#     #     ]
#     #     self.mode_goal = {}



if __name__ == "__main__":
    pass
    