from pathlib import Path

from pybullet_suite import *
from ...tamp import *


##############################
class ProblemKitchen(TAMPProblem):
    def __init__(
        self, 
        gui, 
        num_box=3, 
        goal_box_list=None,
    ):
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
            domain_pddl_path=domain_pddl_path,
        )
        
    def set_task_scene(self):
        with self.world.no_rendering():
            sm = BulletSceneMaker(self.world)
            plane = sm.create_plane(z_offset=-0.4)

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
            half_extents = [
                [w/2, w/2, h/2],
                [w/2, w/2, h/2],
                [w/2, w/2, h/2],
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
        hand_clean = ("handempty",)
        attached_dish = [("attached", box, "dish1") for box in foods]
        # tuple: abstract variable, att: mode variable 
        abs_var_init = [
            hand_clean,
            *attached_dish
        ]
        atts_init = self.get_attachments_from_scene()
        q_init = self.get_config()
        self.init = abs_var_init + atts_init + [q_init]
        
        init_atts = []
        for var in self.init:
            if isinstance(var, Attachment):
                init_atts.append(var)
            if isinstance(var, Config):
                self.q_init = var
        assert set(self.movables.keys()) == set([att.obj_name for att in init_atts])
        self.mode_init = Mode.from_list(init_atts)

        cooked = []
        for box_num in self.goal_box_list:
            assert f"box{box_num}" in foods
            cooked.append(("cooked", f"box{box_num}"))
        abs_var_goal = [ #and
            *cooked,
            *hand_clean
        ]
        self.atts_goal = {}
        atts_goal_list = list(self.atts_goal.keys())
        self.q_goal = deepcopy(q_init)
        self.goal = abs_var_goal + atts_goal_list + [self.q_goal]
        self.atts_goal_dict = {att.obj_name:att for att in self.atts_goal}


    def get_attachments_from_scene(self):
        # mode_init: all movables are placed to the dish
        #parent_obj = "dish1"    #TODO: automatically detect the attachment between objects
        att_list = []
        for placeable_name, placeable in self.placeables.items():
            for movable_name, movable in self.movables.items():
                if self.is_placed(movable, placeable): #TODO: automatically detect the grasp
                    sop = movable.sops[0] # TODO: calculate sop from scene
                    placement = self.calculate_current_placement_from_scene(movable_name, placeable_name, sop)
                    att_list.append(placement)
        return att_list
        


if __name__ == "__main__":
    pass
    