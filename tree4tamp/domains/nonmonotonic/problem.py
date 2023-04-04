from pathlib import Path

from pybullet_suite import *
from ...tamp import *


##############################
class ProblemNonmonotonic(TAMPProblem):
    def __init__(
        self, 
        gui, 
        num_box=3, 
    ):
        domain_pddl_path = Path(__file__).parent / Path("domain_abstract.pddl")
        self.num_box = num_box
        self.domain_name = "PICK-AND-PLACE"
        self.problem_name = f"NONMONOTONIC{num_box}"
        super().__init__(
            self.domain_name,
            self.problem_name,
            gui,
            domain_pddl_path=domain_pddl_path,
        )

    def set_task_scene(self):
        with self.world.no_rendering(True):
            sm = BulletSceneMaker(self.world)
            plane = sm.create_plane(z_offset=-0.4)

            # set environment
            self.envs = {}
            table_x_offset = 0.4
            h_table = 0.2
            self.envs["table"] = sm.create_table(
                "table", 0.4, 1.2, 0.2, 
                x_offset=table_x_offset, y_offset=0, z_offset=h_table)
            self.envs["ground"] = sm.create_table("ground", 2, 2, 0.4) #ground
            
            # regions spec
            w_plate, h_plate = 0.3, 0.02
            gap_plate = 0.32
            plate_names = ["plate_red", "plate_green", "plate_blue"]
            plate_half_extents = [w_plate/2, w_plate/2, h_plate/2]
            colors = [
                [1, 0, 0, 1],
                [0, 1, 0, 1],
                [0, 0, 1, 1]
            ]
            # movables spec
            w_movable, h_movable = 0.03, 0.07
            movable_half_extents = [w_movable/2, w_movable/2, h_movable/2]
            #gap = w_movable+0.02
            self.colorbox_names = ["red", "green", "blue"]
            x_offset = table_x_offset + 0.12

            w_obs, h_obs = 0.03, 0.15
            obs_half_extents = [w_obs/2, w_obs/2, h_obs/2]
            obs_names = [f"obs{i+1}" for i in range(self.num_box)]
            obs_color = [0.6, 0.6, 0.6, 1]

            if self.num_box == 2:
                plate_positions = [
                    [table_x_offset, +gap_plate/2, 0.2+0.01],
                    [table_x_offset, -gap_plate/2, 0.2+0.01],
                ]
                colorbox_positions = [
                    [x_offset, -gap_plate/2, h_table+h_plate+h_movable/2],
                    [x_offset, +gap_plate/2, h_table+h_plate+h_movable/2],
                ]
            elif self.num_box == 3:
                plate_positions = [
                    [table_x_offset, 0, 0.2+0.01],
                    [table_x_offset, +gap_plate, 0.2+0.01],
                    [table_x_offset, -gap_plate, 0.2+0.01]
                ]
                colorbox_positions = [
                    [x_offset, +gap_plate, 0.2+0.02+h_movable/2],
                    [x_offset, -gap_plate, 0.2+0.02+h_movable/2],
                    [x_offset, +0, 0.2+0.02+h_movable/2],
                ]
            else:
                raise NotImplementedError("num box")
            
            # set regions and movables
            self.regions, self.movables = {}, {}
            
            # set regions    
            for idx in range(self.num_box):
                plate_name = plate_names[idx]
                colorblock_name = self.colorbox_names[idx]
                obs_name = obs_names[idx]
                obs_position = deepcopy(colorbox_positions[idx])
                obs_position[0] -= 0.04
                obs_position[-1] = h_table+h_plate+h_obs/2

                self.regions[plate_name] = sm.create_box(
                    body_name=plate_name, 
                    half_extents=plate_half_extents, 
                    mass=1., 
                    pose=Pose(trans=plate_positions[idx]),
                    rgba_color=colors[idx])
                self.movables[colorblock_name] = sm.create_box(
                    body_name=colorblock_name, 
                    half_extents=movable_half_extents, 
                    mass=1., 
                    pose=Pose(trans=colorbox_positions[idx]),
                    rgba_color=colors[idx])
                self.movables[obs_name] = sm.create_box(
                    body_name=obs_name, 
                    half_extents=obs_half_extents, 
                    mass=1., 
                    pose=Pose(trans=obs_position),
                    rgba_color=obs_color)
            
            # set robot
            self.robots = {"robot":self.world.load_robot(name="robot", robot_class=Panda)}
            self.object_types = {
                "robot": ["robot"],
                "region": [*list(self.regions.keys())],
                "movable": [*list(self.movables.keys())],
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
        hand_clean = ("handempty",)
        attached = []
        for movable_name, movable in self.movables.items():
            for placeable_name, placeable in self.placeables.items():
                if self.is_placed(movable, placeable):
                    attached.append(("attached", movable_name, placeable_name))
        # tuple: abstract variable, att: mode variable 
        abs_var_init = [
            hand_clean,
            *attached
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

        # GOAL
        goal = []
        for movable_name in self.movables.keys():
            if movable_name in self.colorbox_names:
                goal.append(("attached", f"{movable_name}", f"plate_{movable_name}"))
        abs_var_goal = [ #and
            *goal,
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
    prob = ProblemNonmonotonic(gui=True, num_box=2)
    