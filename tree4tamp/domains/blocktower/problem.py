from pathlib import Path

from pybullet_suite import *
from ...tamp import *


##############################
class ProblemBlocktower(TAMPProblem):
    def __init__(
        self, 
        gui, 
        num_box=5, 
    ):
        domain_pddl_path = Path(__file__).parent / Path("domain_abstract.pddl")
        self.num_box = num_box
        self.domain_name = "PICK-PLACE-STACK-UNSTACK"
        self.problem_name = f"BLOCKTOWER{num_box}"
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
            plate_names = ["plate1", "plate2", "plate3"]
            plate_half_extents = [w_plate/2, w_plate/2, h_plate/2]
            plate_color = [0.6, 0.6, 0.6, 1]
            plate_positions = [
                [table_x_offset, 0, 0.2+0.01],
                [table_x_offset, +gap_plate, 0.2+0.01],
                [table_x_offset, -gap_plate, 0.2+0.01]
            ]
            self.regions = {}
            for idx in range(3):
                region_name = plate_names[idx]
                self.regions[region_name] = sm.create_box(
                    body_name=region_name,
                    half_extents=plate_half_extents,
                    mass=1.,
                    pose=Pose(trans=plate_positions[idx]),
                    rgba_color=plate_color
                )
            
            # movables spec
            w_movable, h_movable = 0.04, 0.04
            movable_half_extents = [w_movable/2, w_movable/2, h_movable/2]
            movable_colors = [
                [1, 0, 0, 1],
                [1, 0.5, 0, 1],
                [1, 1, 0, 1],
                [0, 1, 0, 1],
                [0, 0, 1, 1],
                [0.3, 0, 0.5, 1],
                [0.6, 0, 0.8, 1],
            ]
            movable_names = [f"block{i}" for i in range(1, self.num_box+1)]
            left_xy_position = [table_x_offset, -gap_plate]
            right_xy_position = [table_x_offset, +gap_plate]
            xy_positions = [left_xy_position, right_xy_position]

            left_side_blocks = list(np.random.choice(movable_names, self.num_box // 2, replace=False))
            right_side_blocks = [name for name in movable_names if name not in left_side_blocks]
            block_groups = [left_side_blocks, right_side_blocks]

            np.random.shuffle(left_side_blocks)
            np.random.shuffle(right_side_blocks)

            for xy_pos, blocks in zip(xy_positions, block_groups):
                h = h_table + h_plate + h_movable/2 #init
                for block_name in blocks:
                    block_idx = int(block_name.strip("block")) - 1
                    self.movables[block_name] = sm.create_box(
                        body_name=block_name, 
                        half_extents=movable_half_extents, 
                        mass=1., 
                        pose=Pose(trans=[*xy_pos, h]),
                        rgba_color=movable_colors[block_idx])
                    h += h_movable

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
        
        self.placeables = {}
        self.placeables.update(self.regions)
        self.placeables.update(self.movables)
        
        self.robots = {}
        for name, robot in robots.items():
            self.robots[name] = robot

    def set_init_goal(self):
        # set by parent class
        hand_clean = ("handempty",)
        attached = []
        cleared_objects = [*self.movables.keys()]
        for movable_name, movable in self.movables.items():
            for placeable_name, placeable in self.placeables.items():
                if self.is_placed(movable, placeable):
                    attached.append(("attached", movable_name, placeable_name))
                    if placeable_name in cleared_objects:
                        cleared_objects.remove(placeable_name)
        
        cleared = [("clear", movable) for movable in cleared_objects]
        # tuple: abstract variable, att: mode variable 
        abs_var_init = [
            hand_clean,
            *attached,
            *cleared
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
        movables = [f"block{i+1}" for i in range(self.num_box)]
        for i in range(len(movables)-1):
            block_name = movables[i]
            block_parent = movables[i+1]
            goal.append(("attached", f"{block_name}", f"{block_parent}"))
        goal.append(("attached", f"{block_parent}", f"plate1")) # the tower is placed on the middle plate
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
    prob = ProblemBlocktower(gui=True, num_box=5)
    