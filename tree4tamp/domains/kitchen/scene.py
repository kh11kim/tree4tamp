from pathlib import Path

from pybullet_suite import *
from ...tamp import *

##############################
class SceneKitchen(TAMPDomain):
    def __init__(self, gui, num_box=2):
        domain_pddl_path = Path(__file__).parent / Path("domain_abstract.pddl")
        self.num_box = num_box
        self.domain_name = "KITCHEN"
        super().__init__(
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
            envs = {}
            envs["table"] = sm.create_table(
                "table", 0.4, 1.2, 0.2, 
                x_offset=0.4, y_offset=0, z_offset=0.2)
            envs["ground"] = sm.create_table("ground", 2, 2, 0.4) #ground
            
            # set regions
            regions = {}
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
                regions[name] = sm.create_box(
                    body_name=name, 
                    half_extents=half_extents[idx], 
                    mass=1., 
                    pose=Pose(trans=positions[idx]),
                    rgba_color=colors[idx])
            
            # set movables
            movables = {}
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
                movables[name] = sm.create_box(
                    body_name=name, 
                    half_extents=half_extents, 
                    mass=1., 
                    pose=Pose(trans=positions[idx]),
                    rgba_color=colors[idx])

            # set robot
            robots = {"robot":self.world.load_robot(name="robot", robot_class=Panda)}
                
        self.world.set_view(eye_point=[1.2,-0.2,0.7])
        self.world.wait_for_rest()
        return movables, regions, robots, envs

    def set_tamp_objects(self, movables:Dict, regions:Dict, robots:Dict):
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