from pybullet_suite import *
from copy import deepcopy
from typing import Union, Callable, ClassVar
from dataclasses import dataclass, field 

PRE_POSE_DISTANCE = 0.1

# The definitions of SOP, SSSP are in this paper:  https://ieeexplore.ieee.org/abstract/document/8411475
@dataclass
class SOP:
    """Stable object poses"""
    rot: Rotation
    axis: np.ndarray # axis of rotation
    height: float # height from sssp

    @property
    def tf(self):
        return Pose(rot=self.rot, trans=[0,0,self.height]).inverse() #TODO:this assumes axis is z

@dataclass
class SSSP:
    """Surfaces Supporting Stable Placement"""
    lower: np.ndarray
    upper: np.ndarray

@dataclass
class Attachment:
    obj_name: str
    tf: Pose
    parent_name: Optional[str] = field(default_factory=lambda : None)
    pre_pose_distance: Optional[float] = field(default_factory=lambda : PRE_POSE_DISTANCE)
    att_type: str = field(default_factory=lambda :"")

    @staticmethod
    def get_pre_pose(pose:Pose):
        raise NotImplementedError()
    
    # def __eq__(self, other):
    #     return (self.parent_name == other.parent_name) and \
    #         (self.obj_name == other.obj_name) and \
    #         (self.tf == other.tf)

@dataclass
class Grasp(Attachment):
    width: Optional[float] = field(default_factory=lambda : 0)
    approach: str = field(default_factory=lambda :"top")
    pre_pose: Optional[Pose] = field(default_factory=lambda :None)
    att_type: str = field(default_factory=lambda :"grasp")
    
    def __post_init__(self):
        if self.approach == "top":
            self.pre_pose = Pose(trans=[0,0,+self.pre_pose_distance])
        elif self.approach == "side":
            self.pre_pose = Pose(trans=[0,0,-self.pre_pose_distance])

    def __repr__(self):
        return f"Grasp: obj-{self.obj_name}"

    def get_pre_pose(self, pose:Pose, approach = None):
        if approach is None:
            approach = self.approach
        if approach == "top":
            return self.pre_pose * pose
        elif approach == "side":
            return pose * self.pre_pose
    
    # def __hash__(self):
    #     return f"Grasp:{self.obj_name}_{self.parent_name}_{self.tf.as_1d_numpy()}"

@dataclass
class Placement(Attachment):
    point: Optional[np.ndarray] = field(default_factory=lambda : None)
    sop: Optional[SOP] = field(default_factory=lambda : None)
    yaw: Optional[float] = field(default_factory=lambda : None)
    obj_pose: Optional[Pose] = field(default_factory=lambda : None)
    att_type: str = field(default_factory=lambda :"placement")

    def get_pre_pose(self, pose: Pose):
        pre_pose = Pose(trans=[0,0,+self.pre_pose_distance])
        return pre_pose * pose

    def __repr__(self):
        return f"Placement: obj-{self.obj_name}, region-{self.parent_name}"
    
    # def __hash__(self):
    #     return f"Placement:{self.obj_name}_{self.parent_name}_{self.tf.as_1d_numpy()}"

    @classmethod
    def from_point_and_sop(
        cls, 
        movable_name: str,
        placeable_name: str,
        placeable_pose: Pose,
        point: np.ndarray,
        sop: SOP,
        yaw: Optional[float]=None #if None, a random yaw is sampled
    ):
        if yaw is None:
            yaw = np.random.uniform(0, np.pi*2)
        trans = point + np.array([0,0,sop.height])
        rot = Rotation.from_euler("xyz",[0,0,yaw]) * sop.rot
        obj_pose = Pose(rot=rot, trans=trans)
        tf = obj_pose.inverse() * placeable_pose
        return Placement(
            obj_name=movable_name, 
            tf=tf, 
            point=point, 
            sop=sop, 
            yaw=yaw,
            obj_pose=obj_pose,
            parent_name=placeable_name)

class Mode:
    """ All attachments of movables"""
    def __init__(self, atts: Dict[str, Attachment]):
        self.atts: Dict[str, Attachment] = atts
        #self.mode_key: Dict[str, Tuple[str, np.ndarray]] = self.get_mode_key(atts)

    def set_attachment(self, att:Attachment):
        mode_new = deepcopy(self)
        if att is None: return mode_new
        mode_new.atts[att.obj_name] = att
        return mode_new
        #self.mode_key = self.get_mode_key(self.attachments)

    @classmethod
    def from_list(cls, atts_list:List[Attachment]):
        atts = {}
        for att in atts_list:
            atts[att.obj_name] = att
        return cls(atts)

    def __repr__(self):
        s = "_".join([obj+"-"+key[0] for obj, key in self.mode_key.items()])
        return f"Mode: {s}"

    # def get_mode_key(self, atts:Dict[str, Attachment]):
    #     mode_key = {}
    #     for obj, att in atts.items():
    #         mode_key[obj] = (att.parent_name, att.tf.as_1d_numpy())
    #     return mode_key
    
    # def get_hashable_key(self):
    #     temp = []
    #     for obj, att in sorted(self.attachments.items()):
    #         temp += [obj, att.__hash__()]
    #     return "_".join(temp)
    
    # def as_1d_numpy(self):
    #     names = np.sort(list(self.attachments.keys()))
    #     array = np.concatenate([self.attachments[name].tf.as_1d_numpy() for name in names])
    #     return array


    @classmethod
    def from_list(cls, att_list: List[Attachment]):
        atts = {}
        for att in att_list:
            obj = att.obj_name
            atts[obj] = att
        return cls(atts)
            
    def copy(self):
        return Mode(deepcopy(self.atts))

    # def __eq__(self, other:"Mode"):
    #     for obj in self.mode_key:
    #         parent_name1, tf_vec1 = self.mode_key[obj]
    #         parent_name2, tf_vec2 = other.mode_key[obj]
    #         if (parent_name1 != parent_name2) | \
    #             (not np.array_equal(tf_vec1, tf_vec2)):
    #             return False
    #     return True


class TAMPObject(Body):
    """Wrapper function of Body object from pybullet_suite library"""
    def __init__(
        self, 
        physics_client: BulletClient, 
        body_uid: int,
        name: str,
        sssp: SSSP
    ):
        super().__init__(
            physics_client,
            body_uid,
            name=name
        )
        self.name = name
        self.is_movable = None
        self.sssp = sssp # surfaces supporting stable placement
    
class Movable(TAMPObject):
    def __init__(
        self,
        physics_client: BulletClient,
        body_uid: int,
        name: str,
        sops: List[SOP],
        sssp: SSSP
    ):
        super().__init__(
            physics_client=physics_client,
            body_uid=body_uid,
            name=name,
            sssp=sssp
        )
        # Stable Object Poses : (Rotation, axis of rotation: R3, distance to a sssp: R1)
        self.sops: List[SOP] = sops
        
    @classmethod
    def from_body(cls, body: Body, name: str, sops, sssp):
        return cls(body.physics_client, body.uid, name, sops, sssp)
    
    def set_grasps(self, grasps:Union[List, Callable]):
        if isinstance(grasps, List):
            self.grasps = grasps
            self.grasp_type = "discrete"
        else:
            #assume continuous grasp (function)
            self.grasp_gen_fn: Callable[[],Grasp] = grasps
            self.grasp_type = "continuous"
        
    def sample_grasp(self):
        if self.grasp_type == "discrete":
            i = np.random.randint(0, len(self.grasps))
            return deepcopy(self.grasps[i])
        elif self.grasp_type == "continuous":
            return self.grasp_gen_fn()
    
    def sample_sop(self):
        """Stable object pose
        """
        i = np.random.randint(0, len(self.sops))
        return deepcopy(self.sops[i])


class Region(TAMPObject):
    def __init__(
        self,
        physics_client: BulletClient,
        body_uid: int,
        name: str,
        sssp: SSSP
    ):
        super().__init__(
            physics_client=physics_client,
            body_uid=body_uid,
            name=name,
            sssp=sssp
        )
        
    @classmethod
    def from_body(cls, body: Body, name: str, sssp: SSSP):
        return cls(body.physics_client, body.uid, name, sssp)


class Config:    
    """A Configuration of the robots"""
    robot_names = []
    def __init__(self, q_dict : Dict[str, np.ndarray]):
        self.q = q_dict
    #robot_names: ClassVar[List[str]] = []
    
    @classmethod
    def set_robot_names(cls, robot_names:List):
        cls.robot_names = robot_names

    def __repr__(self):
        return f"Config:"

    def set_joints(self, robot_name, value:np.ndarray):
        assert robot_name in self.q.keys()
        self.q[robot_name] = value