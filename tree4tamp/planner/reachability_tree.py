import sys
from tree4tamp.domains import *
from dataclasses import dataclass, field
from typing import List, Dict

sys.path.append('./pyperplan/')
from pyperplan.task import Operator, Task

class RTNode:
    def __init__(
        self, 
        abs_state:frozenset,
        mode: Mode, #Dict[str, Attachment], 
        q:Config
    ):
        self.s = abs_state
        self.mode = mode
        self.q = q
        self.grasp_pose = {}
        self.approach_traj = None
        self.q_grasp = None
        self.index: int = -1
        self.parent: RTNode = None
        
        

    def unpack(self):
        return self.s, self.mode, self.q

    def set_q_grasp(self, q_grasp, grasp_robot_name):
        self.q_grasp = q_grasp
        self.grasp_robot_name = grasp_robot_name

    def set_grasp_pose(self, grasp_pose, robot_name):
        self.grasp_pose[robot_name] = grasp_pose
    
    def set_approach_traj(self, traj):
        self.approach_traj = traj

    # def copy(self):
    #     q = deepcopy(self.q)
    #     sigma = deepcopy(self.sigma)
    #     state = deepcopy(self.state)
    #     self.abs_state_node
    #     node = RGNode(state, sigma, q)
    #     node.abs_state_node = self.abs_state_node
    #     return node


@dataclass
class RTEdge:
    action: Union[Operator, str]
    traj: field(default_factory=lambda :None)
    mode: Mode

class RT:
    """Reachability Tree"""
    def __init__(self, root:RTNode):
        self.V: List[RTNode] = []
        self.E = {}
        self.sol:RTNode = None
        self.add_node(root)
    
    @property
    def root(self):
        return self.V[0]

    def add_node(self, node:RTNode):
        node.index = len(self.V)
        self.V += [node]
    
    def add_edge(self, parent:RTNode, child:RTNode, a:Operator, traj:List[Config]=None, mode=None):
        child.parent = parent
        edge = RTEdge(a, traj, parent.mode)
        self.E[(parent.index, child.index)] = edge
    
    def get_edge(
        self, parent:RTNode, child:RTNode)->RTEdge:
        return self.E[(parent.index, child.index)]
    
    def add_child(self, parent:RTNode, child:RTNode, a:Operator, traj:List[Config]=None):
        self.add_node(child)
        self.add_edge(parent, child, a, traj, parent.mode)
        child.parent = parent
    

@dataclass
class ARTNode:
    abs_state: frozenset
    num_visits: int = field(default_factory=lambda :0)
    reward_sum: float = field(default_factory=lambda :0.)
    rt_node_list: List[RTNode] = field(default_factory=lambda :[])
    # tree
    index: int = field(default_factory=lambda :-1)
    parent: Optional["ARTNode"] = field(default_factory=lambda :None)
    children: Dict[Operator,"ARTNode"] = field(default_factory=lambda :{})

    def sample_rt_node(self):
        if len(self.rt_node_list) == 0: return None
        idx = np.random.randint(0, len(self.rt_node_list))
        return self.rt_node_list[idx]

class ART:
    """Abstract Reachability Tree
    """
    def __init__(self, root:ARTNode):
        self.V: List[ARTNode] = [root]
        root.index = len(self.V)
        
    @property
    def root(self):
        return self.V[0]
    
    # def add_node(self, node:ARTNode):
    #     node.index = len(self.V)
    #     self.V.append(node)

    def add_child(self, parent:ARTNode, child:ARTNode, action:Operator):
        assert parent.index != -1
        child.index = len(self.V)
        self.V.append(child)
        #self.add_node(child)
        child.parent = parent
        parent.children[action] = child
    
