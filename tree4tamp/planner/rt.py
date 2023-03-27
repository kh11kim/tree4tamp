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
        mode: Dict[str, Attachment], 
        q:Config, 
        # traj_switch: List[Config]=None
    ):
        self.s = abs_state
        self.sigma = mode
        self.q = q
        
        self.index: int = -1
        self.parent: RTNode = None
        # self.traj_switch = traj_switch

    # def copy(self):
    #     q = deepcopy(self.q)
    #     sigma = deepcopy(self.sigma)
    #     state = deepcopy(self.state)
    #     self.abs_state_node
    #     node = RGNode(state, sigma, q)
    #     node.abs_state_node = self.abs_state_node
    #     return node

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
        self.add_node(child)
        child.parent = parent
        parent.children[action] = child
    
