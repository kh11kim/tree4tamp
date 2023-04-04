import numpy as np
import sys

#from sympy import re
from .reachability_tree import ART, ARTNode
from typing import Iterable, TypeVar, Union, List
from pyperplan.task import Operator, Task
#from tree4tamp.tamp.tamp_problem import TAMPProblem
sys.path.append('./pyperplan/')

from pyperplan.pddl.parser import Parser
from pyperplan.planner import HEURISTICS, SEARCHES, _parse, _ground, _search

T = TypeVar('T')
def choose_random(x:Iterable[T])->T:
    i = np.random.randint(0, len(x))
    return x[i]

class TaskPlanner:
    def __init__(
        self,
        task,
        eps,
        planner,
        heuristic,
        p_terminal=0.05,
    ):
        self.task = task
        self.search = SEARCHES[planner]
        self.heuristic = HEURISTICS[heuristic](task)

        self.p_terminal = p_terminal
        self.eps = eps

    def get_abstract_plan(self, art:ART):
        """the main function of Task Planning Layer.
        """
        pi = self.sample_action_sequence(art)
        return pi, self.get_ARTnode_sequence(art, pi)

    def get_plan_from_state(self, state:frozenset=None):
        """Symbolic Planner
        """
        if isinstance(state, ARTNode):
            state = state.abs_state
        assert isinstance(state, frozenset)
        if state is None:
            self.task.initial_state = self.abs_state_init
        else:
            self.task.initial_state = state
        return _search(self.task, self.search, self.heuristic)
    
    def randomized_tree_search(self, tree:ART):
        pi_rand, node = [], tree.root
        #e-greedy:
        while True:
            a = self.select_by_egreedy(node)
            if isinstance(a, str): #TODO: string means the terminal action
                break # terminate in the middle
            elif a not in node.children:
                node = a.apply(node.abs_state)
                pi_rand.append(a)
                break # terminate with a new leaf
            else:
                node = node.children[a]
                pi_rand.append(a) # select the node by the action and continue
        return node, pi_rand

    def select_by_egreedy(self, node:ARTNode)->Union[str, Operator]:
        if np.random.random() < self.p_terminal:
            # terminate sequence with some probability to revisit previous plans.
            return "terminal"
        elif len(node.children) == 0 or np.random.random() > self.eps:
            # choose an action randomly
            applicable_actions = [op for op in self.task.operators if op.applicable(node.abs_state)]
            return choose_random(applicable_actions)
        else:
            # exploit the previous results
            actions = list(node.children.keys())
            np.random.shuffle(actions) #tie breaker
            values = []
            for a in actions:
                child = node.children[a]
                if child.num_visits == 0.:
                    values.append(0.)
                else: values.append(child.reward_sum/child.num_visits)
            return actions[np.argmax(values)]

    def sample_action_sequence(self, art:ART):
        last_abs_state, pi_rand = self.randomized_tree_search(art)
        pi_plan = self.get_plan_from_state(last_abs_state)
        pi = pi_rand + pi_plan
        self.extend_tree_by_pi(art, pi)
        return pi
    
    def extend_tree_by_pi(self, art:ART, pi:List[Operator]):
        node = art.root
        for a in pi:
            if a in node.children:
                node = node.children[a]
            else:
                abs_state_new = a.apply(node.abs_state)
                node_new = ARTNode(abs_state_new)
                art.add_child(node, node_new, a)
                node = node_new

    def get_ARTnode_sequence(self, tree:ART, pi:List[Operator]):
        node = tree.root
        seq = [node]
        for a in pi:
            node = node.children[a]
            seq.append(node)
        return seq

    def update_tree(self, art_node_seq:List[ARTNode], reward:float):
        for art_node in art_node_seq:
            art_node.num_visits += 1
            art_node.reward_sum += reward
