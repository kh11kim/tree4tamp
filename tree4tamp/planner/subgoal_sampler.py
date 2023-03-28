from tree4tamp.tamp import *
from tree4tamp.planner.reachability_tree import *
from tree4tamp.planner.task_planner import *
from copy import deepcopy

class SubgoalSamplingLayer:
    def __init__(self, problem:TAMPProblem):
        self.problem = problem

    def do_geometric_planning(
            self,
            pi:List[Operator], 
            art_node_seq:List[ARTNode],
            tree:RT
        ):
            """the main function of SS layer"""
            pass
    
    def feasibility_check_by_goal_candidate(self, tree:RT):
        pass

    def sample_batch_attachments(self, pi: List[Operator]):
        attachments = []
        movables = deepcopy(self.problem.movables)
        for a in pi[::-1]:
            if not a.is_geometric_action():
                attachments.append(None)
                continue
            m = a.get_target_movable()
            if (m in movables) and (m in self.problem.mode_goal.keys()):
                att = self.problem.mode_goal[m] #get goal attachment
                attachments.remove(m)
            else:
                #sample attachment
                att = self.problem.sample_attachment_by_action(a)
            attachments.append(att)
        return attachments[::-1]

    def make_goal_candidate(
        self, 
        state_init:RTNode, 
        pi:List[Operator], 
        batch_atts:List[Attachment]
    ):
        abs_state_goal = deepcopy(state_init.s)
        mode_goal = deepcopy(state_init.sigma)
        for a, att in zip(pi, batch_atts):
            a.apply(abs_state_goal)
            mode_goal.set_attachment(att)
        
        RTNode(abs_state_goal, mode_goal, q)
        