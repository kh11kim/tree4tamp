from tree4tamp.tamp import *
from tree4tamp.planner.reachability_tree import *
from tree4tamp.planner.task_planner import *
from copy import deepcopy

class SubgoalSampler:
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
        for a in pi[::-1]:
            if not a.is_geometric_action():
                attachments.append(None)
                continue
            m = a.get_target_movable()
            if (m in self.problem.movables.keys()) and (m in self.problem.atts_goal_dict.keys()):
                att = self.problem.atts_goal_dict[m] #get goal attachment
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
        mode_goal = state_init.mode
        for a, att in zip(pi, batch_atts):
            abs_state_goal = a.apply(abs_state_goal)
            mode_goal = mode_goal.set_attachment(att)
        if self.problem.q_goal is not None:
            q_goal = self.problem.q_goal
        else:
            q_goal = self.problem.sample_random_config()
        return RTNode(abs_state_goal, mode_goal, q_goal)
        
