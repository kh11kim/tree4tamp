#from tree4tamp.planner.task_planner import Parser
from tree4tamp import *

#scene = DomainKitchen(gui=False, num_box=3)
problem = ProblemKitchen(gui=False, num_box=3, goal_box_list="all")
domain_file = "tree4tamp/domains/kitchen/domain_abstract.pddl"

pddl_prob_string = problem.generate_pddl_problem_as_string()
tp = TaskPlanningLayer(
    domain_file,
    pddl_prob_string
)

art = ART(ARTNode(tp.abs_state_init))
art_node_seq = tp.get_abstract_plan(art)

print(pddl_prob_string)

pi = tp_layer.get_plan_from_state()
print(pi)

#plan!
task = ground(problem)

print(pddl_string)
input()