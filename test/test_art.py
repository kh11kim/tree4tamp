from tree4tamp import *

problem = ProblemKitchen(gui=False, num_box=3, goal_box_list="all")
domain_file = "tree4tamp/domains/kitchen/domain_abstract.pddl"
pddl_prob_string = problem.generate_pddl_problem_as_string()
tp = TaskPlanningLayer(
    domain_file,
    pddl_prob_string,
    eps = 0.6
)
art = ART(ARTNode(tp.abs_state_init))

for i in range(10):
    pi, art_node_seq = tp.get_abstract_plan(art)
    for a in pi:
        print(f"{i}", a.name)

print("done")