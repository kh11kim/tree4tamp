from tree4tamp import *

problem = ProblemKitchen(gui=False, num_box=3, goal_box_list="all")
domain_file = "tree4tamp/domains/kitchen/domain_abstract.pddl"
pddl_prob_string = problem.generate_pddl_problem_as_string()
tp = TaskPlanningLayer(
    domain_file,
    pddl_prob_string,
    eps = 0.6
)

for a in tp.task.operators:
    print(a.name)
    
rt_root = RTNode(tp.abs_state_init, problem.mode_init, problem.q_init)
art_root = ARTNode(tp.abs_state_init)
art_root.rt_node_list.append(rt_root)

rt = RT(rt_root)
art = ART(art_root)

for i in range(10):
    pi, art_node_seq = tp.get_abstract_plan(art)
    for a in pi:
        print(f"{i}", a.name)

print("done")