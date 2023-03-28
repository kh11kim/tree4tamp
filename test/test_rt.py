from tree4tamp import *

prob = ProblemKitchen(gui=True, num_box=3, goal_box_list="all")
domain_file = "tree4tamp/domains/kitchen/domain_abstract.pddl"
pddl_prob_string = prob.generate_pddl_problem_as_string()
tp = TaskPlanningLayer(
    domain_file,
    pddl_prob_string,
    eps = 0.6
)
ss = SubgoalSamplingLayer(prob)
for a in tp.task.operators:
    print(a.name)
    
rt_root = RTNode(tp.abs_state_init, prob.mode_init, prob.q_init)
art_root = ARTNode(tp.abs_state_init)
art_root.rt_node_list.append(rt_root)

rt = RT(rt_root)
art = ART(art_root)

for i in range(10):
    pi, art_node_seq = tp.get_abstract_plan(art)
    batch_atts = ss.sample_batch_attachments(pi)
    ss.make_goal_candidate()
    print("a")

print("done")