from pybullet_suite import *
from tree4tamp import *
import time


def visualize_plan(commands):
    for command in commands:
        for q in command.traj:
            prob.geometry_assign(command.mode, q)
            time.sleep(0.05)

viz = True

prob = ProblemKitchen(gui=viz, num_box=5, goal_box_list="all")
# prob = ProblemBlocktower(gui=viz, num_box=6)
# prob = ProblemNonmonotonic(gui=viz, num_box=3)

elapsed_times = []
for i in range(10):
    tree = TAMPTree(prob, planner="wastar", heuristic="hff")
    state_init = RTNode(prob.abs_state_init, prob.mode_init, prob.q_init)
    result = tree.plan(state_init)
    elapsed_times.append(tree.elapsed_time)
    
    if result is not None and viz == True:
        visualize_plan(result[1])
        

