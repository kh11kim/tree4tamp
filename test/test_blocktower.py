from pybullet_suite import *
from tree4tamp import *
from pyperplan import *
import time

prob = ProblemBlocktower(gui=True, num_box=7)
tree = TAMPTree(prob)
state_init = RTNode(prob.abs_state_init, prob.mode_init, prob.q_init)
states, commands = tree.plan(state_init)

for command in commands:
    for q in command.traj:
        prob.geometry_assign(command.mode, q)
        time.sleep(0.02)