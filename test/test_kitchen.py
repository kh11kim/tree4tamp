from pybullet_suite import *
from tree4tamp import *
from pyperplan import *

#world = BulletWorld(gui=True)
#panda = world.load_robot(Panda, "panda")

#

dom = DomainKitchen(gui=True, num_box=5)
prob = ProblemKitchen(dom, cooked_list="all")

input()
# problem = ProblemKitchen(dom)
# dom.domain_pddl_path
# problem.make_temp_pddl_files()