from pybullet_suite import *

world = BulletWorld(gui=True)

panda = world.load_robot(Panda, "panda")

input()
#from tree4tamp import *

# dom = DomainKitchen(gui=True, num_box=5)
# problem = ProblemKitchen(dom)
# dom.domain_pddl_path
# problem.make_temp_pddl_files()