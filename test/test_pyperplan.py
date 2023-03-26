from tree4tamp.planner.task_planner import Parser
from tree4tamp import *

#scene = DomainKitchen(gui=False, num_box=3)
problem = ProblemKitchen(gui=False, num_box=3, goal_box_list="all")
domain_file = "tree4tamp/domains/kitchen/domain_abstract.pddl"

def generate_pddl_problem_as_string(problem:ProblemKitchen,):
    def addline(pddl, fact):
        if isinstance(fact, str):
            pddl += f"    ({fact})\n"
        if isinstance(fact, tuple):
            pddl += f"    ({' '.join(fact)})\n"
        return pddl
    
    # Generate the PDDL problem file as a string
    pddl = f"(define (problem {problem.domain_name}_PROB1)\n"
    pddl += f"  (:domain {problem.domain_name})\n"
    pddl += "  (:objects\n"
    for obj, obj_type in problem.object_types.items():
        pddl += f"    {obj} - {obj_type}\n"
    pddl += "  )\n"
    pddl += "  (:init\n"
    for fact in problem.init:
        pddl = addline(pddl, fact) #pddl +=     #f"    {fact}\n"
    pddl += "  )\n"
    pddl += "  (:goal\n"
    for fact in problem.goal:
        pddl = addline(pddl, fact) #pddl += f"    {fact}\n"
    pddl += "  )\n"
    pddl += ")\n"
    return pddl

pddl_prob_string = generate_pddl_problem_as_string(problem)
parser = Parser(domain_file)
parser.probInput = pddl_prob_string
pddl_domain = parser.parse_domain()
pddl_problem = parser.parse_problem(pddl_domain, read_from_file=False)
print(pddl_string)
input()