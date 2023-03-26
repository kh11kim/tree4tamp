# Define the objects with their types
objects = {"box1": "food", "box2": "food", "robot": "robot"}

# Define the domain name and problem name
domain_name = "my_domain"
problem_name = "my_problem"

# Define the initial state of the problem
initial_state = ["(at robot warehouse)"]
for obj, obj_type in objects.items():
    initial_state.append("(at {} warehouse)".format(obj))

# Define the goal state of the problem
goal_state = ["(and"]
for obj, obj_type in objects.items():
    goal_state.append("(at {} fridge)".format(obj))
goal_state.append(")")

def generate_pddl_problem_as_string(domain, problem):
    # Generate the PDDL problem file as a string
    pddl = f"(define (problem {problem_name})\n"
    pddl += f"  (:domain {domain_name})\n"
    pddl += "  (:objects\n"
    for obj, obj_type in objects.items():
        pddl += f"    {obj} - {obj_type}\n"
    pddl += "  )\n"
    pddl += "  (:init\n"
    for fact in initial_state:
        pddl += f"    {fact}\n"
    pddl += "  )\n"
    pddl += "  (:goal\n"
    for fact in goal_state:
        pddl += f"    {fact}\n"
    pddl += "  )\n"
    pddl += ")\n"

# Print the PDDL problem file
print(pddl)
