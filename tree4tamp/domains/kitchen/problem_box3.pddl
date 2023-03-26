(define 
(problem KITCHEN-4-0)
(:domain KITCHEN)
(:objects
    robot - robot
    box1 box2 box3 - food
    sink1 - sink
    dish1 - dish
    oven1 - oven
)
(:init 
    (attached box1 dish1) 
    (attached box2 dish1) 
    (attached box3 dish1) 
    (handempty))

(:goal (and 
    (cooked box1) 
    (cooked box2) 
    (cooked box3) 
    
    ))
)