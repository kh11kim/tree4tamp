(define 
(problem KITCHEN-4-0)
(:domain KITCHEN)
(:objects
    robot - robot
    box1 box2 box3 box4 box5 - food
    sink1 - sink
    dish1 - dish
    oven1 - oven
)
(:init 
    ;(clear box1) 
    ;(clear box2) 
    ;(clear box3) 
    ;(clear box4) 
    (attached box1 dish1) 
    (attached box2 dish1) 
    (attached box3 dish1) 
    (attached box4 dish1) 
    (attached box5 dish1) 
    ;(attached box4 dish1)
    (handempty))

(:goal (and 
    (cooked box1) 
    (cooked box2) 
    (cooked box3) 
    (cooked box4) 
    (cooked box5) 
    ;(cooked box4)
    
    ))
)