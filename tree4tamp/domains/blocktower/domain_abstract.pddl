;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Pick-And-Place domain
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain PICK-PLACE-STACK-UNSTACK)
  (:requirements :strips :typing :equality)
  (:types
    robot placeable - object
    movable region - placeable
  )

  (:predicates 
    (attached ?obj - movable ?parent - object)
    (handempty)
    (clear ?obj - movable)
  )

  (:action pick
    :parameters (?obj - movable ?parent - region ?robot - robot)
    :precondition (and 
      (attached ?obj ?parent) 
      (handempty) 
      (clear ?obj)
    )
    :effect (and 
      (not (attached ?obj ?parent))
      (not (handempty))
      (not (clear ?obj))
      (attached ?obj ?robot)
    )    
  )

  (:action place
    :parameters (?obj - movable ?robot - robot ?parent - region)
    :precondition (and (attached ?obj ?robot))
    :effect
      (and 
        (not (attached ?obj ?robot))
        (handempty)
        (attached ?obj ?parent)
        (clear ?obj)
      )
  )
  (:action stack
    :parameters (?obj - movable ?robot - robot ?parent - movable)
    :precondition (and (attached ?obj ?robot) (clear ?parent))
    :effect
      (and 
        (not (attached ?obj ?robot))
        (handempty)
        (attached ?obj ?parent)
        (not (clear ?parent))
        (clear ?obj)
      )
  )

  (:action unstack
    :parameters (?obj - movable ?parent - movable ?robot - robot)
    :precondition (and (attached ?obj ?parent) (handempty) (clear ?obj)) ;
    :effect
    (and 
      (not (attached ?obj ?parent))
      (not (handempty))
      (not (clear ?obj))
      (clear ?parent)
      (attached ?obj ?robot)
    )    
  )

)
