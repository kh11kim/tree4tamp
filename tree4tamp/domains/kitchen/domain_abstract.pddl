;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Kitchen domain : you have to wash foods before to cook.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain KITCHEN)
  (:requirements :strips :typing)
  (:types
    robot placeable - object
  	movable region - placeable
    sink oven dish - region
    food - movable
  )

  (:predicates 
    (attached ?obj - food ?parent - object)
    (handempty)
    (cleaned ?obj - food)
    (cooked ?obj - food)
  )

  (:action wash
	     :parameters (?obj - food ?parent - sink)
	     :precondition (and (attached ?obj ?parent))
	     :effect
	     (and (cleaned ?obj)))
  
  (:action cook
	     :parameters (?obj - food ?parent - oven)
	     :precondition (and (attached ?obj ?parent) (cleaned ?obj))
	     :effect
	     (and (cooked ?obj)))
		   
  (:action pick
	     :parameters (?obj - food ?parent - region ?robot - robot)
	     :precondition (and (attached ?obj ?parent) (handempty))
	     :effect
	     (and (not (attached ?obj ?parent))
		   (not (handempty))
		   (attached ?obj ?robot)))

  (:action place
	     :parameters (?obj - food ?robot - robot ?parent - region)
	     :precondition (attached ?obj ?robot)
	     :effect
	     (and (not (attached ?obj ?robot))
		   (handempty)
		   (attached ?obj ?parent))))
