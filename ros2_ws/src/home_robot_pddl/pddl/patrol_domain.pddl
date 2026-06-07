(define (domain home_robot_patrol)
  (:requirements :strips :typing :durative-actions :numeric-fluents)

  (:types
    robot location
  )

  (:functions
    (travel_time ?from ?to - location)
  )

  (:predicates
    (robot_at ?r - robot ?l - location)
    (connected ?from ?to - location)
    (visited ?l - location)
  )

  (:durative-action move
    :parameters (?r - robot ?from ?to - location)
    :duration (= ?duration (travel_time ?from ?to))
    :condition (and
      (at start (robot_at ?r ?from))
      (at start (connected ?from ?to))
    )
    :effect (and
      (at start (not (robot_at ?r ?from)))
      (at end (robot_at ?r ?to))
      (at end (visited ?to))
    )
  )
)
