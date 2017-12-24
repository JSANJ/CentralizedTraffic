# CentralizedTraffic
Code created and maintained by Justin David Q. SAN JUAN, 
email: jdqsj1997@yahoo.com
website: justinsj.weebly.com

This project will simulate a centralized traffic system whose aim is to minimize the need for deceleration in traffic intersections given random requests for entry in the intersection. The current stage will look at a rotationally symmetric 4-sided, 2-lane in each direction intersection. The aim for the project is to: 
  1. Use simple rule-based, non-iterative physical equations to quickly calculate the necessary actions for each car coming to an intersection to prevent crashes. 
  2. Be generalizable to different intersection conditions, ex. asymmetrical intersections, three-way intersections, slanted road merges, rotundas, etc.

Conditions: 
  1. Max acceleration/deceleration are defined.
  2. Max speed is defined.
  3. Runway lengths are equal.
  4. Interesction has 4 sides.
  5. 4 lanes (2 each direction) on each side.
  6. Constant speed in intersection (for simplifaction).
  7. Right-lane driving (left-side driver).
  
Assumptions:  
  1. (Currently) Only going straight.
  2. Acceleration/deceleration is constant and discontinuous.
  
To be implemented:
  1. Detect collisions.
  2. Determine collision area.
  3. Vectorization of car velocity/acceleration/facing-direction, etc.
  4. Safety margin for collisions.
  5. Left/Right/U-turns.
  6. Continuous (linear) acceleration/deceleration.
  7. Continuous (non-linear) acceleration/deceleration.
  8. Generalization to left-lane driving (right-side driver).
  9. Generalization to different intersection conditions.

Features:
  1. Simulates car crossings as Python Animation.
  2. Calculates and decelerates (with constant deceleration) straight-crossing vehicles based on collision from runway until collision area.
  
