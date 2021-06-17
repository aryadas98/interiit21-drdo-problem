# Inter IIT Tech Meet 2021 DRDO Problem

This is the source code for IIT Patna's submission to the Inter IIT Tech Meet 2021 DRDO problem. Specifically, the solution is found in the file [`./src/controller.py`](./src/controller.py).

## Problem Statement

The problem statement is to make a drone pass autonomously through an indoor environment filled with obstacles. The drone is equipped with a downward facing rgb camera and a forward facing depth camera. The drone must use its sensors to avoid obstacles and navigate through the space.

At the end of the room, there is an aruco marker which the drone must detect and land on, autonomously.

## Our Solution

We chose a simple solution based on virtual forces. The drone is assumed to have a virtual positive charge. There is assumed to be a large negative charge in front of the drone. This causes a virtual attraction at the front and the drone moves towards the front. Meanwhile, all obstacles are assumed to have virtual positive charges. This causes a virtual repulsion and the drone moves away from the obstacles.

This was a simple and effective way to complete a significant part of the problem statement.

Limitations of this approach:
1. The drone might get stuck in a local minima. This is because the drone does not consider the bigger picture when moving.
2. The drone might turn around 180 degrees and move backwards. This might happen when the drone is stuck.

For landing the drone, we simply detect the aruco marker and its center. Then we fly towards the center and issue the landing command.

Our final presentation is available here: [Final PPT.pdf](./Final%20PPT.pdf)

A demo video is available here: https://www.youtube.com/watch?v=_hyK7T1MAbo
