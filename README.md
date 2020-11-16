# Delivery Robot

This repository contains the code for the CS 81 final project, the "Delivery robot", in which we design a robot that explores an environment and delivers packages to the appropriate locations.
The environment and obstacles are defined by a *map* loaded by the server at the start of the program.
The delivery locations can be specified as input or are randomly distributed in *sites* within the map.
When all delivery locations are visited, the program ends and a summary is printed.

## Requirements spec
* The robot should successfully load in a map
* The robot should find an optimal path to all delivery locations
* The robot should complete all deliveries, ensuring the right packages are delivered to the right destinations
* The robot should
## Other materials provided
See the [maps]() for some test maps.


## Compilation / Usage
 * Setup the ROS environment by running `roscore` in one terminal
 * To launch the `delivery_robot`, run `roslaunch delivery_robot delivery_robot.launch` in another terminal


