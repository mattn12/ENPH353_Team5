# Introduction

## Authors
Project repository for UBC's ENPH 353 course, a guided, self-directed project course aimed at applying control software and machine learning.

Written by Matthew Ngai and Christopher Griffiths in Winter 2022 Term 1.


## Project Description
* Have a robot autonomously drive around a parking environment while remaining on the road, avoiding a moving (circling) vehicle, and stopping at cross walks to wait for pedestrians to cross
* Report licence plate numbers of parked cars with their respective position number
* Overcome challenging environment features: multiple intersections connecting two concentric roads which limit road line visibility; inclined hills requiring more work to ascend; an "off-road" section with non-uniformly coloured path


## Our Objectives
* Confidently and correctly identify licence plates and position numbers
* Follow the road in the outer ring reliably while identifying and responding to crosswalks and pedestrians


## File Structure
* Our controller is located inside a ROS package called [`enph353_comp_controller`](https://github.com/mattn12/ENPH353_Team5/tree/master/src/enph353_comp_controller). 
* Inside the source directory, we have the main script `state_machine.py`, and the three modules that are used by the main script: `cross_walk.py`, `read_plate.py`, `move_robot.py`.
* The `plate_models` folder stores the two `*.h5` files of the neural network models used to read letters and numbers.


## State Machine Structure
* On the left, a legend of which python module/script the blocks belong to 
* Rectangular blocks are methods (process robot camera input data)
* Rounded blocks are instructions (output to robot motors/licence plate reporting server)
* Diamond shaped blocks are processes running in the background


## More Information
Please refer to the [project report](https://drive.google.com/file/d/1nqoO3BlICYaCd9h2kU0Hw2mw3Q8LhTc6/view?usp=sharing) for a more in-depth analysis of the PID controller and optical character recognition neural network.
