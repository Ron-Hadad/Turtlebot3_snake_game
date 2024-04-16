# ROS Turtlebot3 Snake

## Overview
In this project, our goal is to make a turtle walk 10 different distances and 10 different angles. We also want to observe the turtle actions and make sure there are no rule breaking.

## Directory Contents:
### Scripts
1. `turtle_controller.py`: This script runs a node providing two services: one for moving the turtle and another for changing its angle.
2. `Observation.py`: This script runs a node observing turtle actions. It prints warnings if there are rule breaks such as insufficient segments, identical angles/lengths, wall contact, and line crossings. Note: "not enough segments" warnings appear when closing the Observation node only.
3. `control_script.py`: This script runs a node executing pre-decided actions. It adheres to rules and creates a track with 10 segments for the turtle.
4. `control_script_err.py`: Similar to `control_script.py`, this script executes actions but intentionally breaks rules, resulting in warnings for every possible rule break.

### Service Files (srv)
There are two service files: one for moving the turtle and another for turning it.

### Launch Files
There are two launch files:
1. `ass1.launch`: Activates nodes with the correct script for executing the task.
2. `ass1_err.launch`: Activates nodes with the script that breaks the rules, displaying rule-breaking messages in the terminal where the launch file was activated.

## How to Run
Follow these steps to run the project:

1. Place the package in the `catkin_ws/src` directory.
2. Ensure that the Python files (`ObservationService.py`, `control_script.py`, `control_script_err.py`, and `turtle_controller.py`) have executable permissions (make it happen by command 'chmod +x control_script.py' for every python file).
3. Make sure your ROS environment is set up correctly including sourcing (`you can do that with echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`).
4. Open a new terminal and run `roscore`.
5. Open another terminal and run the following command to observe the correct script execution.
   ```
    roslaunch ass1 ass1.launch
   ```
6. For rule-breaking scripts, use:
     ```
    roslaunch ass1 ass1_err.launch
     ```
