#!/usr/bin/env python3

import rospy
from ass1.srv import Move_Turtle, Turn_Turtle
import math  # Import the math module

def move_turtle(length):
    rospy.wait_for_service('move_turtle',timeout=10)
    try:
        move_turtle_service = rospy.ServiceProxy('move_turtle', Move_Turtle)
        response = move_turtle_service(length)
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def turn_turtle(angle):
    rospy.wait_for_service('turn_turtle',timeout=10)
    try:
        turn_turtle_service = rospy.ServiceProxy('turn_turtle', Turn_Turtle)
        response = turn_turtle_service(angle)
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def control_script():
    rospy.init_node('control_script')
    rospy.loginfo("Control script node initialized.")


    # Specify 10 different lengths
    lengths = [1.5, 2.0, 1.8, 2.2, 1.7, 1.9, 2.5, 2.3, 2.8, 0.6]

    # Specify 10 different angles in degrees
    angles_degrees = [90, 110, -15, -25, 100, 80, -45, -110, 30, -85]
    for i in range(10):
        move_length = lengths[i]
        turn_angle_degrees = angles_degrees[i]

        # Move turtle
        if not move_turtle(move_length):
            rospy.logerr("Failed to move turtle.")
            return
        rospy.loginfo("finsh moving for now, time to turn")

        rospy.sleep(1)  # Add a 1-second delay between movement and turning

        #Convert degrees to radians
        turn_angle_radians = math.radians(turn_angle_degrees)

        # Turn turtle
        if not turn_turtle(turn_angle_radians):
            rospy.logerr("Failed to turn turtle.")
            return
        
        rospy.loginfo("finsh turning for now, time to move")

        rospy.sleep(1.0)  # Add a 1-second delay between movement and turning

    rospy.loginfo("Finished moving and turning the turtle.")

if __name__ == "__main__":
    control_script()
