#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from ass1.srv import Move_Turtle, Move_TurtleResponse
from ass1.srv import Turn_Turtle, Turn_TurtleResponse


class TurtleController:
    def __init__(self):
        rospy.init_node('turtle_controller')

        self.turtle_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)

        rospy.Service('move_turtle', Move_Turtle, self.move_turtle_handler)
        rospy.Service('turn_turtle', Turn_Turtle, self.turn_turtle_handler)

        rospy.loginfo("Turtle controller node ready to accept service requests.")
        rospy.spin()

    def move_turtle_handler(self, request):
        twist_msg = Twist()
        twist_msg.linear.x = request.length
        self.turtle_pub.publish(twist_msg)

        rospy.loginfo(f"Moving turtle forward by {request.length} units")

        rospy.sleep(1.0)

        response = Move_TurtleResponse(success=True)
        return response

    def turn_turtle_handler(self, request):
        twist_msg = Twist()
        twist_msg.angular.z = request.angle
        self.turtle_pub.publish(twist_msg)

        rospy.loginfo(f"Turning turtle by {request.angle} degrees")
        rospy.sleep(1.0)

        response = Turn_TurtleResponse(success=True)
        return response

if __name__ == "__main__":
    try:
        turtle_controller = TurtleController()
    except rospy.ROSInterruptException:
        pass
