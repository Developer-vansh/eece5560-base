#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState

class Drive_Square:
    def __init__(self):
        # Initialize global class variables
        self.cmd_msg = Twist2DStamped()

        # Initialize ROS node
        rospy.init_node('drive_square_node', anonymous=True)
        
        # Initialize Publisher and Subscriber
        self.pub = rospy.Publisher('/yumdoot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/yumdoot/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)

    # Robot only moves when lane following is selected on the Duckiebot joystick app
    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()
        elif msg.state == "LANE_FOLLOWING":
            rospy.sleep(1)  # Give some time before starting
            self.move_robot()

    # Sends zero velocities to stop the robot
    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    # Robot drives in a square and then stops
    def move_robot(self):
        for i in range(4):
            # Drive forward
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.5  # linear velocity
            self.cmd_msg.omega = 0.0
            self.pub.publish(self.cmd_msg)
            rospy.loginfo("Driving forward - side %d", i+1)
            rospy.sleep(1.5)  # Adjust this for distance

            # Stop briefly
            self.stop_robot()
            rospy.sleep(0.2)

            # Rotate 90 degrees
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.0
            self.cmd_msg.omega = 1.5  # angular velocity
            self.pub.publish(self.cmd_msg)
            rospy.loginfo("Turning 90 degrees")
            rospy.sleep(1.5)  # Adjust this for accurate 90° turn

            # Stop briefly before next motion
            self.stop_robot()
            rospy.sleep(0.2)

        # Stop the robot after completing the square
        self.stop_robot()
        rospy.loginfo("Finished driving in a square.")

    # Keeps node from exiting until it's shut down
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass
