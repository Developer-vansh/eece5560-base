#******************5.1P************************

##!/usr/bin/env python3
#
#import rospy
#from duckietown_msgs.msg import Twist2DStamped
#from duckietown_msgs.msg import AprilTagDetectionArray
#
#class Target_Follower:
#    def __init__(self):
#        rospy.init_node('target_follower_node', anonymous=True)
#        rospy.on_shutdown(self.clean_shutdown)
#
#        self.cmd_vel_pub = rospy.Publisher('/yumdoot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
#        rospy.Subscriber('/yumdoot/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
#
#        self.latest_x = None
#        self.tag_visible = False
#        self.last_tag_seen_time = rospy.Time.now()
#        self.tag_timeout = rospy.Duration(1.0)
#        self.last_action = None
#
#        rospy.Timer(rospy.Duration(0.1), self.behavior_timer)
#        rospy.spin()
#
#    def tag_callback(self, msg):
#        self.move_robot(msg.detections)
#
#    def clean_shutdown(self):
#        rospy.loginfo("System shutting down. Stopping robot...")
#        self.stop_robot()
#
#    def stop_robot(self):
#        cmd_msg = Twist2DStamped()
#        cmd_msg.header.stamp = rospy.Time.now()
#        cmd_msg.v = 0.0
#        cmd_msg.omega = 0.0
#        self.cmd_vel_pub.publish(cmd_msg)
#
#    def move_robot(self, detections):
#        if len(detections) == 0:
#            self.tag_visible = False
#            return
#
#        x = detections[0].transform.translation.x
#        rospy.loginfo("x: %f", x)
#
#        self.latest_x = x
#        self.tag_visible = True
#        self.last_tag_seen_time = rospy.Time.now()
#
#    def behavior_timer(self, event):
#        time_since_seen = rospy.Time.now() - self.last_tag_seen_time
#        cmd_msg = Twist2DStamped()
#        cmd_msg.header.stamp = rospy.Time.now()
#
#        if self.tag_visible and time_since_seen < self.tag_timeout:
#            center_threshold = 0.04
#
#            if abs(self.latest_x) < center_threshold:
#                cmd_msg.v = 0.0
#                cmd_msg.omega = 0.0
#                self.publish_action("Stopping")
#            else:
#                gain = 2.0
#                omega = -gain * self.latest_x
#
#                if omega < 0:
#                    omega *= 2  # Boost speed when turning right
#
#                omega = max(-6.5, min(omega, 2.5))
#                cmd_msg.v = 0.0
#                cmd_msg.omega = omega
#
#                if omega > 0:
#                    self.publish_action("Rotating left")
#                else:
#                    self.publish_action("Rotating right")
#        else:
#            # Intelligent seeking: spin in direction of last known tag position
#            cmd_msg.v = 0.0
#            if self.latest_x is not None:
#                cmd_msg.omega = -2.0 if self.latest_x > 0 else 3.0
#            else:
#                cmd_msg.omega = 3.0  # Default spin left
#
#            self.publish_action("Seeking object...")
#
#        self.cmd_vel_pub.publish(cmd_msg)
#
#    def publish_action(self, action_text):
#        if self.last_action != action_text:
#            rospy.loginfo(action_text)
#            print(action_text)
#            self.last_action = action_text
#
#if __name__ == '__main__':
#    try:
#        Target_Follower()
#    except rospy.ROSInterruptException:
#        pass




#******************5.2C************************

#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray

class Target_Follower:
    def __init__(self):
        rospy.init_node('target_follower_node', anonymous=True)
        rospy.on_shutdown(self.clean_shutdown)

        self.cmd_vel_pub = rospy.Publisher('/yumdoot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/yumdoot/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)

        self.latest_x = None
        self.latest_z = None
        self.tag_visible = False
        self.last_action = None

        # Distance control goal
        self.target_distance = 0.25  # meters
        self.distance_tolerance = 0.05  # meters
        rospy.Timer(rospy.Duration(0.1), self.behavior_timer)
        rospy.spin()

    def tag_callback(self, msg):
        if len(msg.detections) == 0:
            self.tag_visible = False
            return

        tag = msg.detections[0]
        x = tag.transform.translation.x
        z = tag.transform.translation.z  # forward distance to tag

        rospy.loginfo("x: %f, z: %f", x, z)

        self.latest_x = x
        self.latest_z = z
        self.tag_visible = True

    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def behavior_timer(self, event):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()

        if self.tag_visible and self.latest_x is not None and self.latest_z is not None:
            # --- ANGULAR CONTROL (Rotate to center the tag) ---
            center_threshold = 0.04
            angular_gain = 20
            if abs(self.latest_x) > center_threshold:
                cmd_msg.omega = -angular_gain * self.latest_x
            else:
                cmd_msg.omega = 0.0

            # --- LINEAR CONTROL (Move toward/far from tag) ---
            distance_error = self.latest_z - self.target_distance
            linear_gain = 2.5

            if abs(distance_error) > self.distance_tolerance:
                cmd_msg.v = linear_gain * distance_error
                cmd_msg.v = max(min(cmd_msg.v, 1.5), -1.5)  # clamp linear speed
            else:
                cmd_msg.v = 0.0

            self.publish_action(f"Following target | v={cmd_msg.v:.2f}, omega={cmd_msg.omega:.2f}")
        else:
            # No tag detected
            cmd_msg.v = 0.0
            cmd_msg.omega = 0.0
            self.publish_action("No tag detected. Stopping.")

        self.cmd_vel_pub.publish(cmd_msg)

    def publish_action(self, action_text):
        if self.last_action != action_text:
            rospy.loginfo(action_text)
            print(action_text)
            self.last_action = action_text

if __name__ == '__main__':
    try:
        Target_Follower()
    except rospy.ROSInterruptException:
        pass


