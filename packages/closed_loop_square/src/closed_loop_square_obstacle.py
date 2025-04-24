
#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import Range

class ClosedLoopDriver:
    def __init__(self):
        rospy.init_node('closed_loop_square_node', anonymous=True)

        # Parameters
        self.ticks_per_meter = 750
        self.ticks_per_90_deg = 50

        self.left_ticks = 0
        self.right_ticks = 0
        self.start_left = 0
        self.start_right = 0

        self.autopilot_enabled = False
        self.obstacle_detected = False
        self.stopping_threshold = 0.30  # meters

        self.cmd = Twist2DStamped()
        self.pub = rospy.Publisher('/yumdoot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)

        rospy.Subscriber('/yumdoot/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_encoder_callback)
        rospy.Subscriber('/yumdoot/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_encoder_callback)
        rospy.Subscriber('/yumdoot/joy_mapper_node/joystick_override', Bool, self.autopilot_callback)
        rospy.Subscriber('/yumdoot/front_center_tof_driver_node/range', Range, self.tof_callback)

    def autopilot_callback(self, msg):
        self.autopilot_enabled = not msg.data
        rospy.loginfo("Autopilot enabled: %s", self.autopilot_enabled)

    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data

    def right_encoder_callback(self, msg):
        self.right_ticks = msg.data

    def tof_callback(self, msg):
        self.obstacle_detected = msg.range < self.stopping_threshold
        rospy.logdebug("ToF sensor reading: %.2f m | Obstacle detected: %s", msg.range, self.obstacle_detected)

    def reset_encoders(self):
        self.start_left = self.left_ticks
        self.start_right = self.right_ticks

    def stop(self):
        self.cmd.v = 0.0
        self.cmd.omega = 0.0
        self.pub.publish(self.cmd)

    def move_straight(self, distance_m, speed):
        target_ticks = abs(distance_m * self.ticks_per_meter)
        direction = 1 if distance_m >= 0 else -1

        self.reset_encoders()
        rospy.sleep(0.1)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            current_ticks = abs(((self.left_ticks - self.start_left) + (self.right_ticks - self.start_right)) / 2.0)
            if current_ticks >= target_ticks:
                break

            if self.obstacle_detected:
                rospy.loginfo("Obstacle detected! Pausing movement...")
                self.stop()
                while self.obstacle_detected and not rospy.is_shutdown():
                    rospy.sleep(0.1)
                rospy.loginfo("Obstacle cleared! Resuming movement...")

            self.cmd.v = direction * abs(speed)
            self.cmd.omega = 0.0
            self.pub.publish(self.cmd)
            rate.sleep()

        self.stop()
        rospy.loginfo("Move completed: %.2f meters", distance_m)

    def rotate_in_place(self, angle_deg, angular_speed):
        target_ticks = abs(angle_deg / 90.0 * self.ticks_per_90_deg)
        direction = 1 if angle_deg >= 0 else -1

        self.reset_encoders()
        rospy.sleep(0.1)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            current_ticks = abs((self.right_ticks - self.start_right) - (self.left_ticks - self.start_left)) / 2.0
            if current_ticks >= target_ticks:
                break

            self.cmd.v = 0.0
            self.cmd.omega = direction * abs(angular_speed)
            self.pub.publish(self.cmd)
            rate.sleep()

        self.stop()
        rospy.loginfo("Rotation completed: %.2f degrees", angle_deg)

    def drive_square(self, side_length=1.0, linear_speed=0.3, angular_speed=1.0):
        for i in range(4):
            rospy.loginfo("Driving side %d", i + 1)
            self.move_straight(side_length, linear_speed)
            rospy.sleep(0.2)
            self.rotate_in_place(90, angular_speed)
            rospy.sleep(0.2)
        rospy.loginfo("Completed Closed-Loop Square")

    def main_menu(self):
        rospy.loginfo("Waiting for autopilot to be enabled. Press 'l' to switch from manual...")
        while not rospy.is_shutdown() and not self.autopilot_enabled:
            rospy.sleep(1)

        rospy.loginfo("Autopilot enabled! Entering command menu...")

        while not rospy.is_shutdown():
            print("\n=== Closed Loop Driver Menu ===")
            print("1. Move Straight")
            print("2. Rotate In Place")
            print("3. Drive Square")
            print("4. Exit")
            choice = input("Select an option: ")

            if choice == '1':
                try:
                    dist = float(input("Enter distance in meters (negative for backward): "))
                    speed = float(input("Enter speed in m/s (positive): "))
                    self.move_straight(dist, speed)
                except ValueError:
                    print("Invalid input.")
            elif choice == '2':
                try:
                    angle = float(input("Enter angle in degrees: "))
                    speed = float(input("Enter angular speed in rad/s (positive): "))
                    self.rotate_in_place(angle, speed)
                except ValueError:
                    print("Invalid input.")
            elif choice == '3':
                try:
                    side = float(input("Enter square side length in meters: "))
                    lin_speed = float(input("Enter linear speed (m/s): "))
                    ang_speed = float(input("Enter angular speed (rad/s): "))
                    self.drive_square(side_length=side, linear_speed=lin_speed, angular_speed=ang_speed)
                except ValueError:
                    print("Invalid input.")
            elif choice == '4':
                rospy.signal_shutdown("User exited.")
                break
            else:
                print("Invalid option. Try again.")

if __name__ == '__main__':
    try:
        ClosedLoopDriver().main_menu()
    except rospy.ROSInterruptException:
        pass
