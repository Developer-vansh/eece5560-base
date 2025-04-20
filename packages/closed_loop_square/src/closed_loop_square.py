#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped

class ClosedLoopDriver:
    def __init__(self):
        rospy.init_node('closed_loop_square_node', anonymous=True)

        # Parameters (Tune these based on your measurements)
        self.ticks_per_meter = 750     # ticks count for straight 1 meter travel
        self.ticks_per_90_deg = 50    # ticks count for 90 degree rotation

        self.left_ticks = 0
        self.right_ticks = 0

        self.start_left = 0
        self.start_right = 0

        self.cmd = Twist2DStamped()
        self.pub = rospy.Publisher('/yumdoot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)

        rospy.Subscriber('/yumdoot/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_encoder_callback)
        rospy.Subscriber('/yumdoot/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_encoder_callback)

    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data

    def right_encoder_callback(self, msg):
        self.right_ticks = msg.data

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
            rospy.loginfo("Driving side %d", i+1)
            self.move_straight(side_length, linear_speed)
            rospy.sleep(0.2)
            self.rotate_in_place(90, angular_speed)
            rospy.sleep(0.2)
        rospy.loginfo("Completed Closed-Loop Square")

    def main_menu(self):
        while not rospy.is_shutdown():
            print("\n=== Closed Loop Driver Menu ===")
            print("1. Move Straight (Forward or Backward)")
            print("2. Rotate In Place (Left or Right)")
            print("3. Drive Square")
            print("4. Exit")
            choice = input("Select an option: ")

            if choice == '1':
                try:
                    dist = float(input("Enter distance in meters (negative for backward): "))
                    speed = float(input("Enter speed in m/s (always positive): "))
                    if speed <= 0:
                        print("Speed must be positive.")
                        continue
                    self.move_straight(dist, speed)
                except ValueError:
                    print("Invalid input. Please enter numeric values.")

            elif choice == '2':
                try:
                    angle = float(input("Enter angle in degrees (positive for left, negative for right): "))
                    speed = float(input("Enter angular speed in rad/s (always positive): "))
                    if speed <= 0:
                        print("Angular speed must be positive.")
                        continue
                    self.rotate_in_place(angle, speed)
                except ValueError:
                    print("Invalid input. Please enter numeric values.")

            elif choice == '3':
                try:
                    side = float(input("Enter side length of square in meters: "))
                    lin_speed = float(input("Enter linear speed in m/s: "))
                    ang_speed = float(input("Enter angular speed in rad/s: "))
                    if lin_speed <= 0 or ang_speed <= 0:
                        print("Speeds must be positive.")
                        continue
                    self.drive_square(side_length=side, linear_speed=lin_speed, angular_speed=ang_speed)
                except ValueError:
                    print("Invalid input. Please enter numeric values.")

            elif choice == '4':
                rospy.signal_shutdown("User exited.")
                break

            else:
                print("Invalid option. Try again.")

if __name__ == '__main__':
    try:
        driver = ClosedLoopDriver()
        rospy.sleep(2)  # wait for encoder values to come in
        driver.main_menu()

    except rospy.ROSInterruptException:
        pass

