#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt, fabs, pi
import time

class DarwinController:
    def __init__(self):
        rospy.init_node('darwin_controller', anonymous=True)
        
        rospy.Subscriber('/my_odom', Odometry, self.update_pose)
        self.cmd_vel_pub = rospy.Publisher('/darwin/cmd_vel', Twist, queue_size=10)
        
        self.pose = Odometry()
        self.rate = rospy.Rate(10)

        # PID parameters for x direction
        self.kp_linear_x = 0.15
        self.ki_linear_x = 0.0
        self.kd_linear_x = 0.1

        # PID parameters for y direction
        self.kp_linear_y = 0.15
        self.ki_linear_y = 0.0
        self.kd_linear_y = 0.1

        # Maximum velocities
        self.max_linear_vel = 1.5

        # Previous error for derivative term
        self.prev_linear_error_x = 0.0
        self.prev_linear_error_y = 0.0

        # Integral term for accumulating errors
        self.integral_linear_error_x = 0.0
        self.integral_linear_error_y = 0.0

        # Maximum time for reaching the goal
        self.max_time_to_goal = 200  # seconds

        # Tolerances for x and y directions
        self.tolerance_x = 0.2  # meters
        self.tolerance_y = 0.2  # meters

        # Maximum iterations without progress
        self.max_iterations_without_progress = 250

    def update_pose(self, data):
        self.pose = data

    def move_to_goal(self, x_goal, y_goal):
        goal_pose = Odometry()
        goal_pose.pose.pose.position.x = x_goal
        goal_pose.pose.pose.position.y = y_goal

        start_time = time.time()
        iteration = 0

        while True:
            # Check if max time to reach the goal has elapsed
            if time.time() - start_time > self.max_time_to_goal:
                rospy.loginfo("Max time reached. Unable to reach goal.")
                break

            # Calculate errors
            linear_error_x = goal_pose.pose.pose.position.x - self.pose.pose.pose.position.x
            linear_error_y = goal_pose.pose.pose.position.y - self.pose.pose.pose.position.y

            # Integral term
            self.integral_linear_error_x += linear_error_x
            self.integral_linear_error_y += linear_error_y

            # PID control for linear velocity in x direction
            linear_vel_x = self.kp_linear_x * linear_error_x + self.ki_linear_x * self.integral_linear_error_x + self.kd_linear_x * (linear_error_x - self.prev_linear_error_x)
            linear_vel_x = min(max(linear_vel_x, -self.max_linear_vel), self.max_linear_vel)

            # PID control for linear velocity in y direction
            linear_vel_y = self.kp_linear_y * linear_error_y + self.ki_linear_y * self.integral_linear_error_y + self.kd_linear_y * (linear_error_y - self.prev_linear_error_y)
            linear_vel_y = min(max(linear_vel_y, -self.max_linear_vel), self.max_linear_vel)

            # Update previous errors
            self.prev_linear_error_x = linear_error_x
            self.prev_linear_error_y = linear_error_y

            # Publish the velocity message
            vel_msg = Twist()
            vel_msg.linear.x = linear_vel_x
            vel_msg.linear.y = linear_vel_y
            self.cmd_vel_pub.publish(vel_msg)
            

            # Sleep for the specified rate
            self.rate.sleep()

            # Check if both x and y errors are within tolerances
            if abs(linear_error_x) <= self.tolerance_x and abs(linear_error_y) <= self.tolerance_y:
                rospy.loginfo("Reached the goal.")
                break

            iteration += 1

        # Stop the robot when the goal is reached
        self.stop_robot()
        rospy.loginfo("stopped")

    def stop_robot(self):
        # Publish a Twist message with zero velocities to stop the robot
        vel_msg = Twist()
        self.cmd_vel_pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        controller = DarwinController()
        while not rospy.is_shutdown():
            x_goal = float(input("Enter the x coordinate of the goal: "))
            y_goal = float(input("Enter the y coordinate of the goal: "))
            rospy.loginfo("Moving to goal: ({}, {})".format(x_goal, y_goal))
           
            controller.move_to_goal(x_goal, y_goal)
    except rospy.ROSInterruptException:
        pass