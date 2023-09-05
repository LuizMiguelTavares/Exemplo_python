#!/usr/bin/env python3

import rospy
import tf.transformations as tf
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from obstacle_avoidance.obstacle_avoidance import ObstacleAvoidance
from math import cos, sin, pi
import numpy as np
import time
import math

import csv
import os


class PioneerController:
    def __init__(self):
        rospy.init_node('pioneer_controller')
        
        self.prev_pose = None
        self.prev_time = None
        self.start_time = None

        self.pgains = [1.5, 1, 1.5, 1]
        self.a = 0.15

        # Initializing emergency button to False
        self.btn_emergencia = False
        
        # Initializing path variables
        self.path_x = 0.0
        self.path_y = 0.0
        self.path_z = 0.0

        self.path_linear_x = 0.0
        self.path_linear_y = 0.0
        self.path_linear_z = 0.0

        self.path_angular_x = 0.0
        self.path_angular_y = 0.0
        self.path_angular_z = 0.0
        
        #### OBSTACULO
        self.obstacle_avoidance = None
        
        # Pioneer
        self.robot_heigth = 0.7/2
        self.robot_width = 0.6/2
        #REAL
        # 45x51
        
        # SolverBot 
        self.solver_bot_heigth = 0.7/2
        self.solver_bot_width = 0.6/2

        # Obstacle
        self.obstacle = False # Sets if the obstacle counts
        self.solver_bot = False # Sets if the solver_bot is online counts
        
        self.obs_height = 0.3/2
        self.obs_width  = 0.3/2

        # Starting Obstacle Avoidance Class
        self.obs_avoidance = ObstacleAvoidance(n=4, a=0.35, b=0.35, k=1.0)
        # self.obs_avoidance = ObstacleAvoidance(n=4, a=0.2, b=0.2, k=0.4) # Não funciona bem
        
        # Specify the folder path where you want to create the file
        folder_path = '/root/data/'

        # Create the file in the specified folder
        file_path = os.path.join(folder_path, 'obstacle_detection_pioneer.csv')

        self.csv_file = open(file_path, 'w')

        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time', 'X til',
                                  'X_robot_solver', 'Y_robot_solver', 
                                  'X_solver_robot', 'Y_solver_robot', 
                                  'X_robot_obs', 'Y_robot_obs', 
                                  'X_obs_robot', 'Y_obs_robot', 
                                  'X_dot', 'Y_dot'])
        
        self.publisher = rospy.Publisher(
            '/RosAria/cmd_vel',
            Twist,
            queue_size=10)

        self.pose_subscriber = rospy.Subscriber(
            "/vrpn_client_node/P1/pose",
            PoseStamped,
            self.RobotPose,
            queue_size=40)
        
        self.pose_subscriber = rospy.Subscriber(
            "/vrpn_client_node/OBS/pose",
            PoseStamped,
            self.obstacle_pose,
            queue_size=40)

        self.subscription = rospy.Subscriber(
            'path_pose',
            Odometry,
            self.Path,
            queue_size=10)
        
        self.subscription = rospy.Subscriber(
            'solver_bot_odom',
            Odometry,
            self.solver_bot_path,
            queue_size=10)

        self.subscription = rospy.Subscriber(
            'emergency_flag',
            Bool,
            self.emergency_button_callback,
            queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(1/30), self.control_loop)

        rospy.loginfo('Pioneer navigation node started')

    def RobotPose(self, msg):

        # Process the pose data
        pose = msg.pose

        # Process the pose data
        self.center_robot_x = pose.position.x
        self.center_robot_y = pose.position.y
        self.center_robot_z = pose.position.z

        orientation = pose.orientation
        self.robot_roll, self.robot_pitch, self.robot_yaw = tf.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        
        self.robot_x = self.center_robot_x + self.a*np.cos(self.robot_yaw)
        self.robot_y = self.center_robot_y + self.a*np.sin(self.robot_yaw)

        if self.prev_pose is not None and self.prev_time is not None:
            current_time = time.time()
            time_diff = current_time - self.prev_time

            # Calculate linear velocity
            self.robot_linear_x = (msg.pose.position.x - self.prev_pose.position.x) / time_diff
            self.robot_linear_y = (msg.pose.position.y - self.prev_pose.position.y) / time_diff
            self.robot_linear_z = (msg.pose.position.z - self.prev_pose.position.z) / time_diff

            # Convert Euler angles (roll, pitch, yaw) to angular velocities
            euler_diff = self.calculate_euler_diff(msg.pose.orientation, self.prev_pose.orientation)

            # Calculate angular velocity
            self.robot_roll_velocity = euler_diff[0] / time_diff
            self.robot_pitch_velocity = euler_diff[1] / time_diff
            self.robot_yaw_velocity = euler_diff[2] / time_diff

        self.prev_pose = msg.pose
        self.prev_time = time.time()
    
    
    def obstacle_pose(self, msg):
        self.obstacle_avoidance = True
        self.obstacle = True
        
        # Process the pose data
        pose = msg.pose

        # Process the pose data
        obstacle_x = pose.position.x
        obstacle_y = pose.position.y
        obstacle_z = pose.position.z

        orientation = pose.orientation
        obstacle_roll, obstacle_pitch, obstacle_yaw = tf.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        
        self.obstacle_pose = [obstacle_x, obstacle_y, obstacle_yaw, self.obs_height, self.obs_width]


    def calculate_euler_diff(self, current_orientation, previous_orientation):
        # Convert the quaternion objects to lists
        current_quaternion = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]
        previous_quaternion = [previous_orientation.x, previous_orientation.y, previous_orientation.z, previous_orientation.w]

        # Calculate the Euler angle differences
        current_euler = tf.euler_from_quaternion(current_quaternion)
        previous_euler = tf.euler_from_quaternion(previous_quaternion)
        euler_diff = [
            self.normalize_angle(current_euler[0] - previous_euler[0]),
            self.normalize_angle(current_euler[1] - previous_euler[1]),
            self.normalize_angle(current_euler[2] - previous_euler[2])
        ]

        return euler_diff


    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


    def Path(self, msg):
        pose = msg.pose.pose
        velocity = msg.twist.twist

        # Process the pose and velocity data
        position = pose.position
        orientation = pose.orientation

        linear_velocity = velocity.linear
        angular_velocity = velocity.angular

        # Access specific components of the pose and velocity
        self.path_x = position.x
        self.path_y = position.y
        self.path_z = position.z

        # Convert the transformed orientation quaternion to Euler angles
        roll, pitch, yaw = tf.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )

        self.path_linear_x = linear_velocity.x
        self.path_linear_y = linear_velocity.y
        self.path_linear_z = linear_velocity.z

        self.path_angular_x = angular_velocity.x
        self.path_angular_y = angular_velocity.y
        self.path_angular_z = angular_velocity.z


    def solver_bot_path(self, msg):
        
        self.obstacle_avoidance = True
        self.solver_bot = True
        pose = msg.pose.pose
        velocity = msg.twist.twist

        # Process the pose and velocity data
        position = pose.position
        orientation = pose.orientation

        linear_velocity = velocity.linear
        angular_velocity = velocity.angular

        # Access specific components of the pose and velocity
        self.solver_bot_x = position.x
        self.solver_bot_y = position.y
        self.solver_bot_z = position.z

        # Convert the transformed orientation quaternion to Euler angles
        roll, pitch, self.solver_bot_yaw = tf.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )

        self.solver_bot_linear_x = linear_velocity.x
        self.solver_bot_linear_y = linear_velocity.y
        self.solver_bot_linear_z = linear_velocity.z

        self.solver_bot_angular_x = angular_velocity.x
        self.solver_bot_angular_y = angular_velocity.y
        self.solver_bot_angular_z = angular_velocity.z


    def emergency_button_callback(self, msg):
        if msg.data:
            self.btn_emergencia = True


    def control_loop(self, event):

        self.current_time = rospy.Time.now()
        
        
        if self.start_time == None:
            self.start_time = self.current_time
        
        # Calculate elapsed time
        self.elapsed_time = (self.current_time - self.start_time).to_sec()
            
    
        if self.btn_emergencia:
            rospy.loginfo('Robot stopping by Emergency')
            rospy.loginfo('Sending emergency stop command')

            for _ in range(10):
                stop_cmd = Twist()
                stop_cmd.linear.x = 0.0
                stop_cmd.linear.y = 0.0
                stop_cmd.linear.z = 0.0
                stop_cmd.angular.x = 0.0
                stop_cmd.angular.y = 0.0
                stop_cmd.angular.z = 0.0
                # Publish the Twist message to stop the robot
                self.publisher.publish(stop_cmd)

            rospy.signal_shutdown("Emergency stop")
        
        if self.prev_pose is None:
            return
        
        desired_linear_velocity, desired_angular_velocity = self.controller()
        
        #rospy.loginfo('Linear Velocity: ' + str(desired_linear_velocity) + ', Angular Velocity: ' + str(desired_angular_velocity))

        ctrl_msg = Twist()
        ctrl_msg.linear.x = desired_linear_velocity
        ctrl_msg.linear.y = 0.0
        ctrl_msg.linear.z = 0.0
        ctrl_msg.angular.x = 0.0
        ctrl_msg.angular.y = 0.0
        ctrl_msg.angular.z = desired_angular_velocity
        # Publish the Twist message to control the robot
        self.publisher.publish(ctrl_msg)


    def controller(self):
        Kp = np.array([[self.pgains[0], 0],
                       [0, self.pgains[1]]])

        K = np.array([[np.cos(self.robot_yaw), -self.a*np.sin(self.robot_yaw)],
                      [np.sin(self.robot_yaw), self.a*np.cos(self.robot_yaw)]])

        Xtil = np.array([self.path_x - self.robot_x, self.path_y - self.robot_y]) 
        
        if self.obstacle_avoidance:
                
            robot_pose = [self.center_robot_x, self.center_robot_y, self.robot_yaw, self.robot_heigth, self.robot_width ]

            if self.obstacle:
                obs_x_dot, obs_y_dot, pose_robot_obs, pose_obs_robot = self.obs_avoidance.obstacle_avoidance(robot_pose, self.obstacle_pose)
            else:
                obs_x_dot, obs_y_dot, pose_robot_obs, pose_obs_robot = 0.0, 0.0, [None, None], [None, None]
            
            if self.solver_bot:
                solver_bot_pose = [self.solver_bot_x, self.solver_bot_y, self.solver_bot_yaw, self.solver_bot_heigth, self.solver_bot_width]
                solver_bot_x_dot, solver_bot_y_dot, pose_robot_solver, pose_solver_robot = self.obs_avoidance.obstacle_avoidance(robot_pose, solver_bot_pose)
            else:
                solver_bot_x_dot, solver_bot_y_dot, pose_robot_solver, pose_solver_robot = 0.0, 0.0, [None, None], [None, None]
            
            # Somatória dos pesos dos obstáculos
            vobs = np.array([obs_x_dot + solver_bot_x_dot, obs_y_dot + solver_bot_y_dot])

            # Write the data to the CSV file
            # ['Time', 'X_robot_solver', 'Y_robot_solver', 'X_solver_robot', 'Y_solver_robot', 'X_robot_obs', 'Y_robot_obs', 'X_obs_robot', 'Y_obs_robot', 'X_dot', 'Y_dot']
            self.csv_writer.writerow([self.current_time.to_sec(), np.linalg.norm(Xtil),
                                      pose_robot_solver[0], pose_robot_solver[1], 
                                      pose_solver_robot[0], pose_solver_robot[1],
                                      pose_robot_obs[0], pose_robot_obs[1], 
                                      pose_obs_robot[0], pose_obs_robot[1], 
                                      vobs[0], vobs[1]])
        
            nu_obs = (1 - np.abs(np.tanh(vobs)))
            
            # Ganhos do controlador 
            k1 = 0.3 # Ganho do Xtil
            k2 = 1.5 # Ganho do obstáculo
            k3 = 1   # GAnho interno do obstáculo

            Xtil = np.tanh(Xtil) * nu_obs.T * k1
            path_velocity = np.array([self.path_linear_x, self.path_linear_y]) * nu_obs.T
            obs_velocity = k2 * np.tanh(k3*vobs)
            
            desired_velocity = path_velocity + Xtil + obs_velocity
            #rospy.loginfo('Desired velocity' + str(np.round(desired_velocity, 3)))
            reference_velocity = np.dot(np.linalg.inv(K), desired_velocity)

        else:
            desired_velocity = np.array([self.path_linear_x, self.path_linear_y])
            reference_velocity = np.dot(np.linalg.inv(K), (desired_velocity.T + np.dot(Kp, Xtil.T)))

        desired_linear_velocity = reference_velocity[0]
        desired_angular_velocity = reference_velocity[1]

        if np.abs(desired_linear_velocity) > 0.5:
            desired_linear_velocity = np.sign(desired_linear_velocity)*0.5

        if np.abs(desired_angular_velocity) > 0.5:
            desired_angular_velocity = np.sign(desired_angular_velocity)*0.5

        return desired_linear_velocity, desired_angular_velocity


def main():
    rospy.init_node('pioneer_controller')

    controller = PioneerController()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()