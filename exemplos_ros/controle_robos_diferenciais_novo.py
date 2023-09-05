#!/usr/bin/env python3

import rospy
import tf.transformations as tf
from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import Bool, String

from obstacle_avoidance_drone_follower.msg import OdometryAccelStamped, ObjectPoints, PointsList
from aurora_py.differential_robot_controller import differential_robot_controller
from aurora_py.obstacle_avoidance_2d import ObstacleAvoidance

import numpy as np

class DifferentialController:
    def __init__(self):
        rospy.init_node('differential_controller')

        self.pgains = [1.5, 1]
        self.a = 0.15

        self.x_dot, self.y_dot = 0.0, 0.0

        # Initializing emergency button to False
        self.btn_emergencia = False

        self.btn_emergencia_is_on = False
        self.robot_path_is_on = False
        self.robot_pose_is_on = False
        
        # Robot type
        robot_type = rospy.get_param('~robot_type', None)

        if robot_type is None:
            raise TypeError(f"You need to provide a robot type.")

        if robot_type == 'Solverbot':
            self.publisher = rospy.Publisher('/cmd_vel',
                                            Twist,
                                            queue_size=10)
        
        if robot_type == 'Pioneer':
            self.publisher = rospy.Publisher('/RosAria/cmd_vel',
                                            Twist,
                                            queue_size=10)
            
        self.namespace = rospy.get_namespace()

        # Subscribers for the drone's position and the desired path
        self.pose_subscriber = rospy.Subscriber(f"/vrpn_client_node{self.namespace}pose", 
                                                PoseStamped, 
                                                self.pose_callback)
        
        self.path_subscriber = rospy.Subscriber(f"{self.namespace}path", 
                                                    OdometryAccelStamped, 
                                                    self.path_callback)

        self.potential_subscriber = rospy.Subscriber(f"{self.namespace}potential", 
                                                    Point, 
                                                    self.potential_callback)           

        self.emergency_flag_subscriber = rospy.Subscriber('/emergency_flag',
                                            Bool,
                                            self.emergency_button_callback,
                                            queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(1/30), self.control_loop)

        rospy.loginfo('Differential robot navigation node started')

    def pose_callback(self, pose_data):
        self.robot_pose_is_on = True
        pose = pose_data.pose
        center_robot_x = pose.position.x
        center_robot_y = pose.position.y
        center_robot_z = pose.position.z

        orientation = pose.orientation
        robot_roll, robot_pitch, self.robot_yaw = tf.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        
        self.robot_x = center_robot_x + self.a*np.cos(self.robot_yaw)
        self.robot_y = center_robot_y + self.a*np.sin(self.robot_yaw)

    def path_callback(self, path_data):
        self.robot_path_is_on = True
        path_pose = path_data.pose
        path_velocity = path_data.twist

        self.path_x = path_pose.pose.position.x
        self.path_y = path_pose.pose.position.y

        self.path_linear_x = path_velocity.twist.linear.x
        self.path_linear_y = path_velocity.twist.linear.y
    
    def potential_callback(self, potential_data):
        self.x_dot = potential_data.x
        self.y_dot = potential_data.y

    def emergency_button_callback(self, emergency):
        self.btn_emergencia_is_on = True
        if emergency.data:
            self.btn_emergencia = True
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

            rospy.signal_shutdown("Pioneer Emergency stop")

    def control_loop(self, event):

        if self.robot_path_is_on == False or self.btn_emergencia_is_on == False or self.robot_pose_is_on ==False:
            rospy.loginfo('No path or emergency button found')
            return

        self.current_time = rospy.Time.now()
    
        if self.btn_emergencia:
            return

        x_dot, y_dot = self.x_dot, self.y_dot

        robot_pose = np.array([self.robot_x, self.robot_y, self.robot_yaw])

        desired = [        self.path_x        ,          self.path_y      , 
                   self.path_linear_x + x_dot , self.path_linear_y + y_dot]
        
        gains = self.pgains
        a = self.a

        reference_linear_velocity, reference_angular_velocity = differential_robot_controller(robot_pose, desired, gains=gains, limits=[0.5, 0.5], a=a)
        
        #rospy.loginfo('Linear Velocity: ' + str(reference_linear_velocity) + ', Angular Velocity: ' + str(reference_angular_velocity))

        ctrl_msg = Twist()
        ctrl_msg.linear.x = reference_linear_velocity
        ctrl_msg.linear.y = 0.0
        ctrl_msg.linear.z = 0.0
        ctrl_msg.angular.x = 0.0
        ctrl_msg.angular.y = 0.0
        ctrl_msg.angular.z = reference_angular_velocity
        self.publisher.publish(ctrl_msg)

def main():
    DifferentialController()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()