#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped # Tipos de mensagens ROS

class Node:
    def __init__(self):
        rospy.init_node('Node_example')

        self.subscriber = rospy.Subscriber(
            "topic_to_subscribe",
            PoseStamped,
            self.callback,
            queue_size=10)

        self.publisher = rospy.Publisher(
            'topic_to_publish',
            Twist,
            queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(1/30), self.loop)

        rospy.loginfo('Node started')

    def callback(self, msg):
        self.msg = msg
    
    def loop(self, timer):
        print('Loop principal')

def main():
    Node()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()