#!/usr/bin/env python

import rospy
import geometry_msgs.msg
from std_msgs.msg import Float64

class Kinematics:
    left_wheel_vel = 0.0
    right_wheel_vel = 0.0
    def __init__(self):
        self.receiver = rospy.Subscriber(
            "/course_agv/velocity", geometry_msgs.msg.Twist,self.callback)
        self.vel_left_pub = rospy.Publisher(
            '/course_agv/left_wheel_velocity_controller/command',Float64, queue_size=10)
        self.vel_right_pub = rospy.Publisher(
            '/course_agv/right_wheel_velocity_controller/command',Float64, queue_size=10)

    def callback(self, data):
        vx = data.linear.x
        vw = data.angular.z
        self.left_wheel_vel,self.right_wheel_vel = self.kinematics(vx,vw)

    def kinematics(self,vx,vw):
        # too simple method , TODO
        return rx*vx-rw*vw,rx*vx+rw*vw

    def publish(self):
        self.vel_left_pub.publish(self.left_wheel_vel)
        self.vel_right_pub.publish(self.right_wheel_vel)

def main():
    node_name = "course_agv_kinematics"
    print("node : ",node_name)
    try:
        
        rospy.init_node(node_name)
        k = Kinematics()
        rate = rospy.Rate(rospy.get_param('~publish_rate',200))
        while not rospy.is_shutdown():
            k.publish()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
