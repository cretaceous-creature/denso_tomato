#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class C2BaseController:
    def __init__(self):
        self.torque = None
        self.pub_torque = rospy.Publisher(
            '/currentor_socket/request/torque_vector',
            Float32MultiArray,
            queue_size = 10)
        self.wheel_torque = 40.0

    def torque_callback(self, msg):
        self.torque = msg.data

    def set_torque_callback(self, msg):
        if self.torque is None:
            rospy.logerr("torque vector is not subscribed!")
            return
        tq = list(self.torque)

        # calc direction
        # 21: right back, +
        # 22: left back, -
        # 23: right front, +
        # 24: left front, -
        direction = 'stop'
        vx = msg.linear.x
        vy = msg.linear.y
        vtheta = msg.angular.z

        tq[21] = 0.0
        tq[22] = 0.0
        tq[23] = 0.0
        tq[24] = 0.0

        if abs(vtheta) > 0.1:
            if vtheta > 0.0:
                direction = 'leftturn'
                tq[21] = self.wheel_torque
                tq[22] = self.wheel_torque
                tq[23] = self.wheel_torque
                tq[24] = self.wheel_torque
            else:
                direction = 'righttturn'
                tq[21] = -self.wheel_torque
                tq[22] = -self.wheel_torque
                tq[23] = -self.wheel_torque
                tq[24] = -self.wheel_torque
        elif abs(vx) > abs(vy):
            if vx > 0.0:
                direction = 'forward'
                tq[21] = self.wheel_torque
                tq[22] = -self.wheel_torque
                tq[23] = self.wheel_torque
                tq[24] = -self.wheel_torque
            else:
                direction = 'backward'
                tq[21] = -self.wheel_torque
                tq[22] = self.wheel_torque
                tq[23] = -self.wheel_torque
                tq[24] = self.wheel_torque
        elif abs(vy) > abs(vx):
            if vy > 0.0:
                direction = 'left'
                tq[21] = -self.wheel_torque
                tq[22] = -self.wheel_torque
                tq[23] = self.wheel_torque
                tq[24] = self.wheel_torque
            else:
                direction = 'right'
                tq[21] = self.wheel_torque
                tq[22] = self.wheel_torque
                tq[23] = -self.wheel_torque
                tq[24] = -self.wheel_torque

        msg = Float32MultiArray()
        msg.data = tq
        self.pub_torque.publish(msg)

if __name__ == '__main__':
    rospy.init_node('c2_base_control')
    base = C2BaseController()
    rospy.Subscriber('/currentor_socket/sensor_array/torque',
                     Float32MultiArray,
                     base.torque_callback)
    rospy.Subscriber('/c2/cmd_vel',
                     Twist,
                     base.set_torque_callback)
    rospy.spin()


