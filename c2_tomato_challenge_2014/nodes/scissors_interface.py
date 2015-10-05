#!/usr/bin/env python

# @biref control scissors finger
# copied from
# http://wiki.ros.org/dynamixel_controllers/Tutorials/Creatingdynamixelactionclient
#
# subscribe:
#  /scissors_controller/set_angle (std_msgs/Float64)
#   angle of servo hone in radian
#   - close: 0.2
#   - open: 0.7
#


import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from dynamixel_controllers.srv import SetTorqueLimit

class ScissorsJoint:
    def __init__(self):
        self.jta = actionlib.SimpleActionClient(
            '/scissors_trajectory_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')

    def move_joint(self, angle):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['scissors_finger']
        point = JointTrajectoryPoint()
        point.positions = [angle.data]
        point.time_from_start = rospy.Duration(1)
        goal.trajectory.points.append(point)
        self.jta.send_goal_and_wait(goal)


if __name__ == '__main__':
    rospy.init_node('joint_position_tester')

    # set torque limit first!
    tq_lim = 0.8
    rospy.wait_for_service('/scissors_controller/set_torque_limit')
    try:
        set_torque_limit = rospy.ServiceProxy(
            '/scissors_controller/set_torque_limit',
            SetTorqueLimit)
        set_torque_limit(tq_lim)
        rospy.loginfo('set finger torque to ' + str(tq_lim))
    except rospy.ServiceException, e:
        rospy.logerr('set_torque_limit error %s'%e)

    scissors = ScissorsJoint()
    rospy.Subscriber('/scissors_controller/set_angle', Float64,
                     scissors.move_joint)
    rospy.spin()



