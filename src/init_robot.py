#! /usr/bin/env python

import rospy
from bag_manipulation.srv import *
from kuka import *

rospy.init_node('init_robot_service')
kuka=Environment()


def execute_step(request):
        print request
	kuka.init_robot()
        kuka.home()
	return ExcStepResponse(success=True, message="Robot initialized")


action_server=rospy.Service('/bag_manipulation/init_robot',ExcStep, execute_step)
rospy.loginfo("init_robot_service started")
rospy.spin()
