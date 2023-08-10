#! /usr/bin/env python

import rospy
from bag_manipulation.srv import *
from kuka import *

rospy.init_node('step_1_robot_service')
kuka=Environment()


def execute_step(request):
        print primitive_actions['0'][int(request.action)]
	kuka.step_1(primitive_actions['0'][int(request.action)][0],primitive_actions['0'][int(request.action)][1])
	return ExcStepResponse(success=True, message="Action excecuted")


action_server=rospy.Service('/bag_manipulation/step_1',ExcStep, execute_step)
rospy.loginfo("step_1_robot_service started")
rospy.spin()
