#! /usr/bin/env python

import rospy
from bag_manipulation.srv import *
from kuka import *

rospy.init_node('step_3_robot_service')
kuka=Environment()


def execute_step(request):
        print primitive_actions['2'][int(request.action)]
	kuka.step_3(primitive_actions['2'][int(request.action)])
	return ExcStepResponse(success=True, message="Action excecuted")


action_server=rospy.Service('/bag_manipulation/step_3',ExcStep, execute_step)
rospy.loginfo("step_3_robot_service started")
rospy.spin()
