#! /usr/bin/env python

import rospy
from bag_manipulation.srv import *
from kuka import *

rospy.init_node('step_4_robot_service')
kuka=Environment()


def execute_step(request):
        print primitive_actions['3'][int(request.action)]
	kuka.step_4(primitive_actions['3'][int(request.action)])
	return ExcStepResponse(success=True, message="Action excecuted")


action_server=rospy.Service('/bag_manipulation/step_4',ExcStep, execute_step)
rospy.loginfo("step_4_robot_service started")
rospy.spin()
