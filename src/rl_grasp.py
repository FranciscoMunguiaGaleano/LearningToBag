#!/usr/bin/env python

import rospy
import numpy as np
from copy import deepcopy as dcp
from std_msgs.msg import Bool,Float32,String
from geometry_msgs.msg import PoseStamped, PoseArray
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotInput as inputMsg
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput as outputMsg
import time
import yaml
import random
import os, sys
#from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
#from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg


LOCAL_PATH=os.path.dirname(os.path.abspath('__file__'))
MODEL_PATH=LOCAL_PATH+"/src/bag_manipulation/src/TEMP/MODEL"
RESULTS_PATH=LOCAL_PATH+"/src/bag_manipulation/src/TEMP/RESULTS"
GRASP_ADJUST=0.262

waiting_pose = PoseStamped()
waiting_pose.pose.position.x = 0.0
waiting_pose.pose.position.y = -0.55
waiting_pose.pose.position.z = 0.6

waiting_pose.pose.orientation.w = 0.0000
waiting_pose.pose.orientation.x = -0.7071067811
waiting_pose.pose.orientation.y = 0.7071067811
waiting_pose.pose.orientation.z = 0.0000

pre_grasping_pose = PoseStamped()
pre_grasping_pose.pose.position.x = -0.55
pre_grasping_pose.pose.position.y = 0.0
pre_grasping_pose.pose.position.z = 0.45

pre_grasping_pose.pose.orientation.w = 0.0000
pre_grasping_pose.pose.orientation.x = 0.0000
pre_grasping_pose.pose.orientation.y = 1.0000
pre_grasping_pose.pose.orientation.z = 0.0000


DISTANCE_THRESHOLD = 0.001

gripper_activation = outputMsg()
gripper_activation.rACT = 1
gripper_activation.rGTO = 1
gripper_activation.rMOD = 1
gripper_activation.rSPA = 250
gripper_activation.rFRA = 150#

gripper_close = outputMsg()
gripper_close.rACT = 1
gripper_close.rGTO = 1
gripper_close.rMOD = 1
gripper_close.rPRA = 255
gripper_close.rSPA = 200
gripper_close.rFRA = 150

gripper_half = outputMsg()
gripper_half.rACT = 1
gripper_half.rGTO = 1
gripper_half.rMOD = 1
gripper_half.rPRA = 100
gripper_half.rSPA = 200
gripper_half.rFRA = 150

gripper_pre_1 = outputMsg()
gripper_pre_1.rACT = 1
gripper_pre_1.rGTO = 1
gripper_pre_1.rMOD = 1
gripper_pre_1.rPRA = 90
gripper_pre_1.rSPA = 200
gripper_pre_1.rFRA = 150

gripper_pre_2 = outputMsg()
gripper_pre_2.rACT = 1
gripper_pre_2.rGTO = 1
gripper_pre_2.rMOD = 1
gripper_pre_2.rPRA = 65
gripper_pre_2.rSPA = 200
gripper_pre_2.rFRA = 150

gripper_pre_3 = outputMsg()
gripper_pre_3.rACT = 1
gripper_pre_3.rGTO = 1
gripper_pre_3.rMOD = 1
gripper_pre_3.rPRA = 50
gripper_pre_3.rSPA = 200
gripper_pre_3.rFRA = 150

gripper_open = outputMsg()
gripper_open.rACT = 1
gripper_open.rGTO = 1
gripper_open.rMOD = 1
gripper_open.rPRA = 0
gripper_open.rSPA = 200
gripper_open.rFRA = 150

gripper_reset = outputMsg()
gripper_reset.rACT = 0
gripper_reset.rGTO = 0
gripper_reset.rMOD = 0
gripper_reset.rPRA = 0
gripper_reset.rSPA = 0
gripper_reset.rFRA = 0


class Environment:
    def __init__(self):
        rospy.init_node('controller_node', anonymous=True)
        rospy.Subscriber('/iiwa/state/CartesianPose', PoseStamped, callback=self.current_pose_callback)
	self.bag=Bag()
        self.pub_move_cmd = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=2)
        self.pub_gripper_cmd = rospy.Publisher('grasping_bag/Robotiq3FGripperRobotOutput', outputMsg, queue_size=2)
        self.pub_attempt_finished = rospy.Publisher('AttemptFinished', Bool, queue_size=2)
        self.current_pose_msg = PoseStamped()
        self.current_xyz = np.array([0.0, 0.0, 0.0])
        self.current_header_seq = 0
        self.trans=tf.TransformBroadcaster()
        self.draging=[0,0,0]
        self.init_robot()

    def get_wrist_angle(self,bag_x,bag_y,rim_x,rim_y):
        if bag_x<rim_x and bag_y>rim_y:
            theta=-math.atan(abs(bag_y-rim_y)/abs(bag_x-rim_x))
            return theta
        if bag_x<rim_x and bag_y<rim_y:
            theta=math.atan(abs(bag_y-rim_y)/abs(bag_x-rim_x))
            return theta
        if bag_x>rim_x and bag_y>rim_y:
            theta=math.atan(abs(bag_y-rim_y)/abs(bag_x-rim_x))
            return theta
        if bag_x>rim_x and bag_y<rim_y:
            theta=-math.atan(abs(bag_y-rim_y)/abs(bag_x-rim_x))
            return theta
        return 0.0
    def get_pose(self,frame):
        listener = tf.TransformListener()
        trans = []
        rot = []
        rate = rospy.Rate(1000.0)
	attemps=0
        while not trans and not rospy.is_shutdown():		
            try:
		attemps+=1
                (trans,rot) = listener.lookupTransform('/iiwa_link_0', frame, rospy.Time())
		
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		if attemps==25000:
			print "tf Not-found"
			return None, None
		else:
                	continue
	    rate.sleep()
        #print(trans,rot)
        return trans,rot
    def get_angles(self,pos,wrist):
        euler=[0,0,0]
        euler[0]=3.14159
        euler[2]=math.atan(pos[1]/pos[0])-1.5707+wrist
        euler[2]=-1.5707+wrist
        #print(euler[2]*360/(2*3.1416))
        return euler
    def get_angles_lift(self,pos,wrist):
        euler=[0,0,0]
        #euler[0]=3.14159-1.0707
	euler[0]=3.14159-0.5060
        euler[2]=math.atan(pos[1]/pos[0])-1.5707+wrist
        euler[2]=-1.5707+wrist
        #print(euler[2]*360/(2*3.1416))
        return euler
    def get_message_pose(self,pos,euler,height):
        grasping_pose = PoseStamped()
        grasping_pose.pose.position.x = pos[0]
        grasping_pose.pose.position.y = pos[1]
        grasping_pose.pose.position.z = pos[2]+height
        q=quaternion_from_euler(euler[0],euler[1],euler[2])
        grasping_pose.pose.orientation.w = q[3]
        grasping_pose.pose.orientation.x = q[0]
        grasping_pose.pose.orientation.y = q[1]
        grasping_pose.pose.orientation.z = q[2]
        return grasping_pose
    def publish_mock_flenge(self,pos,euler):
        rate=rospy.Rate(100)
        while not rospy.is_shutdown():
            q=quaternion_from_euler(euler[0],euler[1],euler[2])
            self.trans.sendTransform((pos[0],pos[1],pos[2]+0.42),(q[0],q[1],q[2],q[3]),rospy.Time.now(),"mock_flange","iiwa_link_0")
            rate.sleep()
    def init_robot(self):
        rospy.loginfo("Robot ready...")
        self.publish_grip_cmd(gripper_reset)
        self.publish_grip_cmd(gripper_activation)

    def current_pose_callback(self, data):
        self.current_pose_msg = data
        self.current_xyz = np.array([
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z
        ])
        self.current_header_seq = data.header.seq

    def publish_pose(self, data):
        # record target xyz for distance tracking
        target_xyz = np.array([
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z
        ])
        data.header.seq = self.current_header_seq
        data.header.stamp = rospy.Time.now()
        data.header.frame_id = 'iiwa_link_0'
        rospy.sleep(0.5)
        self.pub_move_cmd.publish(data)
        done = False
        while not done and not rospy.is_shutdown():
            d = np.sqrt(np.sum(np.square(self.current_xyz - target_xyz)))
            if d < DISTANCE_THRESHOLD:
                rospy.loginfo("Movement finished")
                done = True

    def publish_grip_cmd(self, data):
        self.pub_gripper_cmd.publish(data)
        rospy.sleep(1)

    def step_1(self,lift_point,height):
        gram_m=0.26
	pos,rot=self.get_pose("/"+lift_point)
	#exit()
        euler=self.get_angles(pos,0)
        if pos[2]<0.0:
            gram_m=0.27
        else:
            gram_m=0.26
            #####
        self.publish_pose(waiting_pose)
        pos[2]=0.002
        grasping_pose=self.get_message_pose(pos,euler,0.35)
        self.publish_pose(grasping_pose)
        self.publish_grip_cmd(gripper_half)
        grasping_pose=self.get_message_pose(pos,euler,0.267)
        self.publish_pose(grasping_pose)
        self.publish_grip_cmd(gripper_pre_2)
        grasping_pose=self.get_message_pose(pos,euler,0.27)
        self.publish_pose(grasping_pose)
        if (1.18-pos[2])<0.00:
            gram_m=0.27
        else:
            gram_m=0.27
        grasping_pose=self.get_message_pose(pos,euler,0.26)
        self.publish_pose(grasping_pose)
        self.publish_grip_cmd(gripper_pre_1)
        grasping_pose=self.get_message_pose(pos,euler,0.28)
        self.publish_pose(grasping_pose)
        self.publish_grip_cmd(gripper_pre_2)
        grasping_pose=self.get_message_pose(pos,euler,0.26)
        self.publish_pose(grasping_pose)
        self.publish_grip_cmd(gripper_pre_1)
        grasping_pose=self.get_message_pose(pos,euler,gram_m)
        self.publish_pose(grasping_pose)
        self.publish_grip_cmd(gripper_close)

        pos[0]=-0.5
        pos[1]=0.0
            #euler=self.get_angles_lift(pos,0)
	euler=self.get_angles_lift(pos,0)
        grasping_pose=self.get_message_pose(pos,euler,0.35+height*0.033)
        self.publish_pose(grasping_pose)
        self.publish_grip_cmd(gripper_open)
            #Open gripper
        self.publish_pose(waiting_pose)
        time.sleep(3)
        return self.bag.state,self.reward_state_0(self.bag.bag_area),False

    def step_2(self,scratch_point,open_point):
        self.publish_pose(waiting_pose)
        pos,rot=self.get_pose("/opening_bag")
	if pos is None:
		return
	print("aqui")
        pos_c,rot_c=self.get_pose("/bag")
	if pos_c is None:
		return
        wrist=self.get_wrist_angle(pos_c[0],pos_c[1],pos[0],pos[1])
        #euler=self.get_angles(pos,wrist)
        #euler_c=self.get_angles(pos,wrist)
        #####
        pos,rot=self.get_pose("/"+scratch_point)
	if pos is None:
		return
        pos_c,rot_c=self.get_pose("/"+open_point)
	if pos_c is None:
		return
	pos_d,rot_d=self.get_pose("/51")
	if pos_d is None:
		return
	
        #wrist=self.get_wrist_angle(pos_d[0],pos_d[1],pos[0],pos[1])
        euler=self.get_angles(pos,wrist)
        euler_c=self.get_angles(pos,wrist)
        pos[2]=0.002
        grasping_pose=self.get_message_pose(pos,euler,0.35)
        self.publish_pose(grasping_pose)
        self.publish_grip_cmd(gripper_close)
        grasping_pose=self.get_message_pose(pos,euler,0.26)
        self.publish_pose(grasping_pose)
        self.publish_grip_cmd(gripper_pre_2)
        grasping_pose=self.get_message_pose(pos,euler,0.27)
        self.publish_pose(grasping_pose)
        #time.sleep(1)

        if (1.18-pos[2])<0.00:
            gram_m=0.277
        else:
            gram_m=0.277
        grasping_pose=self.get_message_pose(pos,euler,GRASP_ADJUST)
        self.publish_pose(grasping_pose)
        self.publish_grip_cmd(gripper_pre_1)
        grasping_pose=self.get_message_pose(pos,euler,0.28)
        self.publish_pose(grasping_pose)
        self.publish_grip_cmd(gripper_pre_2)
        grasping_pose=self.get_message_pose(pos,euler,GRASP_ADJUST)
        self.publish_pose(grasping_pose)
        self.publish_grip_cmd(gripper_pre_1)
        grasping_pose=self.get_message_pose(pos,euler,gram_m)
        self.publish_pose(grasping_pose)
        self.publish_grip_cmd(gripper_close)

        pos[2]=0.001
        grasping_pose=self.get_message_pose(pos,euler,0.3)
        self.publish_pose(grasping_pose)
        grasping_pose=self.get_message_pose(pos_c,euler_c,0.3)
        self.publish_pose(grasping_pose)
        pos_c[2]=0.002
        grasping_pose=self.get_message_pose(pos_c,euler_c,0.26)
        self.draging=pos_c
        self.publish_pose(grasping_pose)
        self.publish_grip_cmd(gripper_half)
	#self.publish_grip_cmd(gripper_open)
        grasping_pose=self.get_message_pose(pos_c,euler_c,0.35)
        self.publish_pose(grasping_pose)
        self.publish_grip_cmd(gripper_open)
        self.publish_pose(waiting_pose)
	time.sleep(2)
	return self.bag.state,self.reward_state_1(self.bag.opening_area),False
    def step_3(self,place_point):
        print "here"
        self.publish_pose(waiting_pose) 
	
	pos_obj,rot_obj=self.get_pose("/object")
	if pos_obj is None:
		return
        pos1,rot1=self.get_pose("/"+place_point)
	if pos1 is None:
		return
	
        pos_obj=[pos_obj[0],pos_obj[1],0.0]
        euler=self.get_angles(pos_obj,0.0)
        grasping_pose=self.get_message_pose(pos_obj,euler,0.35)
        self.publish_pose(grasping_pose)
#1/0
        self.publish_grip_cmd(gripper_open)
        grasping_pose=self.get_message_pose(pos_obj,euler,0.26)
        self.publish_pose(grasping_pose)
        self.publish_grip_cmd(gripper_close)
        grasping_pose=self.get_message_pose(pos_obj,euler,0.35)
        self.publish_pose(grasping_pose)
###

        ##pos_c,rot_c=self.get_pose("/bag")
        ##wrist=self.get_wrist_angle(pos_c[0],pos_c[1],pos[0],pos[1])
        ##euler=self.get_angles(pos,wrist)
        ##euler_c=self.get_angles(pos,wrist)
	euler=self.get_angles(pos1,0)
        pos1[2]=0.002
        grasping_pose=self.get_message_pose(pos1,euler,0.35)
        self.publish_pose(grasping_pose)

        grasping_pose=self.get_message_pose(pos1,euler,0.27)
        self.publish_pose(grasping_pose)
#
        self.publish_grip_cmd(gripper_pre_3)
        grasping_pose=self.get_message_pose(pos1,euler,0.35)
        self.publish_pose(grasping_pose)
        self.publish_pose(waiting_pose)
	time.sleep(2)
	return self.bag.state,self.reward_state_2(pos_obj,pos1),False
    def step_4(self,lift_point):
        self.publish_pose(waiting_pose)
        ##pos,rot=self.get_pose("/"+lift_point)
        ###pos_c,rot_c=self.get_pose("/bag")
        ##wrist=self.get_wrist_angle(self.draging[0],self.draging[1],pos[0],pos[1])
        ##euler=self.get_angles(pos,wrist)
        ##self.draging[0]=self.draging[0]-0.02
        ##euler_c=self.get_angles(self.draging,wrist)
        #pos[2]=0.002
        ##grasping_pose=self.get_message_pose(self.draging,euler_c,0.35)
        ##self.publish_pose(grasping_pose)
        ##self.publish_grip_cmd(gripper_open)
        ##grasping_pose=self.get_message_pose(self.draging,euler_c,0.27)
        ##self.publish_pose(grasping_pose)
        ##self.publish_grip_cmd(gripper_close)
        pos,rot=self.get_pose("/"+lift_point)
	if pos is None:
		return 
	pos[2]=0.002
        euler=self.get_angles(pos,0)
	grasping_pose=self.get_message_pose(pos,euler,0.35)
        self.publish_pose(grasping_pose)
        self.publish_grip_cmd(gripper_open)
        grasping_pose=self.get_message_pose(pos,euler,0.28)
        self.publish_pose(grasping_pose)
        self.publish_grip_cmd(gripper_close)
#
#
        grasping_pose=self.get_message_pose(pos,euler,0.65)
        pos[0]=-0.35
        pos[1]=0.0
        euler=self.get_angles_lift(pos,0)
        grasping_pose=self.get_message_pose(pos,euler,0.8)
        # self.publish_pose(grasping_pose)	
        self.publish_pose(grasping_pose)
        self.publish_pose(waiting_pose)
        self.publish_grip_cmd(gripper_open)
	time.sleep(4)
	s_,r,done=self.reward_state_4(self.bag.state)
	return s_,r,done

    def reward_state_0(self,area):
	return area/34000.0
    def reward_state_1(self,area):
	return area/3900.0
    def reward_state_2(self,obj,place_point):
	reward=1.0-math.sqrt(((obj[0]-place_point[0])**2)+((obj[1]-place_point[1])**2))
	return reward
    def reward_state_4(self,state):
	if state=='3':
		return state,-0.1,False
	if state=='4':
		return state,1.0,True
	if state=='5':
		return state,-0.1,False
	else:
		return state,-0.1,False

class Bag():
    def __init__(self):
        rospy.Subscriber('/bag_manipulation/bag_manipulation_area', Float32, callback=self.current_area_bag_callback)
	rospy.Subscriber('/bag_manipulation/bag_manipulation_opening_area', Float32, callback=self.current_area_opening_callback)
	rospy.Subscriber('/bag_manipulation/bag_manipulation_state', String, callback=self.current_state_callback)
	self.state=0
	self.bag_area=0.0
	self.opening_area=0.0
    def current_area_bag_callback(self,data):
        self.bag_area=data.data
    def current_area_opening_callback(self,data):
        self.opening_area=data.data
    def current_state_callback(self,data):
        self.state=data.data

class CQL():
 	def __init__(self,max_training_steps=100,epsilon_decay=0.01,load_previous=False, name='model'):
		self.cql={'0':[[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
				0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
				0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],0],

		          '1':[[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
				0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
				0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],0],
		          '2':[[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],0],
		          '3':[[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
				0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
				0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],0],
			 }
		self.actions={'0':[['00',0.0],['01',0.0],['02',0.0],['10',0.0],['11',0.0],['12',0.0],['20',0.0],['21',0.0],['22',0.0],
                                   ['00',1.0],['01',1.0],['02',1.0],['10',1.0],['11',1.0],['12',1.0],['20',1.0],['21',1.0],['22',1.0],
                                   ['00',2.0],['01',2.0],['02',2.0],['10',2.0],['11',2.0],['12',2.0],['20',2.0],['21',2.0],['22',2.0],
                                   ['00',3.0],['01',3.0],['02',3.0],['10',3.0],['11',3.0],['12',3.0],['20',3.0],['21',3.0],['22',3.0],
                                   ['00',4.0],['01',4.0],['02',4.0],['10',4.0],['11',4.0],['12',4.0],['20',4.0],['21',4.0],['22',4.0],
                                   ['00',5.0],['01',5.0],['02',5.0],['10',5.0],['11',5.0],['12',5.0],['20',5.0],['21',5.0],['22',5.0],
                                   ['00',6.0],['01',6.0],['02',6.0],['10',6.0],['11',6.0],['12',6.0],['20',6.0],['21',6.0],['22',6.0],
                                   ['00',7.0],['01',7.0],['02',7.0],['10',7.0],['11',7.0],['12',7.0],['20',7.0],['21',7.0],['22',7.0],
                                   ['00',8.0],['01',8.0],['02',8.0],['10',8.0],['11',8.0],['12',8.0],['20',8.0],['21',8.0],['22',8.0]],
			      '1':[['00','40'],['01','40'],['02','40'],['03','40'],['04','40'],['05','40'],['06','40'],['07','40'],['08','40'],
			           ['00','41'],['01','41'],['02','41'],['03','41'],['04','41'],['05','41'],['06','41'],['07','41'],['08','41'],
			           ['00','42'],['01','42'],['02','42'],['03','42'],['04','42'],['05','42'],['06','42'],['07','42'],['08','42'],
			           ['00','50'],['01','50'],['02','50'],['03','50'],['04','50'],['05','50'],['06','50'],['07','50'],['08','50'],
			           ['00','51'],['01','51'],['02','51'],['03','51'],['04','51'],['05','51'],['06','51'],['07','51'],['08','51'],
			           ['00','52'],['01','52'],['02','52'],['03','52'],['04','52'],['05','52'],['06','52'],['07','52'],['08','52'],
			           ['00','60'],['01','60'],['02','60'],['03','60'],['04','60'],['05','60'],['06','60'],['07','60'],['08','60'],
			           ['00','61'],['01','61'],['02','61'],['03','61'],['04','61'],['05','61'],['06','61'],['07','61'],['08','61'],
			           ['00','62'],['01','62'],['02','62'],['03','62'],['04','62'],['05','62'],['06','62'],['07','62'],['08','62'],],
				'2':['00','01','02','10','11','12','20','21','22'],
				'3':['10','11','12','13','14','15','16','17','08',
				     '20','21','22','23','24','25','26','27','18',
				     '30','31','32','33','34','35','36','37','28',
				     '40','41','42','43','44','45','46','47','38',
				     '50','51','52','53','54','55','56','57','48',
				     '60','61','62','63','64','65','66','67','58',
				     '70','71','72','73','74','75','76','77','68',
				     '80','81','82','83','84','85','86','87','78',
				     '90','91','92','93','94','95','96','97','88',]


				}
		self.rewards={'0':[],'1':[],'2':[],'3':[]}
		self.max_training_steps=max_training_steps
		self.epsilon_decay=epsilon_decay
		self.name=name
		self.epsilon_max=0.95
		self.epsilon_min=0.05
		if load_previous:
			self.load_model()
	def update(self,state,action,reward):
		self.cql[state][0][action]=reward
		self.rewards[state].append(reward)

	def select_action(self,state):
		if random.uniform(0,1)<self.e_greedy(state):
			print "random"
			index=random.randint(0,len(self.actions[state])-1) 
			return index,self.actions[state][index]
		else:
			print "maximum"
			#print np.argmax(np.array(self.cql[state][0]))
			#print np.amax(np.array(self.cql[state][0]))
			return np.argmax(np.array(self.cql[state][0])),self.actions[state][np.argmax(np.array(self.cql[state][0]))]	
	def e_greedy(self,state):
		c_steps=self.cql[state][1]
		delta=float(c_steps)/float(self.max_training_steps)
		decay=(1-delta)*self.epsilon_max
		if decay<0:
			decay=0.0
		epsilon=self.epsilon_min+ decay
		return epsilon
	def save_model(self):
		with open(MODEL_PATH+'/'+self.name+'.yaml', 'w') as results:
			yaml.dump(self.cql,results)
		with open(RESULTS_PATH+'/'+self.name+'results.yaml', 'w') as results:
			yaml.dump(self.rewards,results)
	def load_model(self):
		with open(MODEL_PATH+'/'+self.name+'.yaml', 'r') as f:
			data=yaml.safe_load(f)
			for key in data.keys():
				self.cql[key]=data[key]
		with open(RESULTS_PATH+'/'+self.name+'results.yaml', 'r') as f:
			data=yaml.safe_load(f)
			for key in data.keys():
				self.rewards[key]=data[key]
	def max_action(self,state):
		return np.argmax(np.array(self.cql[state][0])),self.actions[state][np.argmax(np.array(self.cql[state][0]))]	



if __name__ == '__main__':
    #agent=CQL(max_training_steps=10,epsilon_decay=0.01,load_previous=True,name='model_10')
    #agent=CQL(max_training_steps=25,epsilon_decay=0.01,load_previous=False,name='model_25')
    agent=CQL(max_training_steps=50,epsilon_decay=0.01,load_previous=True,name='model_50')
    #for i in range(0,100):
    #    agent.cql['1'][1]=i
    #	print(i,agent.select_action('1'))
	
    #exit()
    kuka = Environment()
    #INIT TRAINING
    #LOAD PRETRANING
    #RUN MODEL
    training=True
    while not rospy.is_shutdown():
	    state=kuka.bag.state
            answer=raw_input('Current state: '+str(state)+'. Execute step?(y/n)')
            if answer=="n" or answer=="N":
                continue
            elif answer=="y" or answer=="Y":	
                state=kuka.bag.state
		try:
                        index=None
		        action=None
			if state=='0':
			    #s_,r,done=kuka.step_1("11",2.0)
			    if agent.cql[state][1]<agent.max_training_steps:
                                    print "Training state: "+state+" ("+str(agent.cql[state][1])+" of "+ str(agent.max_training_steps)+")"
				    index,action=agent.select_action(state)
				    s_,r,done=kuka.step_1(action[0],action[1])
				    answer=raw_input('Valid step?(y/n)')
				    if answer=="n" or answer=="N":
					continue
				    elif answer=="y" or answer=="Y":
					    agent.update(state,index,r)
					    agent.cql[state][1]+=1
					    agent.save_model()
					    print "Training state: "+state+" ("+str(agent.cql[state][1])+" of "+ str(agent.max_training_steps)+")"
				    else: continue
			    else:
				index,action=agent.max_action(state)
				s_,r,done=kuka.step_1(action[0],action[1])
				print "Training state: "+state+" finished."
			if state=='1':
		    	    #s_,r,done=kuka.step_2("05","51")
			    if agent.cql[state][1]<agent.max_training_steps and training:
                                    print "Training state: "+state+" ("+str(agent.cql[state][1])+" of "+ str(agent.max_training_steps)+")"

				    index,action=agent.select_action(state)
				    s_,r,done=kuka.step_2(action[0],action[1])
				    answer=raw_input('Valid step?(y/n)')
				    if answer=="n" or answer=="N":
					continue
				    elif answer=="y" or answer=="Y":
					    print ("trainign finished")
					    agent.update(state,index,r)
					    agent.cql[state][1]+=1
					    agent.save_model()
					    print "Training state: "+state+" ("+str(agent.cql[state][1])+" of "+ str(agent.max_training_steps)+")"
		                    else: continue
			    else:
				print(agent.cql[state][1])
				index,action=agent.max_action(state)
				s_,r,done=kuka.step_2(action[0],action[1])
				print "Training state: "+state+" finished."

			if state=='2':
		    	    #s_,r,done=kuka.step_3("11")
			    if agent.cql[state][1]<agent.max_training_steps and training:
                                    print "Training state: "+state+" ("+str(agent.cql[state][1])+" of "+ str(agent.max_training_steps)+")"
				    index,action=agent.select_action(state)
				    s_,r,done=kuka.step_3(action)
				    answer=raw_input('Valid step?(y/n)')
				    if answer=="n" or answer=="N":
					continue
				    elif answer=="y" or answer=="Y":
					    agent.update(state,index,r)
					    agent.cql[state][1]+=1
					    agent.save_model()
					    print "Training state: "+state+" ("+str(agent.cql[state][1])+" of "+ str(agent.max_training_steps)+")"
		                    else: continue
			    else:
				index,action=agent.max_action(state)
				print action
				s_,r,done=kuka.step_3(action)
				print "Training state: "+state+" finished."
			if state=='3':
		    	    #s_,r,done=kuka.step_4("33")
			    if agent.cql[state][1]<agent.max_training_steps and training:
                                    print "Training state: "+state+" ("+str(agent.cql[state][1])+" of "+ str(agent.max_training_steps)+")"
				    index,action=agent.select_action(state)
				    s_,r,done=kuka.step_4(action)
				    print(str(r),str(s_))
				    answer=raw_input('Valid step?(y/n)')
				    if answer=="n" or answer=="N":
					continue
				    elif answer=="y" or answer=="Y":
					    agent.update(state,index,r)
					    agent.cql[state][1]+=1
					    agent.save_model()
					    print "Training state: "+state+" ("+str(agent.cql[state][1])+" of "+ str(agent.max_training_steps)+")"
		                    else: continue
			    else:
				index,action=agent.max_action(state)
				s_,r,done=kuka.step_4(action)
				print "Training state: "+state+" finished."
			    #print r,done
		except Exception as e:
			exc_type, exc_obj, exc_tb=sys.exc_info()
			#fname= os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
			print exc_type, exc_obj, exc_tb.tb_lineno
			#rospy.logerr(e)
			print "Non-valid pose"
	    else:
		print "Non-valid answer"
	    
