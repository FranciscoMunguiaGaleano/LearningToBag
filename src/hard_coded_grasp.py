#!/usr/bin/env python

import rospy
import numpy as np
from copy import deepcopy as dcp
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PoseArray
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotInput as inputMsg
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput as outputMsg
import time
#from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
#from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg


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
gripper_pre_1.rPRA = 50
gripper_pre_1.rSPA = 200
gripper_pre_1.rFRA = 150

gripper_pre_2 = outputMsg()
gripper_pre_2.rACT = 1
gripper_pre_2.rGTO = 1
gripper_pre_2.rMOD = 1
gripper_pre_2.rPRA = 20
gripper_pre_2.rSPA = 200
gripper_pre_2.rFRA = 150

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


class Controller:
    def __init__(self):
        rospy.init_node('controller_node', anonymous=True)
        rospy.Subscriber('/iiwa/state/CartesianPose', PoseStamped, callback=self.current_pose_callback)
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
        while not trans and not rospy.is_shutdown():		
            try:
                #t=listener.getLatestCommonTime('/iiwa_link_0', frame)
                (trans,rot) = listener.lookupTransform('/iiwa_link_0', frame, rospy.Time())
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rate.sleep()
                continue
        print(trans,rot)
        #rospy.loginfo('Bag postion: ' + str(trans))
        #rospy.loginfo('Bag rotation: ' +str( rot))
        return trans,rot
    def get_angles(self,pos,wrist):
        euler=[0,0,0]
        euler[0]=3.14159

        euler[2]=math.atan(pos[1]/pos[0])-1.5707+wrist
        euler[2]=-1.5707+wrist
        print(euler[2]*360/(2*3.1416))
        #euler=[0,0,0]
        #rospy.loginfo(euler,pos)
        return euler
    def get_angles_lift(self,pos,wrist):
        euler=[0,0,0]
        euler[0]=3.14159-1.0707
        #euler[1]=0.5060
        euler[2]=math.atan(pos[1]/pos[0])-1.5707+wrist
        euler[2]=-1.5707+wrist
        print(euler[2]*360/(2*3.1416))
         #euler=[0,0,0]
        #rospy.loginfo(euler,pos)
        return euler
    def get_message_pose(self,pos,euler,height):
        grasping_pose = PoseStamped()
        grasping_pose.pose.position.x = pos[0]
        grasping_pose.pose.position.y = pos[1]
        grasping_pose.pose.position.z = pos[2]+height
        #if pos[2]+height<0.265:
        #    grasping_pose.pose.position.z = 0.265
        #    rospy.logwarn("Bad camera calibration")
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
        rospy.loginfo("Initializing robot...")
        self.publish_grip_cmd(gripper_reset)
        self.publish_grip_cmd(gripper_activation)
    def step_1(self,attemps):
        gram_m=0.26
        for i in range(0,attemps):
	
            pos,rot=self.get_pose("/bag")
            euler=self.get_angles(pos,0)
            if (1.18-pos[2])<0.01:
                gram_m=0.26
            else:
                print("0.24")
                gram_m=0.24
            #####
            self.publish_pose(waiting_pose)
            grasping_pose=self.get_message_pose(pos,euler,0.30)
            self.publish_pose(grasping_pose)
            ##Open gripper
            self.publish_grip_cmd(gripper_open)
            grasping_pose=self.get_message_pose(pos,euler,gram_m)
            self.publish_pose(grasping_pose)
            ##Close gripper
            self.publish_grip_cmd(gripper_close)
            ###MEasering
            pos[0]=-0.35
            pos[1]=0.0
            euler=self.get_angles_lift(pos,0)
            grasping_pose=self.get_message_pose(pos,euler,0.8)
            self.publish_pose(grasping_pose)
            self.publish_grip_cmd(gripper_open)
            #Open gripper
            self.publish_pose(waiting_pose)
            time.sleep(3)
            ##TODO subtitue with the area o the bag trigger
            while True and not rospy.is_shutdown():
            #print()
                answer=raw_input('Is the bag expanded?(y/n)')
                if answer=="y" or answer=="n" or answer=="Y" or answer=="N":
                    break
                if answer=="y" or answer=="Y":	
                    break
    def step_2(self):
        self.publish_pose(waiting_pose)
        pos,rot=self.get_pose("/opening_bag")
        pos_c,rot_c=self.get_pose("/bag")
        wrist=self.get_wrist_angle(pos_c[0],pos_c[1],pos[0],pos[1])
        euler=self.get_angles(pos,wrist)
        euler_c=self.get_angles(pos,wrist)
        #####preparing grasping
        #pos[0]=pos[0]+0.03
        pos[2]=0.002
        grasping_pose=self.get_message_pose(pos,euler,0.35)
        self.publish_pose(grasping_pose)
#if True:
#	self.publish_pose(waiting_pose)
#	return 0
        self.publish_grip_cmd(gripper_close)
        grasping_pose=self.get_message_pose(pos,euler,0.26)
        self.publish_pose(grasping_pose)
        self.publish_grip_cmd(gripper_pre_2)
#grasping_pose=self.get_message_pose(pos,euler,0.35)
#self.publish_pose(grasping_pose)
#self.publish_pose(waiting_pose)
        time.sleep(1)
####
	#pos,rot=self.get_pose("/opening_bag")
        if (1.18-pos[2])<0.01:
            gram_m=0.275
        else:
            gram_m=0.285
#pos[0]=pos[0]+0.03
        grasping_pose=self.get_message_pose(pos,euler,0.266)
        self.publish_pose(grasping_pose)
#self.publish_grip_cmd(gripper_pre_2)
        self.publish_grip_cmd(gripper_pre_1)
        grasping_pose=self.get_message_pose(pos,euler,0.28)
        self.publish_pose(grasping_pose)
        self.publish_grip_cmd(gripper_pre_2)
        grasping_pose=self.get_message_pose(pos,euler,0.266)
        self.publish_pose(grasping_pose)
        self.publish_grip_cmd(gripper_pre_1)
#self.publish_grip_cmd(gripper_pre_2)


####
        grasping_pose=self.get_message_pose(pos,euler,gram_m)
        self.publish_pose(grasping_pose)
#self.publish_grip_cmd(gripper_half)
#time.sleep(1)
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
        grasping_pose=self.get_message_pose(pos_c,euler_c,0.35)
        self.publish_pose(grasping_pose)
        self.publish_grip_cmd(gripper_open)
        self.publish_pose(waiting_pose)
    def step_3(self):
        self.publish_pose(waiting_pose) 
        pos=[-0.47,-0.5,0.0]
        euler=self.get_angles(pos,0.0)
        grasping_pose=self.get_message_pose(pos,euler,0.35)
        self.publish_pose(grasping_pose)
#1/0
        self.publish_grip_cmd(gripper_open)
        grasping_pose=self.get_message_pose(pos,euler,0.26)
        self.publish_pose(grasping_pose)
        self.publish_grip_cmd(gripper_close)
        grasping_pose=self.get_message_pose(pos,euler,0.35)
        self.publish_pose(grasping_pose)
###
        pos,rot=self.get_pose("/rim")
        pos_c,rot_c=self.get_pose("/bag")
        wrist=self.get_wrist_angle(pos_c[0],pos_c[1],pos[0],pos[1])
        euler=self.get_angles(pos,wrist)
        euler_c=self.get_angles(pos,wrist)
        pos[2]=0.002
        grasping_pose=self.get_message_pose(pos,euler,0.35)
        self.publish_pose(grasping_pose)

        grasping_pose=self.get_message_pose(pos,euler,0.27)
        self.publish_pose(grasping_pose)
#
        self.publish_grip_cmd(gripper_pre_1)
        grasping_pose=self.get_message_pose(pos,euler,0.35)
        self.publish_pose(grasping_pose)
        self.publish_pose(waiting_pose)
    def step_4(self):
        self.publish_pose(waiting_pose)
        pos,rot=self.get_pose("/rim")
        #pos_c,rot_c=self.get_pose("/bag")
        wrist=self.get_wrist_angle(self.draging[0],self.draging[1],pos[0],pos[1])
        euler=self.get_angles(pos,wrist)
        self.draging[0]=self.draging[0]-0.02
        euler_c=self.get_angles(self.draging,wrist)
        #pos[2]=0.002
        grasping_pose=self.get_message_pose(self.draging,euler_c,0.35)
        self.publish_pose(grasping_pose)
        self.publish_grip_cmd(gripper_open)
        grasping_pose=self.get_message_pose(self.draging,euler_c,0.27)
        self.publish_pose(grasping_pose)
        self.publish_grip_cmd(gripper_close)
#
#
        grasping_pose=self.get_message_pose(self.draging,euler_c,0.65)
        pos[0]=-0.35
        pos[1]=0.0
        euler=self.get_angles_lift(pos,0)
        grasping_pose=self.get_message_pose(pos,euler,0.8)
        # self.publish_pose(grasping_pose)	
        self.publish_pose(grasping_pose)
        time.sleep(5)
        self.publish_pose(waiting_pose)
        self.publish_grip_cmd(gripper_open)

    def gripper_msg(self, data):
        pass

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


if __name__ == '__main__':
    controller = Controller()
    #controller.step_1(10)
    controller.step_2()
    controller.step_3()
    controller.step_4()
    #rospy.spin()
