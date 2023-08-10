#!/usr/bin/env python


#from cvzone.HandTrackingModule import HandDetector
import numpy as np
import cv2
import pyrealsense2 as rs
import rospy
from cv_bridge import CvBridge
from std_msgs.msg import Int32MultiArray,Float32MultiArray,String,MultiArrayDimension,MultiArrayLayout,Float32
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import roslib
#from irohms_iql.msg import Elements
import tf
import math
from threading import Thread
import sys

rotation_rad=-0.2
x=1.0
y=0.5
q = quaternion_from_euler (0.0, 0.0,rotation_rad)

SH=320
SW=480
RED=[0,0,255]
GREEN=[0,255,0]
BLUE=[255,0,0]
BLUE_GREEN=[255, 255, 0]
YELLOW=[255,0,255]
CAMERA_ID='141722074327'##TODO put in a config file

####TODO list
### 
### 
### 
### 
### learning algorithm                ## independent script
### generate rewards automatically    ##indepented script
### 
### run experiments
###

#MAX_AREA=18000
MAX_AREA=25000
#MAX_AREA_OPENING=50
MAX_AREA_OPENING=700
def state(bag_area,opening_area,piece):
	if bag_area<MAX_AREA and opening_area<1000 and opening_area>0 and bag_area>0 and not piece:
	#if bag_area<27000 and opening_area<100 and opening_area>0 and bag_area>0 and not piece:
		return 0
	elif bag_area>MAX_AREA and opening_area<MAX_AREA_OPENING and opening_area>=0 and bag_area>0 and not piece:
		return 1
	elif bag_area>MAX_AREA and opening_area>MAX_AREA_OPENING and opening_area>0 and bag_area>0 and not piece:
		return 2
	elif bag_area==0 and opening_area==0 and not piece:
		return 4
	elif bag_area==0 and opening_area==0 and piece:
		return 5
	elif bag_area>MAX_AREA and opening_area>=0 and bag_area>0 and piece:
		return 3
	else:
		return 0
def get_center(pts):
	pt=[]	
	for n in pts:
		repeated=False
		for m in pt:
			if m[0]==n[0] and m[1]==n[1]:
				repeated=True 
				break
		if not repeated:
			pt.append([n[0],n[1]]) 
	pts=np.array(pt)
	avg=np.average(pts, axis=0)
	#print avg
	return [int(avg[0]),int(avg[1])]
def find_closest_node(center,pts):
	distance=[]
	for i in range(0,len(pts)):
		distance.append(np.sqrt(((center[0]-pts[i][0])**2)+((center[1]-pts[i][1])**2)))
	index=np.argmin(np.array(distance))
	return index, pts[index]
def get_opening_area(points):
	if len(points)<3:
		return 0, []
	triangles=[]
	nodes=len(points)
	center=get_center(points)
	while True:
		pts=points[:]
		i,n=find_closest_node(center,pts)
		j,m=find_closest_node(n,pts)
		triangles.append([center,n,m])
		pts.pop(j)
		k,o=find_closest_node(n,pts)
		triangles.append([center,n,o])
		points.pop(i)
		if len(points)<3:
			break
	opening=np.zeros((340,340,3),dtype="uint8")
	for triangle in triangles:
		t_points=[(triangle[0][0],triangle[0][1]),(triangle[1][0],triangle[1][1]),(triangle[2][0],triangle[2][1])]
		cv2.fillPoly(opening,np.array([t_points]),color=(255,255,255))
	
	#get contours
	kernel=np.ones((5,5),np.float32)/25
	cv2.filter2D(opening,-1,kernel)
        gray = cv2.cvtColor(opening, cv2.COLOR_BGR2GRAY)
        thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
        cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
	#for c in cnts:
	#	cv2.drawContours(opening, [c], -1, (0, 255, 255), 2)
	#cv2.imshow('Opening',opening)
	#get area
	opening_area=cv2.contourArea(cnts[0])
	#return area and opening contours
	return opening_area, cnts
def find_case(pts,l1,l2,l3,l4):
	cases=['p1p2','p2p3','p3p4','p1p3']
	c1=get_center(l1)
	c2=get_center(l2)
	c3=get_center(l3)
	c4=get_center(l4)
	d1=np.sqrt(((c1[0]-pts[0])**2)+((c1[1]-pts[1])**2))
	d2=np.sqrt(((c2[0]-pts[0])**2)+((c2[1]-pts[1])**2))
	d3=np.sqrt(((c3[0]-pts[0])**2)+((c3[1]-pts[1])**2))
	d4=np.sqrt(((c4[0]-pts[0])**2)+((c4[1]-pts[1])**2))
	index=np.argmin(np.array([d1,d2,d3,d4]))
	#print cases[index]
	return cases[index]

def det(a,b):
	return a[0]*b[1]-a[1]*b[0]
def intersection(l1,l2):
	x=0
	y=0
	inter=False
	xdiff= (l1[0]-l1[2],l2[0]-l2[2])
	ydiff= (l1[1]-l1[3],l2[1]-l2[3])
	div=det(xdiff,ydiff)
	if div==0:
		print("No intersection")
		return [x,y],False
	line1=((l1[0],l1[1]),(l1[2],l1[3]))
	line2=((l2[0],l2[1]),(l2[2],l2[3]))
	d=(det(*line1),det(*line2))
	x= det(d,xdiff)/div
	y= det(d,ydiff)/div
	#print x,y
	return [x,y],True

def list_intersections(h,v):
	intersections={}
	x=0
	y=0
	for h_line in h:
		y=0
		for v_line in v:
			p,inter=intersection(h_line,v_line)
			#p=[]
			#inter=False
			if inter:
				intersections[str(x)+str(y)]=p
			y+=1
		x+=1	
	#print intersections
	return intersections
def list_intersections_oriented(h,v,orientation,stride):
	intersections={}
	x=0+stride[0]
	y=stride[1]
	if orientation=='p1p3':
		for h_line in h:
			y=stride[1]
			for v_line in v:
				p,inter=intersection(h_line,v_line)
				if inter:
					intersections[str(x)+str(y)]=p
				y+=1
			x+=1	
		return intersections
	if orientation=='p3p4':
		try:
			for i in range(0,len(h)):
				y=stride[1]
				for j in range(0,len(v)):
					p,inter=intersection(h[-1-i],v[-1-j])
					if inter:
						intersections[str(x)+str(y)]=p
					y+=1
				x+=1	
			return intersections
		except Exception as e:
			print e
			
	if orientation=='p2p3':
		try:
			for i in range(0,len(v)):
				y=stride[1]
				for j in range(0,len(h)):
					p,inter=intersection(h[j],v[-1-i])
					if inter:
						intersections[str(x)+str(y)]=p
					y+=1
				x+=1	
			return intersections
		except Exception as e:
			print e
	if orientation=='p1p2':
		try:    
			for i in range(0,len(v)):
				y=stride[1]
				for j in range(0,len(h)):
					p,inter=intersection(h[-1-j],v[i])
					if inter:
						intersections[str(x)+str(y)]=p
					y+=1
				x+=1	
			return intersections
		except Exception as e:
			print e

def find_coorners(contours):
	M = cv2.moments(contours)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
	distance_q1=[]
	distance_q2=[]
	distance_q3=[]
	distance_q4=[]
	q_1=[]
	q_2=[]
	q_3=[]
	q_4=[]
	for c in contours:
		if (c[0][0]>cX and c[0][1]>cY):
			distance_q1.append(np.sqrt(((cX-c[0][0])**2+(cY-c[0][1])**2)))
			q_1.append([c[0][0],c[0][1]])
		if (c[0][0]>cX and c[0][1]<cY):
			distance_q2.append(np.sqrt(((cX-c[0][0])**2+(cY-c[0][1])**2)))
			q_2.append([c[0][0],c[0][1]])
		if c[0][0]<cX and c[0][1]<cY:
			distance_q3.append(np.sqrt(((cX-c[0][0])**2+(cY-c[0][1])**2)))
			q_3.append([c[0][0],c[0][1]])
		if c[0][0]<cX and c[0][1]>cY:
			distance_q4.append(np.sqrt(((cX-c[0][0])**2+(cY-c[0][1])**2)))
			q_4.append([c[0][0],c[0][1]])
	coorner_1=q_1[np.argmax(np.array(distance_q1))]
        coorner_2=q_2[np.argmax(np.array(distance_q2))]
        coorner_3=q_3[np.argmax(np.array(distance_q3))]
        coorner_4=q_4[np.argmax(np.array(distance_q4))]
	#print coorner_1
	return [coorner_1,coorner_2],[coorner_3,coorner_4],[coorner_3,coorner_2],[coorner_1,coorner_4]



class perRealsense():
    def __init__(self):
        self.objects={}
        self.objects_temp={}
        self.hands={}
        self.goals={}
        self.contours=[]
	self.interaction_points=[]
        self.running=True
        self.pts=[]
        self.br=CvBridge()
	self.trans = tf.TransformBroadcaster()
	self.dist_rim=0.0
	self.coor_rim_x=0
	self.coor_rim_y=0
        self.dist_bag=0.5
	self.coor_bag_x=0
	self.coor_bag_y=0
	self.intr=0.1
	self.state_bag=0
	self.area_bag=0.0
	self.red_frame=None
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        #self.config.enable_device(CAMERA_ID)
        #self.config.enable_device()
        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        self.device = pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))
	depth_sensor=pipeline_profile.get_device().query_sensors()[0]
	depth_sensor.set_option(rs.option.enable_auto_exposure,False)
	rgb_sensor=pipeline_profile.get_device().query_sensors()[1]
	rgb_sensor.set_option(rs.option.enable_auto_exposure,True)
        self.camera_chaeckup(False)
        #self.detector = HandDetector(detectionCon=0.8, maxHands=2)
        #self.filters={'pink1':[[[166,64,170],[174,255,255]],None],'pink2':[[[0,72,0],[179,255,255]],"goal"],
        #              'blue':[[[65,37,0],[113,255,255]],"b"],'green':[[[33,28,69],[74,230,255]],"g"],
        #              'yellow':[[[20,37,169],[51,222,255]],'y'],'red':[[[0,119,0],[7,255,255]],'r']}
        #self.filters={'green':[[[32,50,187],[179,255,255]],"g"],'bag':[[[0,0,160],[179,255,255]],"bag"]}
	self.filters={'green':[[[32,50,187],[55,255,255]],"g"],'bag':[[[0,0,103],[179,107,255]],"bag"]}
        rospy.init_node('irohms_perception_table', anonymous=True)
        #self.pub = rospy.Publisher('/bag_manipulation/irohms_perception_table_elements', Elements, queue_size=10)
        #self.pub_goal = rospy.Publisher('/irohms_idql/irohms_perception_table_goals', Elements, queue_size=10)
        #self.pub_hands = rospy.Publisher('/irohms_idql/irohms_perception_table_hands', Elements, queue_size=10)
        self.pub_rgb = rospy.Publisher('/bag_manipulation/bag_manipulation_image_rgb', Image, queue_size=10)
        self.pub_depth = rospy.Publisher('/bag_manipulation/bag_manipulation_image_depth', Image, queue_size=10)
	self.pub_graping_points = rospy.Publisher('/bag_manipulation/bag_manipulation_grasping_points', Float32MultiArray, queue_size=10)
	self.pub_state=rospy.Publisher('/bag_manipulation/bag_manipulation_state', String, queue_size=10)
	self.pub_tags=rospy.Publisher('/bag_manipulation/bag_manipulation_grasp_tags', String, queue_size=10)
	self.pub_bag_area=rospy.Publisher('/bag_manipulation/bag_manipulation_area', Float32, queue_size=10)
	self.pub_opening_area=rospy.Publisher('/bag_manipulation/bag_manipulation_opening_area', Float32, queue_size=10)
        #self.rate = rospy.Rate(100)
	self.align_to=rs.stream.color
	self.align=rs.align(self.align_to)
	self.rem_points=[]
	self.red_piece_position=None
	self.opening_area=0.0
	self.opening_cnts=[]
	self.orientacion=None
	tf_thread=Thread(target=self.tf_publisher)
	tf_thread.start()
	self.bag_coorners=[]
	

    def twod_to_threed(self,x,y,z):
	#frames = self.pipeline.wait_for_frames()
        #aligned_frames = self.align.process(frames)
        #depth_frame = aligned_frames.get_depth_frame()
	#self.intr = depth_frame.profile.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics
	return rs.rs2_deproject_pixel_to_point(self.intr,[float(x),float(y)],z)
    def get_dist(self,x,y,depth_frame):
	dist=[]
	for i in range(x-5,x+5):
		for j in range(y-5,y+5):
			dist.append(depth_frame.get_distance(i,j))
	return sum(dist)/len(dist)

    def get_grasp_point(self):
	x=self.coor_rim_x-(self.coor_rim_x-self.coor_bag_x)/10
	y=self.coor_rim_y-(self.coor_rim_y-self.coor_bag_y)/10
	#x=self.coor_rim_x-20
	#y=self.coor_rim_y-20
	return int(x),int(y)
	

    def tf_publisher(self):
	while not rospy.is_shutdown():
		rate=rospy.Rate(1000)
		try:
			q = quaternion_from_euler (3.14159, 0.0,-1.570795)
			self.trans.sendTransform((-0.85,0.05,1.18),
				    (q[0], q[1], q[2], q[3]),
				    rospy.Time.now(),
				    "camera_cal",
				    "camera"	)
			x,y,z=self.twod_to_threed(self.coor_rim_x,self.coor_rim_y,self.dist_rim)
			self.trans.sendTransform((round(x,2),round(y,2),round(z,2)),
				    (0.0, 0.0, 0.0, 1.0),
				    rospy.Time.now(),
				    "rim",
				    "camera_cal")
		
			x,y,z=self.twod_to_threed(self.coor_bag_x,self.coor_bag_y,self.dist_bag)
			self.trans.sendTransform((round(x,2),round(y,2),round(z,2)),
				    (0.0, 0.0, 0.0, 1.0),
				    rospy.Time.now(),
				    "bag",
				    "camera_cal")
			x,y=self.get_grasp_point()
			x,y,z=self.twod_to_threed(x,y,self.dist_bag)
			self.trans.sendTransform((round(x,2),round(y,2),round(z,2)),
				    (0.0, 0.0, 0.0, 1.0),
				    rospy.Time.now(),
				    "opening_bag",
				    "camera_cal")
		#		
		except Exception as e:
			pass
			##rospy.logerr(e)
		try:
			tags="|"
			data=[]
			for gp in self.interaction_points:
				x,y,z=self.twod_to_threed(gp[1],gp[2],gp[3])
				tags+=str(gp[0])+"|"
				self.trans.sendTransform((round(x,2),round(y,2),round(z,2)),
				    (0.0, 0.0, 0.0, 1.0),
				    rospy.Time.now(),
				    str(gp[0]),
				    "camera_cal")
			self.pub_state.publish(str(self.state_bag))
			#self.pub_tags.publish(tags)
				
		except Exception as e:
			pass
			#print(e)
			#rospy.logerr(e)
		try:
			for gp in self.bag_coorners:
				x,y,z=self.twod_to_threed(gp[1],gp[2],gp[3])
				self.trans.sendTransform((round(x,2),round(y,2),round(z,2)),
				    (0.0, 0.0, 0.0, 1.0),
				    rospy.Time.now(),
				    str(gp[0]),
				    "camera_cal")
			self.pub_state.publish(str(self.state_bag))
			#self.pub_tags.publish(tags)
				
		except Exception as e:
			pass
			#print(e)
			#rospy.logerr(e)
#
		try:
			#print(self.red_piece_position)
			x,y,z=self.twod_to_threed(self.red_piece_position[0],self.red_piece_position[1],self.red_piece_position[2])
			self.trans.sendTransform((round(x,2),round(y,2),round(1.16,2)),
			    (0.0, 0.0, 0.0, 1.0),
			    rospy.Time.now(),
			    "object",
			    "camera_cal")
			self.pub_state.publish(str(self.state_bag))
				
		except Exception as e:
			pass
			#print(e)
			#rospy.logerr(e)
		try:
			self.pub_opening_area.publish(self.opening_area)
			self.pub_bag_area.publish(self.area_bag)
		except:
			pass
		rate.sleep()
	
    def camera_chaeckup(self,found_rgb):
        for s in self.device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("This requires Depth camera with Color sensor")
            exit(0)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        if self.device_product_line == 'L500':
            self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        cfg=self.pipeline.start(self.config)
 	#cfg = pipeline.start() # Start pipeline and get the configuration it found
	profile = cfg.get_stream(rs.stream.depth) # Fetch stream profile for depth stream
	self.intr = profile.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics
	
	#
    def hand_detector(self,img):
        color_image=img.copy()
        hands, img = self.detector.findHands(img)  # With Draw
        hand1=None
        hand2=None
        if hands:
            hand1 = hands[0]
            if len(hands) == 2:
                hand2 = hands[1]
        if hand1 is not None:
                #lmList2 = hand2["lmList"]  # List of 21 Landmarks points
                bbox2 = hand1["bbox"]  # Bounding Box info x,y,w,h
                centerPoint2 = hand1["center"]  # center of the hand cx,cy
                handType2 = hand1["type"] 
                cv2.rectangle(color_image, (bbox2[0], bbox2[1]), (bbox2[0]+bbox2[2], bbox2[1]+bbox2[3]), (0,255,0), 1)
                cv2.circle(color_image, (centerPoint2[0], centerPoint2[1]), 2, (255, 255, 255), -1)
                cv2.putText(color_image, str(handType2), (centerPoint2[0], centerPoint2[1]),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                self.hands[handType2]={}
                self.hands[handType2]['position']=[centerPoint2[0],centerPoint2[1]]
                self.hands[handType2]['lenght']=bbox2[2]
                self.hands[handType2]['width']=bbox2[3]


        if hand2 is not None:
                bbox2 = hand2["bbox"] 
                centerPoint2 = hand2["center"] 
                handType2 = hand2["type"] 
                cv2.rectangle(color_image, (bbox2[0], bbox2[1]), (bbox2[0]+bbox2[2], bbox2[1]+bbox2[3]), (0,255,0), 1)
                cv2.circle(color_image, (centerPoint2[0], centerPoint2[1]), 2, (255, 255, 255), -1)
                cv2.putText(color_image, str(handType2), (centerPoint2[0], centerPoint2[1]),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                self.hands[handType2]={}
                self.hands[handType2]['position']=[centerPoint2[0],centerPoint2[1]]
                self.hands[handType2]['lenght']=bbox2[2]
                self.hands[handType2]['width']=bbox2[3]
        return hand1,hand2,color_image

    def observe_environment(self):
        x, y, w, h = 0, 180, 340, 340
        align_to = rs.stream.color
        align = rs.align(align_to)
        fr=0
        try:
            time=0
            #while not rospy.is_shutdown():
            while not rospy.is_shutdown():
                frames = self.pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
		
		#profile = cfg.get_stream(rs.stream.depth) # Fetch stream profile for depth stream
		self.intr = depth_frame.profile.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics
                if not depth_frame or not color_frame:
                    continue
                #depth_image = np.asanyarray(depth_frame.get_data())[x:x+w, y:y+h]
		depth_image = np.asanyarray(depth_frame.get_data())
		full_image=np.asanyarray(color_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
		self.red_frame=color_image.copy()
		color_image_filter = np.asanyarray(color_frame.get_data())[x:x+w, y:y+h]
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.06), cv2.COLORMAP_JET)
                #enhancer = ImageEnhance.Contrast(Image.fromarray(np.uint8(cm.gist_earth(color_image)*255)))
                #factor = 1.5 #gives original image
                #color_image = enhancer.enhance(factor)
                hsv = cv2.cvtColor(color_image_filter, cv2.COLOR_BGR2HSV)
                lower = np.array([0,0,122])
                lower = np.array([0,0,155])
                upper = np.array([179,255,255])
                #upper = np.array([150,255,255])
                mask = cv2.inRange(hsv, lower, upper)
                result = cv2.bitwise_and(color_image_filter, color_image_filter, mask = mask)
                result=color_image_filter
                result_color=color_image_filter
                #color_image_temp=color_image
                if time>50:
                        result_temp =None
                        for filter in self.filters.keys():
                            lower=np.array(self.filters[filter][0][0])
                            upper=np.array(self.filters[filter][0][1])
                            hsv = cv2.cvtColor(result, cv2.COLOR_BGR2HSV)
                            mask = cv2.inRange(hsv, lower, upper)
                            if self.filters[filter][1] is not None:
                                if result_temp is not None:
                                    result_color = cv2.bitwise_and(result_temp, result_temp, mask = mask)
                                    color_image_temp,grid=self.tagger(result_color, color_image_filter,self.filters[filter][1],depth_frame,x,y)
                                    result_temp=None
                                else:
                                    result_color = cv2.bitwise_and(result, result, mask = mask)
                                    color_image_temp,grid=self.tagger(result_color, color_image_filter,self.filters[filter][1],depth_frame,x,y)
                            else:
                                result_temp = cv2.bitwise_and(result, result, mask = mask)  
                else:
                    time+=1
                color_image=self.filter_pass(color_image,result,depth_frame,x,y)
                #hand1,hand2,color_image=self.hand_detector(color_image)
		hand1=hand2=None
                fr+=1
                if fr%5==0:
                    objects_b={}
                    if hand1 is not None or hand2 is not None:
                        objects_b=self.objects_behind(hand1,hand2)
                    self.objects={}
                    self.objects_temp={}
                    self.hands={}
                    self.goals={}
                    self.pts=[]
		    self.rem_points=[]
		    self.interaction_points=[]
                    for obj in objects_b.keys():
                        self.objects[obj]=objects_b[obj]
                        self.objects_temp[obj]=objects_b[obj]
                #self.publish_elemnts_info()
                #self.publish_hands_info()
                #self.publish_goal_info()
                cv2.namedWindow('Bag environment', cv2.WINDOW_AUTOSIZE)
		#self.tf_publisher(depth_frame)                
		if  depth_colormap is not None:
                    #pass
                    self.pub_depth.publish(self.br.cv2_to_imgmsg(depth_colormap))
                    #cv2.imshow('IDQL environment',depth_colormap)
                if color_image is not None:
                    #self.pub_rgb.publish(self.br.cv2_to_imgmsg(full_image))
		    #self.pub_rgb.publish(self.br.cv2_to_imgmsg(full_image))
		    
                    #cv2.imshow('IDQL environment',color_image)
                    #cv2.imshow('IDQL environment',result)
                    depth_colormap_dim = depth_colormap.shape
                    color_colormap_dim = color_image.shape
                    # If depth and color resolutions are different, resize color image to match depth image for display
                    if depth_colormap_dim != color_colormap_dim:
                        resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                        images = np.hstack((resized_color_image, result_color))
                    else:
			try:
                        	images = np.hstack((color_image_filter,result_color))
                                cv2.imshow('Bag environment',images)
				#self.pub_rgb.publish(self.br.cv2_to_imgmsg(grid))
				
			except:
				pass
				cv2.imshow('Bag environment',color_image_filter)
				#self.pub_rgb.publish(self.br.cv2_to_imgmsg())
                    
                cv2.waitKey(1)
                if cv2.getWindowProperty('Bag environment', cv2.WND_PROP_VISIBLE) <1:
                    break
            cv2.destroyAllWindows()
        finally:
            self.pipeline.stop()
    def publish_elemnts_info(self):
        elements=Elements()
        position=Int32MultiArray()
        color=Int32MultiArray()
        try:
            for key in self.objects.keys():
                elements.key.append(key)
                elements.x.append(self.objects[key]['position'][0])
                elements.y.append(self.objects[key]['position'][1])
                elements.z.append(self.objects[key]['position'][2])
                elements.lenght.append(int(self.objects[key]['lenght']))
                elements.width.append(int(self.objects[key]['width']))
                elements.height.append(int(self.objects[key]['height']))
                elements.shape.append(self.objects[key]['shape'])
                elements.r.append(self.objects[key]['color'][0])
                elements.g.append(self.objects[key]['color'][1])
                elements.b.append(self.objects[key]['color'][2])
                elements.goal.append("")
            if len(elements.key)>0:
                self.pub.publish(elements)
        except Exception as e:
            #rospy.logwarn(e)
            pass
    def publish_hands_info(self):
        elements=Elements()
        try:
            if len(self.hands.keys())>0:
                for key in self.hands.keys():
                    elements.key.append(key)
                    elements.x.append(self.hands[key]['position'][0])
                    elements.y.append(self.hands[key]['position'][1])
                    #elements.z=self.hands[key]['position'][2]
                    elements.lenght.append(self.hands[key]['lenght'])
                    elements.width.append(self.hands[key]['width'])
                    elements.height.append(10)
                    elements.shape.append("square")
                    elements.r.append(255)
                    elements.g.append(255)
                    elements.b.append(255)

            else:
                for key in ["left","right"]:
                    elements.key.append(key)
                
            #rospy.loginfo(elements)
            #self.pub_hands.publish(elements)
        except Exception as e:
            #rospy.logwarn(e)
            pass
    def publish_goal_info(self):
        elements=Elements()
        position=Int32MultiArray()
        try:
            for key in self.goals.keys():
                elements.key.append(key)
                elements.x.append(self.goals[key]['position'][0])
                elements.y.append(self.goals[key]['position'][1])
                #elements.z=self.objects[key]['position'][2]
            if len(elements.key)>0:
                #self.pub_goal.publish(elements)
                pass
        except Exception as e:
            print(e)
            pass
    def objects_behind(self,hand1,hand2):
        xmin=[]
        ymax=[]
        ymin=[]
        xmax=[]
        objects_b={}
        if hand1 is not None:
                bbox2 = hand1["bbox"]  # Bounding Box info x,y,w,h
                xmin.append(bbox2[0])
                ymin.append(bbox2[1])
                ymax.append(bbox2[0]+bbox2[2])
                xmax.append(bbox2[1]+bbox2[3])
        if hand2 is not None:
                bbox2 = hand2["bbox"] 
                xmin.append(bbox2[0])
                ymin.append(bbox2[1])
                ymax.append(bbox2[0]+bbox2[2])
                xmax.append(bbox2[1]+bbox2[3])
        for object in self.objects.keys():
            x=self.objects[object]['position'][0]
            y=self.objects[object]['position'][1]
            for i in range(0,len(xmin)):
                if x<xmax[i] and x>xmin[i] and y<ymax[i] and y>ymin[i]:
                    objects_b[object]=self.objects[object]
        return objects_b
               
    def filter_pass(self,color_image,result,depth_frame,u,v):
        #print "error"
	#print u,v
        pop_objects=[]
        for object in self.objects.keys():
            cX=self.objects[object]['position'][0]
            cY=self.objects[object]['position'][1]
            le=self.objects[object]['lenght']
            wi=self.objects[object]['width']
            blank_space=np.asanyarray(result)[cY-int(le/7):cY+int(le/7),cX-int(wi/7):cX+int(wi/7)]
            if np.average(blank_space)==0.0:
                pop_objects.append(object)
        for pop_obj in pop_objects:
            self.objects.pop(pop_obj)
        
        pop_objects=[]

        for object in self.objects.keys():
            temp_objects=self.objects.copy()
            temp_objects.pop(object)
            cX=int(self.objects[object]['position'][0]/10)
            cY=int(self.objects[object]['position'][1]/10)
            for obj in temp_objects.keys():
                X=int(temp_objects[obj]['position'][0]/10)
                Y=int(temp_objects[obj]['position'][1]/10)
                if obj[0]==object[0]:
                    if cX+1==X or cX-1==X:
                        if cY+1==Y or cY-1==Y:
                            pop_objects.append(obj)
        for pop_obj in pop_objects:
            try:
                self.objects.pop(pop_obj)
            except:
                continue
        for object in self.objects.keys():
            c=self.objects[object]['contours']
            cX=self.objects[object]['position'][0]
            cY=self.objects[object]['position'][1]
            types=self.objects[object]['shape'][0]
            #print(object[:3])
            if object[:3]=="bag":
                #cv2.drawContours(color_image, c, -1, (0, 255, 0), 2)
                #cv2.circle(color_image, (int(cX), int(cY)), 2, RED, -1)
                #cv2.putText(color_image, object[:3], (cX - 20, cY - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.3, RED, 1)
		cv2.circle(color_image, (int(cX+v), int(cY+u)), 2, RED, -1)
                cv2.putText(color_image, object[:3], (cX+v - 20, cY+u - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.3, RED, 1)
		try:   
			ran=5     
			dist=depth_frame.get_distance(cX+v, cY+u) 
			dist1=depth_frame.get_distance(cX+v+ran, cY+u)
			dist2=depth_frame.get_distance(cX+v-ran, cY+u)
			dist3=depth_frame.get_distance(cX+v, cY+u+ran)
			dist4=depth_frame.get_distance(cX+v, cY+u-ran)
			self.dist_bag=(dist+dist1+dist2+dist3+dist4)/5.0
			#dist=[]   
			#for a in range(cX+u-5,cX+u+5):
		#		for b in range(cY+u-5,cY+u+5):
		#			#dist=depth_frame.get_distance(cX+u, cY+v)
	#				dist.append(depth_frame.get_distance(a,b))
#			dist=sum(dist)/len(dist)
		except:
			continue
		#if dist>0.5 and dist<1.6:
		
		self.coor_bag_x=cX+v
		self.coor_bag_y=cY+u
                cv2.putText(color_image, str(round(dist,2))+" m", (cX+v - 20, cY+u + 20),cv2.FONT_HERSHEY_SIMPLEX, 0.3, RED, 1)
            #cv2.drawContours(color_image, c, -1, (0, 255, 0), 2)
            #cv2.circle(color_image, (int(cX), int(cY)), 2, RED, -1)
            #cv2.putText(color_image, object[:3], (cX - 20, cY - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.3, RED, 1)
            #dist=depth_frame.get_distance(cX, cY)
            #cv2.putText(color_image, str(round(dist,2))+" m", (cX - 20, cY + 20),cv2.FONT_HERSHEY_SIMPLEX, 0.3, RED, 1)
                #print(dist)

        try:
	    pts=np.array(sorted(self.pts))
	    pt=[]	
	    for n in pts:
		repeated=False
		for m in pt:
			if m[0]==n[0] and m[1]==n[1]:
				repeated=True 
				break
		if not repeated:
			pt.append([n[0]+v,n[1]+u]) 
	    pts=np.array(pt)
            dist=[]
            for x,y in pt:
		point_rim_dis=depth_frame.get_distance(x, y)
                dist.append(point_rim_dis)
		self.rem_points.append([x,y,point_rim_dis])
            pts = pts.reshape((-1,1,2))
            avg=np.average(pts, axis=0)
	   #rim position
            dis_avg=sum(dist)/len(dist)
	    if dis_avg>1.0 and dis_avg<1.6:
	    	#self.dist_rim=dis_avg
		if depth_frame.get_distance(int(avg[0,0]), int(avg[0,1]))>1.0:
			self.dist_rim=depth_frame.get_distance(int(avg[0,0]), int(avg[0,1]))
			self.coor_rim_x=int(avg[0,0])
			self.coor_rim_y=int(avg[0,1])
	    cv2.circle(color_image, (int(avg[0,0]),int(avg[0,1])), 2, BLUE, -1)
	    cv2.putText(color_image, "RIM", (int(avg[0,0]) - 10, int(avg[0,1]) - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.3, BLUE, 1)
	    cv2.putText(color_image, str(round(dis_avg,2))+" m",(int(avg[0,0]) - 10, int(avg[0,1]) + 20),cv2.FONT_HERSHEY_SIMPLEX, 0.3, BLUE, 1)
	    #cv2.fillPoly(color_image,[pts],BLUE)
        except Exception as e:
            print e
            #pass
        return color_image
    def get_color(self,name):
        color=[255,255,255]
        if name == 'b':
            color=BLUE
        elif name =='y':
            color=YELLOW
        elif name=='g':
            color=GREEN
        elif name=='r':
            color=RED
        return color
    def tagger(self,img, image , name,depth_frame,u,v):
        grid=image.copy()
	kernel=np.ones((5,5),np.float32)/25
	cv2.filter2D(img,-1,kernel)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
        cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        types=""
        objective={}
	l_opening=[]
	#if name =="bag" and cv2.contourArea(c)>500:
	#			self.contours=[c]
	#			self.pts.append([cX,cY])
	if name =="bag":
	#	for c in cnts:
			#cv2.drawContours(img, [c], -1, (0, 255, 0), 2)	
		#cv2.imshow('pinshi filrtr',img)
		self.contours=cnts
        try:
            for c in cnts:
                w=max([xmax[0][0] for xmax in c])-min([xmax[0][0] for xmax in c])
                l=max([xmax[0][1] for xmax in c])-min([xmax[0][1] for xmax in c])
                approx = cv2.approxPolyDP(c,0.055*cv2.arcLength(c,True),True)
                if len(approx)==5 and name!='b':
                    types="pentagon"
                elif len(approx)==3:
                    types="triangle"
                elif len(approx)==4:
                    types="square"
                elif len(approx) == 6 and name!='b':
                    types="hexa"
                elif len(approx) == 7 and name!='b':
                    types="hexa"
                elif len(approx) > 7:
                    types="circle"
                elif len(approx) > 4 and name=='b':
                    types="circle"
                else:
                    types=""
                M = cv2.moments(c)
                try:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    color=[255,255,255]

                    color=self.get_color(name)
		    
                    if name!='goal' and name!='hand'  and l.astype(float)> 2 and w.astype(float)>2:
                        self.objects_temp[name+"_"+types[0]+"_"+str(int(cX/10))+"_"+str(int(cY/10))]={}
                        self.objects_temp[name+"_"+types[0]+"_"+str(int(cX/10))+"_"+str(int(cY/10))]['position']=[cX,cY,0]
                        self.objects_temp[name+"_"+types[0]+"_"+str(int(cX/10))+"_"+str(int(cY/10))]['orientation']=[0,0,0]
                        if name=='y':
                            l=l*3
                            w=w*3
                        self.objects_temp[name+"_"+types[0]+"_"+str(int(cX/10))+"_"+str(int(cY/10))]['lenght']=float(l.astype(float))
                        self.objects_temp[name+"_"+types[0]+"_"+str(int(cX/10))+"_"+str(int(cY/10))]['width']=float(w.astype(float))
                        self.objects_temp[name+"_"+types[0]+"_"+str(int(cX/10))+"_"+str(int(cY/10))]['height']=50
                        self.objects_temp[name+"_"+types[0]+"_"+str(int(cX/10))+"_"+str(int(cY/10))]['shape']=types    
                        self.objects_temp[name+"_"+types[0]+"_"+str(int(cX/10))+"_"+str(int(cY/10))]['color']=color
                        self.objects_temp[name+"_"+types[0]+"_"+str(int(cX/10))+"_"+str(int(cY/10))]['path']=[]
                        self.objects_temp[name+"_"+types[0]+"_"+str(int(cX/10))+"_"+str(int(cY/10))]['grid']=[]
                        self.objects_temp[name+"_"+types[0]+"_"+str(int(cX/10))+"_"+str(int(cY/10))]['path_smooth']=[]
                        self.objects_temp[name+"_"+types[0]+"_"+str(int(cX/10))+"_"+str(int(cY/10))]['goal']=[]
                        self.objects_temp[name+"_"+types[0]+"_"+str(int(cX/10))+"_"+str(int(cY/10))]['contours']=[c]
                        if name!='bag':
                            self.pts.append([cX,cY])
			    l_opening.append([cX,cY])
                        
                    else:
                        #print(name)
                        
                        if types=='square' and name=='goal':
                            #cv2.putText(image, name+"_1", (cX - 20, cY - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
                            self.goals['left']={}
                            self.goals['left']['position']=[cX, cY + 40]
                            
                        elif types=='triangle' and name=='goal':
                            #cv2.putText(image, name+"_2", (cX - 20, cY - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
                            self.goals['right']={}
                            self.goals['right']['position']=[cX, cY + 40]
                    
                            
                except Exception as e:
                    pass
                    #rospy.logwarn(e)
            #pts = np.array([[10,5],[20,30],[70,20],[50,10]], np.int32)
        except Exception as e:
            print(e)
            pass
            #rospy.logwarn(e)
        pop_out=[]
        pop_in=[]
        for object in self.objects_temp.keys():
            if object not in self.objects.keys():
                close_obj=False
                i_tmp=self.objects_temp[object]['position'][0]
                j_tmp=self.objects_temp[object]['position'][1]                
                for obj in self.objects.keys():    
                    i_ref=self.objects[obj]['position'][0]
                    j_ref=self.objects[obj]['position'][1]
                    if (i_ref<i_tmp+3 and i_ref>i_tmp-3) and (j_ref<j_tmp+3 and j_ref>j_tmp-3):
                        close_obj=True
                        pop_out.append(obj)
                        pop_in.append(object)
                        break
                if not close_obj:
                    self.objects[object]=self.objects_temp[object]
        for p_out in pop_out:
            try:
                self.objects.pop(p_out)
            except:
                continue
        for o_in in pop_in:
            self.objects[o_in]=self.objects_temp[o_in]
	try:
		
		for c in self.contours:
			try:
				l1,l2,l3,l4=find_coorners(c)
				
				self.area_bag=cv2.contourArea(c)
			except:
				continue
			
			p11=int(l1[0][0])
			p21=int(l1[0][1])
			p31=int(l1[1][0])
			p41=int(l1[1][1])
			#print p1,p2,p3,p4
			#print (l1[1][0],l1[1,1])
			grid=cv2.line(image.copy(),(p11,p21),(p31,p41),(0,255,0),2)
			p12=int(l2[0][0])
			p22=int(l2[0][1])
			p32=int(l2[1][0])
			p42=int(l2[1][1])
			#print p1,p2,p3,p4
			#print (l1[1][0],l1[1,1])
			cv2.line(grid,(p12,p22),(p32,p42),(0,255,0),2)
			p13=int(l3[0][0])
			p23=int(l3[0][1])
			p33=int(l3[1][0])
			p43=int(l3[1][1])
			#print p1,p2,p3,p4
			#print (l1[1][0],l1[1,1])
			cv2.line(grid,(p13,p23),(p33,p43),(0,255,0),2)
			p14=int(l4[0][0])
			p24=int(l4[0][1])
			p34=int(l4[1][0])
			p44=int(l4[1][1])
			#print p1,p2,p3,p4
			#print (l1[1][0],l1[1,1])
			cv2.line(grid,(p14,p24),(p34,p44),(0,255,0),2)
			#grid=cv2.drawContours(grid, [c], -1, (0, 255, 0), 2)
			
		
		####Check opening
		#l=self.pts[:]
		
		opening_area,opening_cnts=get_opening_area(l_opening)
		if len(l_opening)>0:
			self.opening_area=opening_area
			self.opening_cnts=opening_cnts
		else:
			opening_area=self.opening_area
			opening_cnts=self.opening_cnts
			
		#if opening_area==0 and :
		#	return 
		#opening_area=0
		#opening_cnts=[]
		#for c in opening_cnts:
		#	cv2.drawContours(grid, [c], -1, (255, 0, 255), 2)
		#####Look for piece
		piece=False
		if self.red_frame is not None:
			try:
				#piece,self.red_piece_position=get_red_piece(self.red_frame,[[p11,p21],[p31,p41],[p12,p22],[p32,p42]],u,v)
				#x, y, w, h = 0, 180, 340, 340
				piece,self.red_piece_position=get_red_piece(self.red_frame,[[340,340],[340,0],[0,0],[0,340]],u,v)
				#piece=False
				#self.red_piece_position=None
				if self.red_piece_position is not None:
					self.red_piece_position.append(depth_frame.get_distance(int(self.red_piece_position[0]),int(self.red_piece_position[1])))
				#print red_piece_position, piece
			except Exception as e:
				piece=False
				self.red_piece_position=None
				#print e
		#####Get current state
		self.state_bag=state(self.area_bag,opening_area,piece)
		try:
			lado_1x=p31-p11##parallel P1-P2
			lado_1y=p41-p21##parallel P1-P2
			lado_2x=p34-p14#P1-P4
			lado_2y=p44-p24#P1-P4
			lado_3x=p12-p32##parallel P3-P2
			lado_3y=p22-p42##parallel P3-P2
			lado_4x=p13-p31#P3-P4
			lado_4y=p23-p41#P3-P4
                        #print l1
			#self.bag_coorners=[['c1',l1[0][0]-10+v,l1[0][1]-10+u,depth_frame.get_distance(int(l1[0][0]+v-10),int(l1[0][1]+u-10))],
                        #                   ['c2',l1[1][0]-10+v,l1[1][1]+10+u,depth_frame.get_distance(int(l1[1][0]+v-10),int(l1[1][1]+u+10))],
                        #                   ['c3',l2[0][0]+v+10,l2[0][1]+u+10,depth_frame.get_distance(int(l2[0][0]+v+10),int(l2[0][1]+u+10))],
                        #                   ['c4',l2[1][0]+v+10,l2[1][1]+u-10,depth_frame.get_distance(int(l2[1][0]+v+10),int(l2[1][1]+u)-10)]]
		except Exception as e:
			#print e
			#exc_type, exc_obj, exc_tb=sys.exc_info()
		#fname= os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
			#print exc_type, exc_obj, exc_tb.tb_lineno
			piece=False
			if self.red_frame is not None:
				try:
					piece,self.red_piece_position=get_red_piece(self.red_frame,[[340,340],[340,0],[0,0],[0,340]],u,v)
					if self.red_piece_position is not None:
						self.red_piece_position.append(depth_frame.get_distance(int(self.red_piece_position[0]),int(self.red_piece_position[1])))
				except Exception as e:
					print e
					piece=False
					self.red_piece_position=None
			self.state_bag=state(0,0,piece)
			grid=cv2.putText(image.copy(), "Bag area: 0", (5,275),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
			cv2.putText(grid, "Opening area: 0", (5,295),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
			cv2.putText(grid, "Piece: "+str(piece), (5,315),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
			cv2.putText(grid, "State: "+str(self.state_bag), (5,335),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
			self.pub_rgb.publish(self.br.cv2_to_imgmsg(grid))
			return image,grid
		
		
		###STATE 0
		if self.state_bag==0:
			
			n=3
			horizontal_lines=[]
			vertical_lines=[]
			for i in range(1,n):
				fxi=int(p11+(lado_1x*i)/n)
				fyi=int(p21+(lado_1y*i)/n)
				fxf=int(p32+(lado_3x*i)/n)
				fyf=int(p42+(lado_3y*i)/n)
				cv2.line(grid,(fxi,fyi),(fxf,fyf),(0,255,0),2)
			for i in range(1,n):
				fxi=int(p14+(lado_2x*i)/n)
				fyi=int(p24+(lado_2y*i)/n)
				fxf=int(p31+(lado_4x*i)/n)
				fyf=int(p41+(lado_4y*i)/n)
				cv2.line(grid,(fxi,fyi),(fxf,fyf),(0,255,0),2)	
			self.interaction_points=[]
			for i in range(0,n):
				fxi=int(p11+(lado_1x*(i+0.5))/n)
				fyi=int(p21+(lado_1y*(i+0.5))/n)
				fxf=int(p32+(lado_3x*(i+0.5))/n)
				fyf=int(p42+(lado_3y*(i+0.5))/n)
				cv2.line(grid,(fxi,fyi),(fxf,fyf),(255,255,0),2)
				horizontal_lines.append([fxi,fyi,fxf,fyf])
			for i in range(0,n):
				fxi=int(p14+(lado_2x*(i+0.5))/n)
				fyi=int(p24+(lado_2y*(i+0.5))/n)
				fxf=int(p31+(lado_4x*(i+0.5))/n)
				fyf=int(p41+(lado_4y*(i+0.5))/n)
				cv2.line(grid,(fxi,fyi),(fxf,fyf),(255,255,0),2)
				vertical_lines.append([fxi,fyi,fxf,fyf])
				grasping_points=list_intersections(horizontal_lines,vertical_lines)
				for key in grasping_points.keys():
					cv2.circle(grid, (int(grasping_points[key][0]),int(grasping_points[key][1])), 4, (0, 0, 255), -1)
				for key in grasping_points.keys():
					self.interaction_points.append([key,int(grasping_points[key][0])+v,int(grasping_points[key][1])+u,depth_frame.get_distance(int(grasping_points[key][0])+v,int(grasping_points[key][1])+u)])
		###STATE 1
		if self.state_bag==1:
			
			horizontal_lines=[]
			vertical_lines=[]
			case=find_case([self.coor_rim_x-v,self.coor_rim_y-u],l1,l2,l3,l4)
			self.orientacion=case
			#case='p1p3'
			n=9
			####P1 to P3
			if case=='p1p3':
				for i in range(1,2):
					fxi=int(p11+(lado_1x*i)/n)
					fyi=int(p21+(lado_1y*i)/n)
					fxf=int(p32+(lado_3x*i)/n)
					fyf=int(p42+(lado_3y*i)/n)
					#cv2.line(grid,(fxi,fyi),(fxf,fyf),(0,255,0),1)
				for i in range(1,9):
					fxi=int(p14+(lado_2x*i)/n)
					fyi=int(p24+(lado_2y*i)/n)
					fxf=int(p31+(lado_4x*i)/n)
					fyf=int(p41+(lado_4y*i)/n)
					#cv2.line(grid,(fxi,fyi),(fxf,fyf),(0,255,0),1)	
				self.interaction_points=[]
				for i in range(0,1):
					fxi=int(p11+(lado_1x*(i+1.6))/n)
					fyi=int(p21+(lado_1y*(i+1.6))/n)
					fxf=int(p32+(lado_3x*(i+1.6))/n)
					fyf=int(p42+(lado_3y*(i+1.6))/n)
					#cv2.line(grid,(fxi,fyi),(fxf,fyf),(255,255,0),1)
					horizontal_lines.append([fxi,fyi,fxf,fyf])
				for i in range(0,9):
					fxi=int(p14+(lado_2x*(i+0.5))/n)
					fyi=int(p24+(lado_2y*(i+0.5))/n)
					fxf=int(p31+(lado_4x*(i+0.5))/n)
					fyf=int(p41+(lado_4y*(i+0.5))/n)
					#cv2.line(grid,(fxi,fyi),(fxf,fyf),(255,255,0),1)
					vertical_lines.append([fxi,fyi,fxf,fyf])
				#grasping_points=list_intersections(horizontal_lines,vertical_lines)
				grasping_points=list_intersections_oriented(horizontal_lines,vertical_lines,self.orientacion,[0,0])
				for key in grasping_points.keys():
					cv2.circle(grid, (int(grasping_points[key][0]),int(grasping_points[key][1])), 2, (0, 0, 255), -1)
				for key in grasping_points.keys():
					self.interaction_points.append([key,int(grasping_points[key][0])+v,int(grasping_points[key][1])+u,depth_frame.get_distance(int(grasping_points[key][0])+v,int(grasping_points[key][1])+u)])
			####P1 to P3
			if case=='p3p4':
				for i in range(9,10):
					fxi=int(p11+(lado_1x*i)/n)
					fyi=int(p21+(lado_1y*i)/n)
					fxf=int(p32+(lado_3x*i)/n)
					fyf=int(p42+(lado_3y*i)/n)
					#cv2.line(grid,(fxi,fyi),(fxf,fyf),(0,255,0),1)
				for i in range(1,9):
					fxi=int(p14+(lado_2x*i)/n)
					fyi=int(p24+(lado_2y*i)/n)
					fxf=int(p31+(lado_4x*i)/n)
					fyf=int(p41+(lado_4y*i)/n)
					#cv2.line(grid,(fxi,fyi),(fxf,fyf),(0,255,0),1)	
				self.interaction_points=[]
				for i in range(8,9):
					fxi=int(p11+(lado_1x*(i+0.5))/n)
					fyi=int(p21+(lado_1y*(i+0.5))/n)
					fxf=int(p32+(lado_3x*(i+0.5))/n)
					fyf=int(p42+(lado_3y*(i+0.5))/n)
					#cv2.line(grid,(fxi,fyi),(fxf,fyf),(255,255,0),1)
					horizontal_lines.append([fxi,fyi,fxf,fyf])
				for i in range(0,9):
					fxi=int(p14+(lado_2x*(i+0.5))/n)
					fyi=int(p24+(lado_2y*(i+0.5))/n)
					fxf=int(p31+(lado_4x*(i+0.5))/n)
					fyf=int(p41+(lado_4y*(i+0.5))/n)
					#cv2.line(grid,(fxi,fyi),(fxf,fyf),(255,255,0),1)
					vertical_lines.append([fxi,fyi,fxf,fyf])
				#grasping_points=list_intersections(horizontal_lines,vertical_lines)
				grasping_points=list_intersections_oriented(horizontal_lines,vertical_lines,self.orientacion,[0,0])
				for key in grasping_points.keys():
					cv2.circle(grid, (int(grasping_points[key][0]),int(grasping_points[key][1])), 2, (0,0, 255), -1)
				for key in grasping_points.keys():
					self.interaction_points.append([key,int(grasping_points[key][0])+v,int(grasping_points[key][1])+u,depth_frame.get_distance(int(grasping_points[key][0])+v,int(grasping_points[key][1])+u)])
			####P1 to P2
			if case=='p1p2':
				for i in range(1,9):
					fxi=int(p11+(lado_1x*i)/n)
					fyi=int(p21+(lado_1y*i)/n)
					fxf=int(p32+(lado_3x*i)/n)
					fyf=int(p42+(lado_3y*i)/n)
					#cv2.line(grid,(fxi,fyi),(fxf,fyf),(0,255,0),1)
				for i in range(1,2):
					fxi=int(p14+(lado_2x*i)/n)
					fyi=int(p24+(lado_2y*i)/n)
					fxf=int(p31+(lado_4x*i)/n)
					fyf=int(p41+(lado_4y*i)/n)
					#cv2.line(grid,(fxi,fyi),(fxf,fyf),(0,255,0),1)	
				self.interaction_points=[]
				for i in range(0,9):
					fxi=int(p11+(lado_1x*(i+0.5))/n)
					fyi=int(p21+(lado_1y*(i+0.5))/n)
					fxf=int(p32+(lado_3x*(i+0.5))/n)
					fyf=int(p42+(lado_3y*(i+0.5))/n)
					#cv2.line(grid,(fxi,fyi),(fxf,fyf),(255,255,0),1)
					horizontal_lines.append([fxi,fyi,fxf,fyf])
				for i in range(0,1):
					fxi=int(p14+(lado_2x*(i+1.6))/n)
					fyi=int(p24+(lado_2y*(i+1.6))/n)
					fxf=int(p31+(lado_4x*(i+1.6))/n)
					fyf=int(p41+(lado_4y*(i+1.6))/n)
					#cv2.line(grid,(fxi,fyi),(fxf,fyf),(255,255,0),1)
					vertical_lines.append([fxi,fyi,fxf,fyf])
				#grasping_points=list_intersections(horizontal_lines,vertical_lines)
				grasping_points=list_intersections_oriented(horizontal_lines,vertical_lines,self.orientacion,[0,0])
				for key in grasping_points.keys():
					cv2.circle(grid, (int(grasping_points[key][0]),int(grasping_points[key][1])), 2, (0, 0, 255), -1)
				for key in grasping_points.keys():
					self.interaction_points.append([key,int(grasping_points[key][0])+v,int(grasping_points[key][1])+u,depth_frame.get_distance(int(grasping_points[key][0])+v,int(grasping_points[key][1])+u)])
		####P2P3
			if case=='p2p3':
				for i in range(1,9):
					fxi=int(p11+(lado_1x*i)/n)
					fyi=int(p21+(lado_1y*i)/n)
					fxf=int(p32+(lado_3x*i)/n)
					fyf=int(p42+(lado_3y*i)/n)
					##cv2.line(grid,(fxi,fyi),(fxf,fyf),(0,255,0),1)
				for i in range(9,10):
					fxi=int(p14+(lado_2x*i)/n)
					fyi=int(p24+(lado_2y*i)/n)
					fxf=int(p31+(lado_4x*i)/n)
					fyf=int(p41+(lado_4y*i)/n)
					#cv2.line(grid,(fxi,fyi),(fxf,fyf),(0,255,0),1)	
				self.interaction_points=[]
				for i in range(0,9):
					fxi=int(p11+(lado_1x*(i+0.5))/n)
					fyi=int(p21+(lado_1y*(i+0.5))/n)
					fxf=int(p32+(lado_3x*(i+0.5))/n)
					fyf=int(p42+(lado_3y*(i+0.5))/n)
					#cv2.line(grid,(fxi,fyi),(fxf,fyf),(255,255,0),1)
					horizontal_lines.append([fxi,fyi,fxf,fyf])
				for i in range(8,9):
					fxi=int(p14+(lado_2x*(i-1.1))/n)
					fyi=int(p24+(lado_2y*(i-1.1))/n)
					fxf=int(p31+(lado_4x*(i-1.1))/n)
					fyf=int(p41+(lado_4y*(i-1.1))/n)
					#cv2.line(grid,(fxi,fyi),(fxf,fyf),(255,255,0),1)
					vertical_lines.append([fxi,fyi,fxf,fyf])
				#grasping_points=list_intersections(horizontal_lines,vertical_lines)
				grasping_points=list_intersections_oriented(horizontal_lines,vertical_lines,self.orientacion,[0,0])
				for key in grasping_points.keys():
					cv2.circle(grid, (int(grasping_points[key][0]),int(grasping_points[key][1])), 2, (0, 0, 255), -1)
				for key in grasping_points.keys():
					self.interaction_points.append([key,int(grasping_points[key][0])+v,int(grasping_points[key][1])+u,depth_frame.get_distance(int(grasping_points[key][0])+v,int(grasping_points[key][1])+u)])
			###STATE 2
		if self.state_bag==2:
			for c in opening_cnts:
				l1,l2,l3,l4=find_coorners(c)
				p11=int(l1[0][0])
				p21=int(l1[0][1])
				p31=int(l1[1][0])
				p41=int(l1[1][1])
				#print p1,p2,p3,p4
				#print (l1[1][0],l1[1,1])
				cv2.line(grid,(p11,p21),(p31,p41),(0,255,0),2)
				p12=int(l2[0][0])
				p22=int(l2[0][1])
				p32=int(l2[1][0])
				p42=int(l2[1][1])
				#print p1,p2,p3,p4
				#print (l1[1][0],l1[1,1])
				cv2.line(grid,(p12,p22),(p32,p42),(0,255,0),2)
				p13=int(l3[0][0])
				p23=int(l3[0][1])
				p33=int(l3[1][0])
				p43=int(l3[1][1])
				#print p1,p2,p3,p4
				#print (l1[1][0],l1[1,1])
				cv2.line(grid,(p13,p23),(p33,p43),(0,255,0),2)
				p14=int(l4[0][0])
				p24=int(l4[0][1])
				p34=int(l4[1][0])
				p44=int(l4[1][1])
				#print p1,p2,p3,p4
				#print (l1[1][0],l1[1,1])
				cv2.line(grid,(p14,p24),(p34,p44),(0,255,0),2)
			
			n=3
			horizontal_lines=[]
			vertical_lines=[]
			lado_1x=p31-p11##parallel P1-P2
			lado_1y=p41-p21##parallel P1-P2
			lado_2x=p34-p14#P1-P4
			lado_2y=p44-p24#P1-P4
			lado_3x=p12-p32##parallel P3-P2
			lado_3y=p22-p42##parallel P3-P2
			lado_4x=p13-p31#P3-P4
			lado_4y=p23-p41#P3-P4
			for i in range(1,n):
				fxi=int(p11+(lado_1x*i)/n)
				fyi=int(p21+(lado_1y*i)/n)
				fxf=int(p32+(lado_3x*i)/n)
				fyf=int(p42+(lado_3y*i)/n)
				#cv2.line(grid,(fxi,fyi),(fxf,fyf),(0,255,0),2)
			for i in range(1,n):
				fxi=int(p14+(lado_2x*i)/n)
				fyi=int(p24+(lado_2y*i)/n)
				fxf=int(p31+(lado_4x*i)/n)
				fyf=int(p41+(lado_4y*i)/n)
				#cv2.line(grid,(fxi,fyi),(fxf,fyf),(0,255,0),2)	
			self.interaction_points=[]
			for i in range(0,n):
				fxi=int(p11+(lado_1x*(i+0.5))/n)
				fyi=int(p21+(lado_1y*(i+0.5))/n)
				fxf=int(p32+(lado_3x*(i+0.5))/n)
				fyf=int(p42+(lado_3y*(i+0.5))/n)
				cv2.line(grid,(fxi,fyi),(fxf,fyf),(255,255,0),2)
				horizontal_lines.append([fxi,fyi,fxf,fyf])
			for i in range(0,n):
				fxi=int(p14+(lado_2x*(i+0.5))/n)
				fyi=int(p24+(lado_2y*(i+0.5))/n)
				fxf=int(p31+(lado_4x*(i+0.5))/n)
				fyf=int(p41+(lado_4y*(i+0.5))/n)
				cv2.line(grid,(fxi,fyi),(fxf,fyf),(255,255,0),2)
				vertical_lines.append([fxi,fyi,fxf,fyf])
			#grasping_points=list_intersections(horizontal_lines,vertical_lines)
			if self.orientacion is not None:
				grasping_points=list_intersections_oriented(horizontal_lines,vertical_lines,self.orientacion,[0,0])
			else:
				grasping_points=list_intersections_oriented(horizontal_lines,vertical_lines,'p1p3',[0,0])
			for key in grasping_points.keys():
				cv2.circle(grid, (int(grasping_points[key][0]),int(grasping_points[key][1])), 4, (0, 0, 255), -1)
			for key in grasping_points.keys():
				self.interaction_points.append([key,int(grasping_points[key][0])+v,int(grasping_points[key][1])+u,depth_frame.get_distance(int(grasping_points[key][0])+v,int(grasping_points[key][1])+u)])

		###STATE 3
		if self.state_bag==3:
			n=9
			horizontal_lines=[]
			vertical_lines=[]
			for i in range(1,n):
				fxi=int(p11+(lado_1x*i)/n)
				fyi=int(p21+(lado_1y*i)/n)
				fxf=int(p32+(lado_3x*i)/n)
				fyf=int(p42+(lado_3y*i)/n)
				#cv2.line(grid,(fxi,fyi),(fxf,fyf),(0,255,0),2)
			for i in range(1,n):
				fxi=int(p14+(lado_2x*i)/n)
				fyi=int(p24+(lado_2y*i)/n)
				fxf=int(p31+(lado_4x*i)/n)
				fyf=int(p41+(lado_4y*i)/n)
				#cv2.line(grid,(fxi,fyi),(fxf,fyf),(0,255,0),2)	
			self.interaction_points=[]
			for i in range(0,n):
				fxi=int(p11+(lado_1x*(i+0.5))/n)
				fyi=int(p21+(lado_1y*(i+0.5))/n)
				fxf=int(p32+(lado_3x*(i+0.5))/n)
				fyf=int(p42+(lado_3y*(i+0.5))/n)
				cv2.line(grid,(fxi,fyi),(fxf,fyf),(255,255,0),2)
				horizontal_lines.append([fxi,fyi,fxf,fyf])
			for i in range(0,n):
				fxi=int(p14+(lado_2x*(i+0.5))/n)
				fyi=int(p24+(lado_2y*(i+0.5))/n)
				fxf=int(p31+(lado_4x*(i+0.5))/n)
				fyf=int(p41+(lado_4y*(i+0.5))/n)
				cv2.line(grid,(fxi,fyi),(fxf,fyf),(255,255,0),2)
				vertical_lines.append([fxi,fyi,fxf,fyf])
			#grasping_points=list_intersections(horizontal_lines,vertical_lines)
			if self.orientacion is not None:
				grasping_points=list_intersections_oriented(horizontal_lines,vertical_lines,self.orientacion,[0,0])
			else:
				grasping_points=list_intersections_oriented(horizontal_lines,vertical_lines,'p1p3',[0,0])
			for key in grasping_points.keys():
				cv2.circle(grid, (int(grasping_points[key][0]),int(grasping_points[key][1])), 1, (0, 0, 255), -1)
			for key in grasping_points.keys():
				self.interaction_points.append([key,int(grasping_points[key][0])+v,int(grasping_points[key][1])+u,depth_frame.get_distance(int(grasping_points[key][0])+v,int(grasping_points[key][1])+u)])			
		
				
		if self.orientacion is not None and self.state_bag==1:
			n=3
			horizontal_lines=[]
			vertical_lines=[]
			for i in range(0,n):
				fxi=int(p11+(lado_1x*(i+0.5))/n)
				fyi=int(p21+(lado_1y*(i+0.5))/n)
				fxf=int(p32+(lado_3x*(i+0.5))/n)
				fyf=int(p42+(lado_3y*(i+0.5))/n)
				#cv2.line(grid,(fxi,fyi),(fxf,fyf),(255,255,0),2)
				horizontal_lines.append([fxi,fyi,fxf,fyf])
			for i in range(0,n):
				fxi=int(p14+(lado_2x*(i+0.5))/n)
				fyi=int(p24+(lado_2y*(i+0.5))/n)
				fxf=int(p31+(lado_4x*(i+0.5))/n)
				fyf=int(p41+(lado_4y*(i+0.5))/n)
				#cv2.line(grid,(fxi,fyi),(fxf,fyf),(255,255,0),2)
				vertical_lines.append([fxi,fyi,fxf,fyf])
			grasping_points=list_intersections_oriented(horizontal_lines,vertical_lines,self.orientacion,[4,0])
			for key in grasping_points.keys():
				cv2.circle(grid, (int(grasping_points[key][0]),int(grasping_points[key][1])), 2, (255, 0, 0), -1)
			for key in grasping_points.keys():
				self.interaction_points.append([str(key),int(grasping_points[key][0])+v,int(grasping_points[key][1])+u,depth_frame.get_distance(int(grasping_points[key][0])+v,int(grasping_points[key][1])+u)])

		cv2.putText(grid, "Bag area: "+str(self.area_bag), (5,275),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
		cv2.putText(grid, "Opening area: "+str(opening_area), (5,295),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
		cv2.putText(grid, "Piece: "+str(piece), (5,315),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
		cv2.putText(grid, "State: "+str(self.state_bag), (5,335),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
		self.pub_rgb.publish(self.br.cv2_to_imgmsg(grid))
		#cv2.imshow('Grasping points',grid)
	except Exception as e:
		#print "putitou"
		#exc_type, exc_obj, exc_tb=sys.exc_info()
		##fname= os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
		#print exc_type, exc_obj, exc_tb.tb_lineno
		#rospy.logerr(e)
		return image,grid
        return image,grid
def get_red_piece(frame,boundaries,u,v):
	p1x=boundaries[0][0]+v
	p1y=boundaries[0][1]+u
	p2x=boundaries[1][0]+v
	p2y=boundaries[1][1]+u
	p3x=boundaries[2][0]+v
	p3y=boundaries[2][1]+u
	p4x=boundaries[3][0]+v
	p4y=boundaries[3][1]+u
	
	lower = np.array([0, 111, 143])
        upper = np.array([22, 255, 255])
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        output = cv2.bitwise_and(frame,frame, mask= mask)
	kernel=np.ones((5,5),np.float32)/25
	cv2.filter2D(output,-1,kernel)
        gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
        cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
	for c in cnts:
        	try:
			#cv2.drawContours(frame, [c], -1, (255, 0, 255), 2)
			#cv2.imshow('Red piece', frame)
			M = cv2.moments(c)
                	cX = int(M["m10"] / M["m00"])
                	cY = int(M["m01"] / M["m00"])
			p=[cX,cY]
			if p[0]>p3x and p[0]>p4x and p[0]<p2x and p[0]<p1x and p[1]>p3y and p[1]<p4y and p[1]>p2y and p[1]<p1y:
				return True, p
			return False,p
		except Exception as e:
			print "putitou"
			print e
			return False, None
	

if __name__ == '__main__':
    camera=perRealsense()
    camera.observe_environment()
