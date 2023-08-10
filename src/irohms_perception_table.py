#!/usr/bin/env python

import numpy as np
import cv2
import pyrealsense2 as rs
import rospy
from cv_bridge import CvBridge
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import roslib
#from irohms_iql.msg import Elements
import tf
import math


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

class perRealsense():
    def __init__(self):
        self.objects={}
        self.objects_temp={}
        self.hands={}
        self.goals={}
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
        self.filters={'green':[[[32,50,187],[179,255,255]],"g"],'bag':[[[0,0,160],[179,255,255]],"bag"]}
        rospy.init_node('irohms_perception_table', anonymous=True)
        #self.pub = rospy.Publisher('/bag_manipulation/irohms_perception_table_elements', Elements, queue_size=10)
        #self.pub_goal = rospy.Publisher('/irohms_idql/irohms_perception_table_goals', Elements, queue_size=10)
        #self.pub_hands = rospy.Publisher('/irohms_idql/irohms_perception_table_hands', Elements, queue_size=10)
        self.pub_rgb = rospy.Publisher('/bag_manipulation/bag_manipulation_image_rgb', Image, queue_size=10)
        self.pub_depth = rospy.Publisher('/bag_manipulation/bag_manipulation_image_depth', Image, queue_size=10)
        #self.rate = rospy.Rate(100)
	self.align_to=rs.stream.color
	self.align=rs.align(self.align_to)
	self.rem_points=[]
	

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
	

    def tf_publisher(self,depth_frame):
	try:
		q = quaternion_from_euler (3.14159, 0.0,-1.570795)
		self.trans.sendTransform((-0.85,0.05,1.18),
			    (q[0], q[1], q[2], q[3]),
			    rospy.Time.now(),
			    "camera_cal",
			    "camera"	)
		x,y,z=self.twod_to_threed(self.coor_rim_x,self.coor_rim_y,self.dist_rim)
		self.trans.sendTransform((x,y,z),
			    (0.0, 0.0, 0.0, 1.0),
			    rospy.Time.now(),
			    "rim",
			    "camera_cal")
		
		x,y,z=self.twod_to_threed(self.coor_bag_x,self.coor_bag_y,self.dist_bag)
		self.trans.sendTransform((x,y,z),
			    (0.0, 0.0, 0.0, 1.0),
			    rospy.Time.now(),
			    "bag",
			    "camera_cal")
		x,y=self.get_grasp_point()
		x,y,z=self.twod_to_threed(x,y,self.dist_bag)
		self.trans.sendTransform((x,y,z),
			    (0.0, 0.0, 0.0, 1.0),
			    rospy.Time.now(),
			    "opening_bag",
			    "camera_cal")
				
	except Exception as e:
		rospy.logerr(e)
	
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
            while True and not rospy.is_shutdown():
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
                                    color_image_temp=self.tagger(result_color, color_image_filter,self.filters[filter][1])
                                    result_temp=None
                                else:
                                    result_color = cv2.bitwise_and(result, result, mask = mask)
                                    color_image_temp=self.tagger(result_color, color_image_filter,self.filters[filter][1])
                            else:
                                result_temp = cv2.bitwise_and(result, result, mask = mask)  
                else:
                    time+=1
                color_image=self.filter_pass(color_image,result,depth_frame,x,y)
                #hand1,hand2,color_image=self.hand_detector(color_image)
		hand1=hand2=None
                fr+=1
                if fr%20==0:
                    objects_b={}
                    if hand1 is not None or hand2 is not None:
                        objects_b=self.objects_behind(hand1,hand2)
                    self.objects={}
                    self.objects_temp={}
                    self.hands={}
                    self.goals={}
                    self.pts=[]
		    self.rem_points=[]
                    for obj in objects_b.keys():
                        self.objects[obj]=objects_b[obj]
                        self.objects_temp[obj]=objects_b[obj]
                #self.publish_elemnts_info()
                #self.publish_hands_info()
                #self.publish_goal_info()
                cv2.namedWindow('Bag environment', cv2.WINDOW_AUTOSIZE)
		self.tf_publisher(depth_frame)                
		if  depth_colormap is not None:
                    #pass
                    self.pub_depth.publish(self.br.cv2_to_imgmsg(depth_colormap))
                    #cv2.imshow('IDQL environment',depth_colormap)
                if color_image is not None:
                    self.pub_rgb.publish(self.br.cv2_to_imgmsg(full_image))
                    #cv2.imshow('IDQL environment',color_image)
                    #cv2.imshow('IDQL environment',result)
                    depth_colormap_dim = depth_colormap.shape
                    color_colormap_dim = color_image.shape
                    # If depth and color resolutions are different, resize color image to match depth image for display
                    if depth_colormap_dim != color_colormap_dim:
                        resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                        images = np.hstack((resized_color_image, result_color))
                    else:
                        images = np.hstack((color_image_filter,result_color))
                    cv2.imshow('Bag environment',images)
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
                

    def tagger(self,img, image,name):
	kernel=np.ones((5,5),np.float32)/25
	cv2.filter2D(img,-1,kernel)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
        cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        types=""
        objective={}
        
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
                    if name!='goal' and name!='hand'  and l.astype(float)> 0 and w.astype(float)>0:
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
                        #image_cont=cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
                        #image=cv2.circle(image, (cX, cY), 2, (255, 255, 255), -1)
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
        return image
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
			self.dist_bag=round((dist+dist1+dist2+dist3+dist4)/5.0,2)
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
            pass
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
if __name__ == '__main__':
    camera=perRealsense()
    camera.observe_environment()
