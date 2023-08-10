#!/usr/bin/env python
import rospy
import tf
import math
import time
import ast
import threading
import geometry_msgs.msg
import tf_conversions

import Tkinter as tk
import ttk

import sys


class GUI():
    def __init__(self):
        self.root = tk.Tk()
        self.root.title('Realsense tf calibrator')
        self.root.geometry('350x110')
        self.root.resizable(False, False)
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=5)
        self.root.columnconfigure(2, weight=1)
        self.precision=100.0
        self.slide_lenght=200.0

        self.current_x=tk.DoubleVar()
        self.current_y=tk.DoubleVar()
        self.current_z=tk.DoubleVar()
        self.current_pitch=tk.DoubleVar()
        self.current_yaw=tk.DoubleVar()
        self.current_roll=tk.DoubleVar()

        self.slider_label_x = ttk.Label(self.root,text='x:')
        self.slider_label_y = ttk.Label(self.root,text='y:')
        self.slider_label_z = ttk.Label(self.root,text='z:')

        self.slider_label_pitch = ttk.Label(self.root,text='pitch:')
        self.slider_label_yaw = ttk.Label(self.root,text='yaw:')
        self.slider_label_roll = ttk.Label(self.root,text='roll:')

        self.slider_label_x.grid(column=0,row=0,sticky='w')
        self.slider_label_y.grid(column=0,row=1,sticky='w')
        self.slider_label_z.grid(column=0,row=2,sticky='w')

        self.slider_label_pitch.grid(column=0,row=3,sticky='w')
        self.slider_label_yaw.grid(column=0,row=4,sticky='w')
        self.slider_label_roll.grid(column=0,row=5,sticky='w')

        self.slider_x = ttk.Scale(self.root,from_=-2.0*self.precision,to=2.0*self.precision,length=self.slide_lenght,orient='horizontal', command=self.set_x, variable=self.current_x)
        self.slider_y = ttk.Scale(self.root,from_=-2.0*self.precision,to=2.0*self.precision,length=self.slide_lenght,orient='horizontal', command=self.set_y, variable=self.current_y)
        self.slider_z = ttk.Scale(self.root,from_=-2.0*self.precision,to=2.0*self.precision,length=self.slide_lenght,orient='horizontal', command=self.set_z, variable=self.current_z)

        self.slider_pitch = ttk.Scale(self.root,from_=0.0,to=6.0*self.precision,length=self.slide_lenght,orient='horizontal', command=self.set_pitch, variable=self.current_pitch)
        self.slider_yaw = ttk.Scale(self.root,from_=0.0,to=6.0*self.precision,length=self.slide_lenght,orient='horizontal', command=self.set_yaw, variable=self.current_yaw)
        self.slider_roll = ttk.Scale(self.root,from_=0.0,to=6.0*self.precision,length=self.slide_lenght,orient='horizontal', command=self.set_roll, variable=self.current_roll)

        self.slider_x.grid(column=1,row=0,sticky='w')
        self.slider_y.grid(column=1,row=1,sticky='w')
        self.slider_z.grid(column=1,row=2,sticky='w')

        self.slider_pitch.grid(column=1,row=3,sticky='w')
        self.slider_yaw.grid(column=1,row=4,sticky='w')
        self.slider_roll.grid(column=1,row=5,sticky='w')

        self.current_x_label= ttk.Label(self.root, text='')
        self.current_y_label= ttk.Label(self.root, text='')
        self.current_z_label= ttk.Label(self.root, text='')

        self.current_pitch_label= ttk.Label(self.root, text='')
        self.current_yaw_label= ttk.Label(self.root, text='')
        self.current_roll_label= ttk.Label(self.root, text='')

        self.current_x_label.grid(column=2,row=0,sticky='w')
        self.current_y_label.grid(column=2,row=1,sticky='w')
        self.current_z_label.grid(column=2,row=2,sticky='w')
        self.current_pitch_label.grid(column=2,row=3,sticky='w')
        self.current_roll_label.grid(column=2,row=5,sticky='w')
        self.current_yaw_label.grid(column=2,row=4,sticky='w')

        self.current_x_label.configure(text='{: .2f}'.format(self.current_x.get()/self.precision))
        self.current_y_label.configure(text='{: .2f}'.format(self.current_y.get()/self.precision))
        self.current_z_label.configure(text='{: .2f}'.format(self.current_z.get()/self.precision))
        self.current_pitch_label.configure(text='{: .2f}'.format(self.current_pitch.get()/self.precision))
        self.current_yaw_label.configure(text='{: .2f}'.format(self.current_yaw.get()/self.precision))
        self.current_roll_label.configure(text='{: .2f}'.format(self.current_roll.get()/self.precision))

        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(1000.0)
        self.gui_thread = threading.Thread(target=self.node_publisher)
        self.gui_thread.start()
        #self.node_publisher()
        self.x=0
    def node_publisher(self):
        while not rospy.is_shutdown():
            x=self.current_x.get()/self.precision
            y=self.current_y.get()/self.precision
            z=self.current_z.get()/self.precision
            pitch=self.current_pitch.get()/self.precision
            roll=self.current_roll.get()/self.precision
            yaw=self.current_yaw.get()/self.precision
            try:
                self.br.sendTransform((x, y, z),
                                tf_conversions.transformations.quaternion_from_euler(pitch,roll,yaw),
                                rospy.Time.now(),
                                "camera",
                                "world")
                #print()
            except:
		pass
                #print("error")
            self.rate.sleep()
        self.gui_thread.join()
        sys.exit()
    def set_x(self,event):
        self.current_x_label.configure(text='{: .2f}'.format(self.current_x.get()/self.precision))
    def set_y(self,event):
        self.current_y_label.configure(text='{: .2f}'.format(self.current_y.get()/self.precision))
    def set_z(self,event):
        self.current_z_label.configure(text='{: .2f}'.format(self.current_z.get()/self.precision))
    def set_pitch(self,event):
        self.current_pitch_label.configure(text='{: .2f}'.format(self.current_pitch.get()/self.precision))
    def set_yaw(self,event):
        self.current_yaw_label.configure(text='{: .2f}'.format(self.current_yaw.get()/self.precision))
    def set_roll(self,event):
        self.current_roll_label.configure(text='{: .2f}'.format(self.current_roll.get()/self.precision))
def main():
    rospy.init_node('irohms_tf_vicon_calibrator')
    Vicon_Calibrator=GUI()
    Vicon_Calibrator.root.mainloop()
if __name__ == '__main__':
    
    main()


