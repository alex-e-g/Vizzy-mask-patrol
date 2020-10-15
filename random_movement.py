#!/usr/bin/env python 

#Joao Avelino, 2020.
#ISR-Lisboa / IST

#A template of a GeneralActionlib actionserver for you to use

import random
import ast

#ROS imports
import rospy
import actionlib

#General action files
from vizzy_behavior_trees.msg import GeneralAction, GeneralFeedback, GeneralResult

#Laser scan messages
from sensor_msgs.msg import LaserScan

#Direct wheel command messages
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped

#Odometry messages: wheel information from encoders (unreliable in real scenarios)
from nav_msgs.msg import Odometry

from locale import atof


class RandomMovementServer(object):
    # create messages that are used to publish feedback/result
    _feedback = GeneralFeedback()
    _result = GeneralResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, GeneralAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):

        #5Hz control loop
        dt = 1.0/5.0
        r = rospy.Rate(1.0/dt)
        success = True
        

	#Choosing the desired coordinates
	txt=open("/home/alex/Desktop/coords.txt","r")
	limits=ast.literal_eval(txt.read())

	position = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, timeout=0.5)

	x1=position.pose.pose.position.x
	y1=position.pose.pose.position.y

	if limits[0][0][0] <= x1 <= limits[0][0][1] and limits[0][1][0] <= y1 <= limits[0][1][1]:
	    visited='0'
	if limits[1][0][0] <= x1 <= limits[1][0][1] and limits[1][1][0] <= y1 <= limits[1][1][1]:
	    visited='1'
	if limits[2][0][0] <= x1 <= limits[2][0][1] and limits[3][1][0] <= y1 <= limits[2][1][1]:
	    visited='2'
	if limits[3][0][0] <= x1 <= limits[3][0][1] and limits[3][1][0] <= y1 <= limits[3][1][1]:
	    visited='3'
	
	print(visited)

	a=0
	for i in goal.constants:
	    a+=1
	    if i==visited:
		break
	if a==len(goal.constants):
	    a=0

	room_limits=limits[ast.literal_eval(goal.constants[a])]
	
        x=round(random.uniform(room_limits[0][0],room_limits[0][1]),4)
        y=round(random.uniform(room_limits[1][0],room_limits[1][1]),4)
        result=str(x)+";"+str(y)+";0;0;0;0;1"
	    
        print(result)
	
	if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
              
        if success:
	    self._result.result = result
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
        else:
            self._as.set_aborted()
            self._result.result = "Nao consigo andar. Um obstaculo!"
            rospy.loginfo('%s: Failed' % self._action_name)
        
	r.sleep()
        
if __name__ == '__main__':
    rospy.init_node('random_movement')
    server = RandomMovementServer(rospy.get_name())
    rospy.spin()
