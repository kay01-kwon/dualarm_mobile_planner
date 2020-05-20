#!/usr/bin/env python

#--------Include modules---------------
import os
import tf
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from time import time
import numpy as np
from actor_net import ActorNet

# Subscribers' callbacks----------------------------------------
# mapData=OccupancyGrid()
scandata=[]
odom_robot=[]
path=[]


# def mapCallBack(data):
#     global mapData
#     mapData=data
    
def scanCallBack(data):
	global scandata   
	scandata=list(data.ranges)
	i=0
	while i < len(scandata):
		if scandata[i] == 0 :
			scandata[i] = 10.0
		i=i+1

def odomCallBack(data):
	global odom_robot   
	odom_robot=list([data.pose.pose.position.x, data.pose.pose.position.y, tf.transformations.euler_from_quaternion([
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w])[2]])
	
def pathCallback(data):
	global path
	path=[]
	for pose in data.poses:
		path.append(list([pose.pose.position.x, pose.pose.position.y, tf.transformations.euler_from_quaternion([
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w])[2]]))
	
# Node------------------------------------------------------------------

def node():
	global scandata,w
	rospy.init_node('ddpg', anonymous=False)
	agent = ActorNet(60, 3)

	# load weights
	agent.load_actor(os.path.abspath(__file__).replace('ddpg.py','weights/actor/model.ckpt'))

	# map_topic = rospy.get_param('~map_topic','/map')
	scan_topic = rospy.get_param('~scan_topic','/scan')
	odom_topic = rospy.get_param('~odom_topic','/odom')
	path_topic = rospy.get_param('~path_topic','/move_base/TebLocalPlannerROS/local_plan')
	rate = rospy.Rate(rospy.get_param('~rate',10))
#-------------------------------------------------------------------------
	# rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
	rospy.Subscriber(scan_topic, LaserScan, scanCallBack)
	rospy.Subscriber(odom_topic, Odometry, odomCallBack)
	rospy.Subscriber(path_topic, Path, pathCallback)
#-------------------------------------------------------------------------
	pub = rospy.Publisher('ddpg_goal', Marker, queue_size=10) 
	pub2 = rospy.Publisher('cmd_vel', Twist, queue_size=10) 
#-------------------------------------------------------------------------
	listener = tf.TransformListener()
		
	q=Point()
	
	points_ddpg=Marker()
	# points_ddpg.header.frame_id= mapData.header.frame_id
	points_ddpg.header.stamp= rospy.Time.now()
	points_ddpg.ns= "markers1"
	points_ddpg.id = 1
	points_ddpg.type = Marker.POINTS
	points_ddpg.action = Marker.ADD
	points_ddpg.pose.orientation.w = 1.0
	points_ddpg.scale.x=0.1
	points_ddpg.scale.y=0.1 
	points_ddpg.color.r = 0.0/255.0
	points_ddpg.color.g = 255.0/255.0
	points_ddpg.color.b = 255.0/255.0
	points_ddpg.color.a=1
	points_ddpg.lifetime = rospy.Duration()
		
#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
	
	print("Main Loop runnig...")
	trans=[]
	rot=[]
	# prev_action = [0.0, 0.0, 0.0]
	while not rospy.is_shutdown():
		listener.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(10.0))
		cond=0
		while cond==0:	
			try:
				(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time.now())
				cond=1
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				cond==0
			
			position = trans[0:2]
			orientation = [list(tf.transformations.euler_from_quaternion(rot))[2]]
			odom_world = position + orientation
			
			print("odom_robot : ", odom_robot, type(odom_robot))
			print("odom_world : ", odom_world, type(odom_world)) 
		
		
		if len(path)>5: 
			q.x=path[5][0]
			q.y=path[5][1]
			qq=[]
			qq.append(copy(q))
			points_ddpg.points=qq
			pub.publish(points_ddpg)
			
			'''
			path[5][0]- position
			state = np.array(scandata + prev_action + path[5])
			action = agent.evaluate_actor(state)
			print('Current Decision:',action)
			vel.linear.x = aciton[0]
			vel.linear.y = aciton[1]
			vel.linear.z = 0
			vel.angular.x = 0
			vel.angular.y = 0
			vel.angular.z = aciton[2]
			pub2(vel)
			prev_action = action
			'''
		else :
			print("Path length not enough")

#------------------------------------------------------------------------- 
		rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
	try:
		node()
	except rospy.ROSInterruptException:
		pass
 
 
 
 
