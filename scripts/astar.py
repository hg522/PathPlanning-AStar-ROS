#!/usr/bin/env python
#license removed for brevity

import rosbag
import rospy
import numpy as np
import sys
import math
import time
from operator import attrgetter
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

pathFollower = Odometry()
velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=5)
ubid = '50292195'
np.random.seed(sum([ord(c) for c in ubid]))
startPos = [-8.0,-2.0]
horizcost = 10
diagcost = 14
gridmap = []
checkedlist = []
uncheckedlist = []
currentPos = []
fpath=[]
find = 0
rangedist = 3
startlocation = []

#class to store each point
class node:
	def __init__(self,parent = None, loc = None, g=0,h=0):
		self.g = g
		self.h = h
		self.f = self.g+self.h
		self.parent = parent
		self.loc = loc

#function to change grid locations to map coordinates
def getOrgCartCoord(row,col):
	return [col-9,10-row]

def getApproxCartCoord(row,col):
	return [col-9+0.5,10-row-0.5]

#function to change map locations to grid locations
def getGridCoord(x,y):
	return [int(10-y),int(x+9)]

#function to find the neighbours of the current point
def getNeighbours(loc):
	r = loc[0]
	c = loc[1]
	return [[r-1,c-1],[r-1,c],[r-1,c+1],[r,c-1],[r,c+1],[r+1,c-1],[r+1,c],[r+1,c+1]]

#function to move the robot when an obstacle is detected
def avoidObstacle():
	global velocity_publisher,rangedist
	vel = Twist()
	while rangedist < 2.0:
		vel.angular.z = -0.5
		velocity_publisher.publish(vel)
	s = time.time()
	while time.time() - s < 1:
		vel.linear.x = 0.5
		vel.angular.z = 0.0
		velocity_publisher.publish(vel)
	
#function to make the robot follow the path
def updateRoboPos(data):
	global currentPos,fpath,find,currentPos,startPos,rangedist,velocity_publisher
	rot = data.pose.pose.orientation
    	(roll, pitch, yaw) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
	currentPos = [data.pose.pose.position.x,data.pose.pose.position.y,yaw]
	vel = Twist()
	
	if find < len(fpath):
		if (abs(currentPos[0] - fpath[find][0]) > 0.2) or (abs(currentPos[1] - fpath[find][1]) > 0.2):
			if rangedist < 0.4:
				print("\nObstacle detected")
				avoidObstacle()
				vel.linear.x = 0.0
				vel.angular.z = 0.0
			else:
				angle = math.atan2(fpath[find][1]-currentPos[1],fpath[find][0]-currentPos[0])
				if abs(angle-currentPos[2]) > 0.2:
					vel.linear.x = 0.0
					vel.angular.z = angle-currentPos[2]
				else:
					vel.linear.x = 2
					vel.angular.z = 0.0
		else:
			find+=1	
			if find == len(fpath)-1:
				vel.linear.x = 0.0
				vel.angular.z = 0.0
				print("Target Reached")
		velocity_publisher.publish(vel)

	

#function to read the map and create a 2D grid
def getMap():
	global gridmap
	f = open(sys.argv[1],"r")
	gmap = f.read()
	f.close()
	gmap = gmap[7:len(gmap)-2]
	gmap = gmap.split()
	for el in gmap:
		tmp = np.array(el.split(","))
		arr = []
		for i in tmp:
			if i != '':
				arr.append(int(i))
		gridmap.append(arr)
	gridmap = np.array(gridmap)

#function to get the required point or neighbour from the list with the points remaining to be checked
def getNeighbourNode(loc):
	global uncheckedlist
	for nd in uncheckedlist:
	 	if nd.loc == loc:
			return nd

#function to generate the path from all the checked nodes by checking their parent. We start from the target location and move backwards
def getPath():
	global checkedlist
	nd = checkedlist[len(checkedlist)-1]
	path = []
	while nd.loc != checkedlist[0].loc:
		path.append(nd.loc)
		nd = nd.parent
	return path

def checkDiagMove(nb,ind):
	global gridmap
	if ind == 0:
		if (gridmap[nb[0],nb[1]+1] == 1) and (gridmap[nb[0]+1,nb[1]] == 1):
			return False
		else:
			return True
	elif ind == 2:
		if (gridmap[nb[0],nb[1]-1] == 1) and (gridmap[nb[0]+1,nb[1]] == 1):
			return False
		else:
			return True
	elif ind == 5:
		if (gridmap[nb[0]-1,nb[1]] == 1) and (gridmap[nb[0],nb[1]+1] == 1):
			return False
		else:
			return True
	elif ind == 7:
		if (gridmap[nb[0],nb[1]-1] == 1) and (gridmap[nb[0]-1,nb[1]] == 1):
			return False
		else:
			return True
	
	
#function to find the optimal path using A* algorithm	
def astar(goalx,goaly):
	global startPos,checkedlist,uncheckedlist,gridmap,horizcost,diagcost,startlocation
	startlocation = getGridCoord(startPos[0],startPos[1])
	goallocation = getGridCoord(goalx,goaly)
	startNode = node(loc=startlocation)
	goalNode = node(loc=goallocation)
	uncheckedlist.append(startNode)
	while len(uncheckedlist) > 0:
		minfnode = min(uncheckedlist,key = attrgetter('f'))
		uncheckedlist.remove(minfnode)
		checkedlist.append(minfnode)
		if minfnode.loc == goallocation:
			print("\nTarget location obtained")
			break
		neighbours = getNeighbours(minfnode.loc)
		for indn,nb in enumerate(neighbours):
			if (nb[0] < 0) or (nb[0] > len(gridmap)-1) or (nb[1] < 0) or (nb[1] > len(gridmap[0])-1) or (gridmap[nb[0],nb[1]] == 1) or any(nd.loc == nb for nd in checkedlist):
				continue
			if indn==0 or indn==2 or indn==5 or indn==7:
				if checkDiagMove(nb,indn) == False:
					continue
			g = 0			
			if (nb[0] == minfnode.loc[0]) or (nb[1] == minfnode.loc[1]):	
					g = minfnode.g + horizcost
			else:
					g = minfnode.g + diagcost
			h = np.square(goallocation[0]-nb[0]) + np.square(goallocation[1]-nb[1])
			if any(nd.loc == nb for nd in uncheckedlist):
				nbnode = getNeighbourNode(nb)				
				if g < nbnode.g:
					newnode = node(parent = minfnode,loc=nb,g=g,h=h)
					ind = uncheckedlist.index(nbnode)
					uncheckedlist[ind] = newnode
			else:
				newnode = node(parent=minfnode,loc=nb,g=g,h=h)
				uncheckedlist.append(newnode)
	path = getPath()
	return path

def baseScanFunc(data):
	global rangedist
	st = data.ranges[90:271]	
	rf = data.ranges[45]
	lf = data.ranges[135]
	rangedist = min(st)

#function to initiate robot path following
def followPath(path,goalx,goaly):
	global currentPos,startPos,rangedist
	ground_listener = rospy.Subscriber('base_pose_ground_truth', Odometry, updateRoboPos)
	scan = rospy.Subscriber('base_scan', LaserScan, baseScanFunc)
	vel = Twist()
	rate = rospy.Rate(10)
	rospy.spin()

#function to get the goal locations from the arguments and initiate A* algorithm
def start():
	global fpath,gridmap
	if rospy.has_param("goalx") and rospy.has_param("goaly"):
		goalx = rospy.get_param("goalx")
		goaly = rospy.get_param("goaly")
		path = astar(goalx,goaly)
		path.reverse()
		fullpath = [startlocation] + path
		print "\nFinal Grid Path: ",fullpath
		gridpath = []
		for el in fullpath:
			gridpath.append(getOrgCartCoord(el[0],el[1]))
		print "\nFinal World Path: ",gridpath
		for el in path:
			fpath.append(getApproxCartCoord(el[0],el[1]))
		fpath.append([goalx,goaly])
		followPath(fpath,goalx,goaly)
	else:
		print("Couldn't find the goal parameters")
		
if __name__ == '__main__':
	try:
	    rospy.init_node('astar', anonymous=True)
	    getMap()
	    start()
	except rospy.ROSInterruptException:
	    pass
		
		


