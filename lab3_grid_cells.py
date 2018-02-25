#!/usr/bin/env python

import rospy
from nav_msgs.msg import GridCells, Path
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
import tf
import numpy
import math 
import rospy, tf, numpy, math
from Astar import *



# reads in global map
def mapCallBack(data):
    global mapData
    global width
    global height
    global mapgrid
    global resolution
    global offsetX
    global offsetY
    mapgrid = data
    resolution = data.info.resolution
    mapData = data.data
    width = data.info.width
    height = data.info.height
    offsetX = data.info.origin.position.x
    offsetY = data.info.origin.position.y
    print data.info

def readGoal(goal):
	global goalX
	global goalY
	global goalPos
	goalX= goal.pose.position.x
	goalY= goal.pose.position.y
	goalPos = (goalX,goalY)
	#print goal.pose
	# Start Astar


def readStart(startPos):

    global startPosX
    global startPosY
    startPosX = startPos.pose.pose.position.x
    startPosY = startPos.pose.pose.position.y
    print startPos.pose.pose

def aStar(start,goal):
    pass
    # create a new instance of the map
	
    # generate a path to the start and end goals by searching through the neighbors, refer to aStar_explanied.py
    
    # for each node in the path, process the nodes to generate GridCells and Path messages

    # Publish points
    
#publishes map to rviz using gridcells type

# take in a list of nodes representing the path
def publishPath(nodeList):
	# make sure the nodeList contains enough elements to be a path
	if len(nodeList) < 2:
	        rospy.loginfo("NO PATH")
	        return

	my_path = Path() # create a new path message
	my_path.header.frame_id = 'map'
	
	# convert list of nodes into a path 
	for node in nodeList:
		pose = PoseStamped() # create a new PoseStamped message
		pose.header.frame_id = 'map'
		pose.header.stamp = rospy.Time.now()
		pose.pose.position.x = node.position[0]
		pose.pose.position.y = node.position[1]
		pose.pose.position.z = 0
		pose.pose.orientation.y = 0.0
      		pose.pose.orientation.z = 0.0
      		pose.pose.orientation.w = 1.0
		my_path.poses.append(pose)
	pubpath.publish(my_path)
	 

def publishCells(grid):
    global pub
    global cellMsg
    print "publishing"

    # resolution and offset of the map
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    # Resolution is set to 0.05m/cell, multiplying by 10 increases this to .5m/cell
    # This is approximately equal to the widest point on the turtlebot plus a little
    cells.cell_width = resolution 
    cells.cell_height = resolution

    for i in range(0,height): #height should be set to hieght of grid        
        for j in range(0,width): #width should be set to width of grid  
            #print k # used for debugging
            if (grid[k] == 100):
                point=Point()
                point.x=(j*resolution)+offsetX + (0.5 * resolution) # added secondary offset 
                point.y=(i*resolution)+offsetY + (0.5 * resolution) # added secondary offset ... Magic ?
                point.z=0
                cells.cells.append(point)
            k=k+1			
        #print "row: ", i+1
    #cells.cells = numpy.reshape(grid.data, (grid.info.width,grid.info.height))
    cellMsg = cells
    pub.publish(cells)

"""
def superCell(cells)
	for x in numpy.arange(0, cells.cell_width, 10)
		for y in numpy.arange(0, cells.cell_height, 10)
			
			for m in numpy.arange(x, x+10)
				for n in numpy.arange(y, y+10)
					cellList.append(x,y)         
"""

#Main handler of the project
def run():
	"""
	Future selves:
	1. The gridVal calculation in the expandNode function of Astar is broken and needs to be fixed
	2. gScore needs to be calculated in a non-recursive way
	3. The map is still missing one wall
	4. Astar may be broken, needs more testing
	5. Need to create waypoint to path logic (synthesize waypoints to remove redundant waypoints)
	"""
	global pub
	global pubpath
	global pubGrid

	rospy.init_node('lab3')
	sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
	pub = rospy.Publisher("/map_check", GridCells, queue_size=1)
	pubpath = rospy.Publisher("/path", Path, queue_size=1) # you can use other types if desired
	pubGrid = rospy.Publisher("/frontier", GridCells, queue_size=1)    
	pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
	goal_sub = rospy.Subscriber('move_base_simple/goal2', PoseStamped, readGoal, queue_size=1) #change topic for best results
	goal_sub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, readStart, queue_size=1) #change topic for best results

	# wait a second for publisher, subscribers, and TF
	rospy.sleep(5)
	path = []

	while not path and not rospy.is_shutdown():
		publishCells(mapData) #publishing map data every 2 seconds
		children = []
		startPos = (startPosX, startPosY)
		startNode = expandNode(mapgrid, cellMsg, None, startPos, cellMsg.cell_width)
		path = astar(startNode, goalPos, mapgrid, cellMsg, startPos)
		publishPath(path)

	rospy.sleep(2)  
	print("Complete")


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
