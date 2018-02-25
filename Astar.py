from Node import *
from Queue import PriorityQueue as pq
import math,numpy,rospy
import time
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
import copy
import rospy
from lab3_grid_cells import run

# closed list is a list called explored, 
# containing all the nodes the algorithm has expanded
explored = []
# open list is a priority queue called frontier, 
# containing the unexplored children of frontier nodes
frontier = pq()
frontierDisplay = [] # create an empty list to hold nodes

def astar(startNode, goal, mapData, grid, startPos):
	"""
	
	"""
	# Calculate travel distance from one cell to another
	# d1 is the orthognal distance from one cell to another
	d1 = grid.cell_width
	# d2 is the diagonal distance from one cell to another
	d2 = math.sqrt(2*(d1**2))

	# Add the starting node to the frontier
	frontier.put(startNode)

	offsetX = mapData.info.origin.position.x
	offsetY = mapData.info.origin.position.y
	resolution = mapData.info.resolution
	height = mapData.info.height
	width = mapData.info.width
	mapSize = width*height

	# Initialize path list
	path = []

	# While there are still nodes to explore on the frontier
	while not frontier.empty() and not rospy.is_shutdown():
		displayFrontier()
		# Get the lowest f-score node from the frontier and make it the current node
		currentNode = frontier.get()
		#if currentNode in frontierDisplay:
			#frontierDisplay.remove(currentNode) # remove the current Node from the froniterDisplay list

		# If the current node is the goal
		if isGoal(currentNode,goal,d1):
			# Generate a path from the goal to the start
			print "At goal!"
			path = generateNodePath(currentNode)
			break
		# Else if the current node is not at the goal
		else:
			# Add the current node to the list of explored nodes
			explored.append(currentNode)
			#print(explored)
			# Create all the children and calculate their g,h,f scores
			# This function will not create duplicate children (ignores nodes already in the explored list)
			children = evalChildren(expandNode(mapData, grid, currentNode, startPos,d1),goal,d1,d2)
			# Add all children to frontier
			#print "Children: ", children
			for child in children:
				frontier.put(child)
				frontierDisplay.append(child)
	print "Path: ", path
	return path

def hScore(node, goal, d1, d2):
	"""
	Calculates the h score of a node using Diagonal distance as the heuristic
	Takes in the node to be evaluated (Node), the goal coordinates (int tuple), the orthognal distance between cells (int), and the diagonal distance between cells (int)
	Returns the h score of the node (int)
	"""
	#return math.sqrt((goal[0]-node.position[0])**2 + (goal[1]-node.position[1])**2)
	dx = abs(node.position[0] - goal[0])
	dy = abs(node.position[1] - goal[1])
	# In this instance, d is the shortest distance between the two grid cells, which will be the width or the height assuming the grid is square
	# D2 is the diagonal distance between the grids, which will be sqrt(width^2+height^2) or sqrt(2(D^2)) in a square grid
	# When D = 1 and D2 = sqrt(2), this is the octile distance.
	#return d1 * (dx + dy) + (d2 - 2 * d1) * min(dx, dy)
	return math.sqrt(dx**2 + dy**2)

def gScore(node,count):
    """
    Calculates the g score of a node recursively, assuming diagonal movement
    Takes in the node to be evaulated (Node) and the count so far (int)
    Returns the g score of the node (int)
    # If the node is the start node, return the count
    if node.parent is None:
        return count
    # If the node is not the start node
    else:
        # Calculate the distance moved in the x and y directions based on the difference betweent the child and parent x and y positions
        dx = abs(node.position[0] - node.parent.position[0])
        dy = abs(node.position[1] - node.parent.position[1])
        print("Current Node pos:%s" % (node.position,))
        print("Node score:%d" % (count + math.sqrt(dx**2 + dy**2)))
        return gScore(node.parent, count + math.sqrt(dx**2 + dy**2))
    """
    curNode = node
    score = 0
    while curNode.parent is not None and not rospy.is_shutdown():
        dx = abs(node.position[0] - node.parent.position[0])
        dy = abs(node.position[1] - node.parent.position[1])
        score += math.sqrt(dx**2 + dy**2)
        #print("Current Node pos:%s" % (node.position,))
        curNode = curNode.parent
    return score
     
def fScore(node, goal, d1, d2): # Could be refactored to use object properties instead of calling functions
    """
    Calculates the f score of a node based on the g and h scores
    Takes in the node to be evaluated (Node), the goal coordinates (int tuple), the orthognal distance between cells (int), and the diagonal distance between cells (int)
    Returns the f score of the node (int)
    """
    return gScore(node, 0) + hScore(node, goal, d1, d2)

def calcScore(node, goal, d1, d2):
    """
    Calculates the g, h, and f scores of a given node using the gScore, hScore, and fScore helper functions
    Takes in the node to be evaluated (Node), the goal coordinates (int tuple), the orthognal distance between cells (int), and the diagonal distance between cells (int)
    Returns the g, h, and f scores of the node (int, int, int)
    """
    g = gScore(node, 0)
    h = hScore(node, goal, d1, d2)
    f = fScore(node, goal, d1, d2)
    return (g,h,f)

def isGoal(node,goal,d1):
	"""
	Checks if the given node is the goal node
	Takes in the node to be evaluated (Node) and the goal coordinates (int tuple)
	Returns true if positions match, false otherwise (Boolean)
	"""
	#return (numpy.all(numpy.absolute(tuple(numpy.subtract(node.position,goal)))) < d1)
	minX = goal[0] - (d1/2)
	maxX = goal[0] + (d1/2)
	minY = goal[1] - (d1/2)
	maxY = goal[1] + (d1/2)
	if minX < node.position[0] and node.position[0] < maxX and minY < node.position[1] and node.position[1] < maxY:
		return True

def evalChildren(nodes, goal, d1, d2):
	"""
	Calculates the g, h, and f scores for a list of nodes
	Takes in a list of nodes ([Node]), the goal coordinates (int tuple), the orthognal distance between cells (int), and the diagonal distance between cells (int)
	Returns the list of nodes ([Node]) with updated g, h, and f scores
	"""
	for node in nodes:
		node.g, node.h, node.f = calcScore(node, goal, d1, d2)
	return nodes

def generateNodePath(node):
	"""
	Generates a path from the given node to the root node by tracing back through the node's parents
	Takes in a node to be evaluated (Node)
	Returns a list of nodes in the path from the given node to the root node ([Node])
	"""
	curNode = node
	nodeList = []
	while True:
		if curNode.parent is None:
			nodeList.append(curNode)
			break
		nodeList.append(curNode)
		curNode = curNode.parent
	return nodeList

def expandNode(mapData, grid, currentNode, startPos, d1):
    """
    Given a node, returns a list of the node's children. 
    This function will ignore nodes that have already been explored to avoid creating duplicates. 
    The neighbour nodes will be created in the following pattern:
    [1] [4] [6]
    [2] [C] [7]
    [3] [5] [8] 
    where 'C' is the current node. 
    Takes in map data (OccupancyGrid), grid data (GridCells), the node to be evaluated (Node), and the starting pose (poseStamped)
    Returns a list of nodes ([Node])
    """
    offsetX = mapData.info.origin.position.x
    offsetY = mapData.info.origin.position.y
    resolution = mapData.info.resolution
    height = mapData.info.height
    width = mapData.info.width
    mapSize = width*height

    # If there is no current node, create and return a root node with no parent
    if currentNode is None:
        node = Node((startPos[0], startPos[1]),None)
        return node
    # If there is a current node, create children nodes for its neighbours
    else:
        # Create an empty list to store children nodes
        children = []
        # Calculate the current node's xy position from meters to grid cells
        parentX = int((currentNode.position[0] - offsetX - (.5 * resolution)) / resolution)
        parentY = int((currentNode.position[1] - offsetY - (.5 * resolution)) / resolution)

        # Iterate through neighbouring nodes to create children
        for x in xrange(-1,2):
            for y in xrange(-1,2):
                # Calculate the x and y position of the new node in grid cells
                # This is used to calculate the index of the occupancy grid value for this cell
                currGridX = parentX+x
                currGridY = parentY+y
                # Calculate the index in Row-Major form to find its threshold in the occupancy grid
                gridVal = currGridX + ((currGridY)*width)
                # Find the x and y position of the new node
                newX = (currentNode.position[0])+(x*d1)
                newY = (currentNode.position[1])+(y*d1)
                #  Creates a child node with the xy position calculated above and the current node as the parent
                node = Node((newX, newY),currentNode)
                #print "gcxy: ", currGridX, currGridY
                #print "cxy: ", currX, currY
                #print node.position
                # Check that the new child node is not already explored or an object or outside the bounds of the map
                #print "gv: ", gridVal
                #print "ms: ", mapSize
                #print "ov: ", mapData.data[int(gridVal)] 
                if node not in explored and gridVal >= 0 and gridVal <= mapSize and (mapData.data[int(gridVal)] >= 0) and (mapData.data[int(gridVal)] < 100):
                    # add new node to the list of children
                    children.append(node)
        return children

# takes in nothing, makes a copy of frontier queue and creates a grid cells message
# returns a grid cells message
def displayFrontier():
	rospy.init_node('lab3')
	pubGrid = rospy.Publisher("/frontier", GridCells, queue_size=1)
	pubGrid2 = rospy.Publisher("/explored", GridCells, queue_size=1)        
	frontierCells = GridCells() # create a new GridCells message
	frontierCells.header.frame_id = 'map'
	frontierCells.cell_width = 0.3000  # resolution value from map.yaml  file
	frontierCells.cell_height = 0.3000

	exploredCells = GridCells() # create a new GridCells message
	exploredCells.header.frame_id = 'map'
	exploredCells.cell_width = 0.3000  # resolution value from map.yaml  file
	exploredCells.cell_height = 0.3000

	for node in frontierDisplay:
		point=Point()
		point.x= node.position[0]
		point.y= node.position[1]
		point.z= 0
		frontierCells.cells.append(point)

	for node2 in explored:
		point=Point()
		point.x= node2.position[0]
		point.y= node2.position[1]
		point.z= 0
		exploredCells.cells.append(point)
		
	pubGrid.publish(frontierCells) 
	pubGrid2.publish(exploredCells)
		
if __name__ == "__main__":
    # Create test nodes
    node1 = Node((1,1), None) 
    node1.f = 2
    node2 = Node((2,1), node1) 
    node2.f = 3
    node3 = Node((3,1), node1) 
    node3.f = 1

    # Tests
    #print gScore(node2,0)
    #print hScore(node1, (5,5))
    #print fScore(node1, (5,5))
    
    frontier.put(node1)
    frontier.put(node2)
    frontier.put(node3)
    #astar(node1, node2)
