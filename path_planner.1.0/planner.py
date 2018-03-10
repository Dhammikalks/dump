import MySQLdb
import numpy as np
import heapq
import time
import traceback
import Pyro4
import scipy.ndimage
from math import sin, cos, atan2
import heapq


# The world extents in units.
canvas_extents = [200, 200]
world_extents = [2000.0,2000.0]

MAX  = [1,1]
base =[1,1]
potential = False
display_factor = canvas_extents[0]/world_extents[0];
# --------------------------------------------------------------------------------
# A* algorithm.
# --------------------------------------------------------------------------------

# Allowed movements and costs on the grid.
# Each tuple is: (movement_x, movement_y, cost).
s2 = np.sqrt(2)
movements = [ # Direct neighbors (4N).
              (1,0, 1.), (0,1, 1.), (-1,0, 1.), (0,-1, 1.),
              # Diagonal neighbors.
              # Comment this out to play with 4N only (faster).
              (1,1, s2), (-1,1, s2), (-1,-1, s2), (1,-1, s2),
            ]
#..........................................................................
def distance(p, q):
    """Helper function to compute distance between two points."""
    return np.sqrt((p[0]-q[0])**2 + (p[1]-q[1])**2)
#..........................................................................
def astar(start, goal, obstacles):
    """A* algorithm."""
    # In the beginning, the start is the only element in our front.
    # NOW, the first element is the total cost through the point, which is
    #   the cost from start to point plus the estimated cost to the goal.
    # The second element is the cost of the path from the start to the point.
    # The third element is the position (cell) of the point.
    # The fourth component is the position we came from when entering the tuple
    #   to the front.

    front = [(0.001 + distance(start, goal), 0.001, start, None)]
    # In the beginning, no cell has been visited.
    extents = obstacles.shape
    visited = np.zeros(extents, dtype=np.float32)

    # Also, we use a dictionary to remember where we came from.
    came_from = {}

    # While there are elements to investigate in our front.
    while front:
        # Get smallest item and remove from front.
        element = heapq.heappop(front)
        # Check if this has been visited already.
        # CHANGE 01_e: use the following line as shown.
        total_cost, cost, pos, previous = element

        if visited[pos] > 0:
            continue
        visited[pos] = cost
        # Now it has been visited. Mark with cost.

        # Also remember that we came from previous when we marked pos.
        came_from[pos] = previous
        # Check if the goal has been reached.
        # Get smallest item and remove from front.

        # Check if this has been visited already.

        # Now it has been visited. Mark with cost.

        # Also remember that we came from previous when we marked pos.

        # Check if the goal has been reached.
        if pos == goal:
            break  # Finished!

        # Check all neighbors.
        for dx, dy, deltacost in movements:
            # Determine new position and check bounds.
            # Determine new position and check bounds.
            old_x, old_y = pos
            new_x = old_x + dx
            new_y = old_y + dy
            if (new_x >= 0 and new_x < extents[0]) and (new_y >= 0 and new_y < extents[1]):
                # Add to front if: not visited before and no obstacle.
                new_pos = (new_x, new_y)
                if visited[new_pos] == 0 and obstacles[new_pos] < 200:
                    heapq.heappush(front, (cost + deltacost+distance(new_pos,goal)+
                                           (obstacles[new_pos] / 64),cost + deltacost+(obstacles[new_pos] / 64), new_pos, pos))
            # CHANGE 01_f: add the 'obstacle cost' to new_cost AND
            #   new_total_cost. As obstacle cost, use:
            #   obstacles[new_pos] / 64.
            #   The divider 64 determines the tradeoff between 'avoiding
            #   obstacles' and 'driving longer distances'. You may experiment
            #   with other values, but make sure you set it back to 64 for
            #   the grader.
            # Please check again that you do not enter a tuple into
            # the heap if it has been visited already or its obstacles[]
            # value is 255 (check for '==255', not for '> 0').

            else:
                continue
    # Reconstruct path, starting from goal.
    path = []
    if pos == goal:  # If we reached the goal, unwind backwards.
        while pos:
            path.append(pos)
            pos = came_from[pos]
        path.reverse()  # Reverse so that path is from start to goal.

    return (path, visited)
#..............................................................................
def connect():
    con = MySQLdb.connect(host='localhost', user='root', passwd='DUKS1992', db='ROBOT')
    try:

        cur = con.cursor()
        con.commit()
        # data = cur.fetchall()e
    except MySQLdb.Error, e:
        if con:
            con.rollback()
            con
    return con

# ..................................................
def SQL_CMD( con, query):
    cur = con.cursor()
    cur.execute(query)
    con.commit()


# ..................................................
def SQL_CMD_getData( con, query):
    cur = con.cursor()
    cur.execute(query)
    result = cur.fetchall()
    cur.close()
    if  not len(result) :
        return False
    else:
        return result

#..................................................
class Data(object):
#.....................................................
    def __init__(self):
        self.filtered_positions = []  #F
        self.world_cylinders = [] #WC
    #.....................................................
    def read(self,data):
        position =  [float(i.strip('.')) for i in data[0][2].strip('[]').split(',')]  # postion
        self.filtered_positions = position[0:2]
        cyliner_list = []
        for i in data[0][5].strip('[[]]').split('], ['):        #cylinders
            cyliner_list.append(tuple(map(float,i.split(','))))
        self.world_cylinders.append(cyliner_list)
          # cylinder pos
#..........................................................................
def set_obstacle(obstacle_array, pos, on):
    """Draw an obstacle (2N+1 x 2N+1) box into numpy obstacle_array."""
    N = 8  # Box size will be 2N+1
    # Handle borders.

    extents = obstacle_array.shape
    x, y = pos
    l = max(0, x - N)
    r = min(extents[0] - 1, x + N)
    d = max(0, y - N)
    u = min(extents[1] - 1, y + N)
    if l <= r and d <= u:
        if on:
            mask = np.ones((r - l + 1, u - d + 1)) * 255

        else:
            mask = np.zeros((r - l + 1, u - d + 1))
        obstacle_array[l:r + 1, d:u + 1] = mask
    return obstacle_array
#..........................................................................
def update_path(start, goal,world_obstacles):
    # First apply distance transform to world_obstacles.
    # Call path planning algorithm.
    optimal_path, visited_nodes = astar(start,goal, world_obstacles)

    print  optimal_path;
    return optimal_path,visited_nodes;
#.........................................................................
def to_canvas(codinate):
    (x,y) = codinate
    return [int(x*display_factor),(canvas_extents[0]-int(y*display_factor))]

#.......................................................................................
if __name__ == '__main__':
    ##############..............................Pyro4 elements
    try:
       control = Pyro4.Proxy("PYRONAME:example.control")  # use name server object lookup uri shortcut
    except:
        Pyro4.errors.CommunicationError
    #############..............................

    #########################################
    #control.set_Goal([0,0])
    control.set_Potential(True)

    while(True):
        connection = connect()

        #########################################
        quary = "SELECT * FROM Data_ref WHERE is_New = '1' LIMIT 1"
        result = SQL_CMD_getData(connection, quary)
        #########################################
        if result:
            print "caluculating the path..."
            quary_get = "SELECT * FROM SLAM WHERE No = '" + str(result[0][0]) + "'"
            result_data = SQL_CMD_getData(connection, quary_get)
            data = Data()
            data.read(result_data)


            goal = control.get_Goal()
            print "Goal : "
            print goal
            #############...............................
            print(float(canvas_extents[0] / world_extents[0]));
            world_obstacles = np.zeros(canvas_extents, dtype=np.uint8)

            potential = control.is_Potential()
            #############..............................

            for i in data.world_cylinders[0]:
                [x,y] = to_canvas(i)
                pos =(x,y)
                world_obstacles = set_obstacle(world_obstacles,pos, 1); # change the value of the True to no if you need potentioal area to be on

            #..........................................distance transform
            dist_transform = 255 - np.minimum(
               64 * scipy.ndimage.morphology.distance_transform_edt(
                    255 - world_obstacles), 255)
            m = max(np.max(dist_transform), 1)
            world_obstacles = np.uint8((dist_transform * 255) / m)
            #..........................................

            print(data.filtered_positions[1])
            [x,y] =to_canvas(data.filtered_positions[0:2])
            origin = (x,y)
            if (x >= canvas_extents[0]):
                x = x - 1;
            if (y >= canvas_extents[1]):
                y = y -1;
            (X,Y) = goal
            if(X >= canvas_extents[0]):
                X = X -1;
            if(Y >= canvas_extents[1]):
                Y = Y -1;
            path,visited_nodes = update_path((X,canvas_extents[1]-Y),(x,y), world_obstacles)

            ########################---seting up visited data
            control.setObstacleSet(world_obstacles.tolist())
            control.setVisitedSet(visited_nodes.tolist())
            control.setVisisted(True)
            ################################## // make old path data obsalte
            quary_old = "UPDATE Path_planner SET isNew = '0' WHERE isNew = '1' LIMIT 1"
            SQL_CMD(connection, quary_old)
            ##################################// make new data aivilable
            quary_path = "INSERT INTO Path_planner( path, slamref, goal, X_base, Y_base, isNew ) values('" + str(path) + "','"+ str(result[0][0])+"','"+str(goal)+"','"+ str(base[0])+"','"+ str(base[1]) +"','"+ str(1)+"')"
            SQL_CMD(connection, quary_path)
            ##################################//make old obstacle data obsalate
            quary2 = "UPDATE Data_ref SET is_New = '0' WHERE No = '" + str(result[0][0]) + "'"
            SQL_CMD(connection, quary2)
            ##################################
            print "end of the cycle..."
        else:
            print "waiting..."
            time.sleep(0.2)
#.........................................................................................
