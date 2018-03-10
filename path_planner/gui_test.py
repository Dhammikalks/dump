from Tkinter import *

# The world extents in units.
world_extents = (200, 150)

# The array of visited cells during search.
visited_nodes = None
#array for world obstacle


# Switch which determines if visited nodes shall be drawn in the GUI.
show_visited_nodes = True

# Switch which determines if potential function should be used.
use_potential_function = True

world_obstacles = np.zeros(world_extents, dtype=np.uint8)

# The optimal path between start and goal. This is a list of (x,y) pairs.
optimal_path = []

goal = []
cod = []

def set_goal(*args):
    global goal
    global cod
    goal = cod
    master.after(0, loop)

def codinate(event):
    global cod
    cod =[event.x,event.y]

#.........................................................
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
# ..................................................
def SQL_CMD(con, query):
    cur = con.cursor()
    cur.execute(query)
    con.commit()

# ..................................................
def SQL_CMD_getData(con, query):
    cur = con.cursor()
    cur.execute(query)
    result = cur.fetchall()
    return result
# .............................................clear_map
def clear_obstacles():
    global world_obstacles
    world_obstacles = np.zeros(world_extents, dtype=np.uint8)

#...................................................
def base(x,y,x_base_p,y_base_p):
    x_base = x_base_p
    y_base = y_base_p
    if x > self.max[0]:
        self.max[0] = x + 10
        x_base = float(200.0 / float(self.max[0] + 1))
    if y > self.max[1] :
        self.max[1] = y + 10
        y_base = float(150.0 / float(self.max[1] + 1))

    return x_base,y_base

# .....................................................................add_obstacle
def add_obstacle(obstacle_array,pos):
    set_obstacle(obstacle_array, pos, True)


def set_obstacle(self, obstacle_array, pos, on):
    """Draw an obstacle (2N+1 x 2N+1) box into numpy obstacle_array."""
    N = 2  # Box size will be 2N+1
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
    global world_obstacles
    world_obstacles = obstacle_array


# ..............................................
def update_callback(start, goal):
    # First apply distance transform to world_obstacles.
    apply_distance_transform()
    # Call path planning algorithm.
    if not (start == None or goal == None):
        global optimal_path
        global visited_nodes
        try:
            global world_obstacles
            optimal_path, visited_nodes = astar(start, goal, world_obstacles)
            # print optimal_path
            # print visited_nodes
        except Exception, e:
            print traceback.print_exc()
    # Draw new background.
    global world_obstacles
    draw_background(world_obstacles, visited_nodes, optimal_path,
                         show_visited_nodes)
    return optimal_path
#.............................................................................
def draw_background(obstacle_array, visited_array, path,
                        show_visited=True):
    red = (obstacle_array == 255) * np.uint8(255)
    blue = obstacle_array - red
    if not show_visited or visited_array == None:
        green = np.zeros(obstacle_array.shape, dtype=np.uint8)
    else:
        max_dist = np.amax(visited_array)
        if max_dist > 0:
            green = np.uint8(visited_array * (255.0 / max_dist))
        else:
            green = np.zeros(obstacle_array.shape, dtype=np.uint8)
    set_background(np.dstack((red.T, green.T, blue.T)), color=True)
    self.set_path(path)
#..................................................
def set_background(self, np_array, color=False):
    """Takes a (numpy) array and sets this as background."""
    if color:
        img = Image.fromarray(np.flipud(np.uint8(np_array)), mode="RGB")
    else:
        img = Image.fromarray(np_array)

    if self.background_image_id:
        self.world_canvas.delete(self.background_image_id)
    img = img.resize(self.extents, Image.NEAREST)
    self.background_image = ImageTk.PhotoImage(img)
    self.background_id = self.world_canvas.create_image(0, 0,
                                                            image=self.background_image, anchor=NW, tag="background")
    # Make sure drawing order is correct.
    self.set_display_order()
#..............................................................................

def apply_distance_transform():
    global world_obstacles
    if use_potential_function and np.max(world_obstacles) == 255:
        # Compute distance transform.
        dist_transform = 255 - np.minimum(
            64 * scipy.ndimage.morphology.distance_transform_edt(
                255 - world_obstacles), 255)
        m = max(np.max(dist_transform), 1)  # Prevent m==0.
        global world_obstacles
        world_obstacles = np.uint8((dist_transform * 255) / m)
    else:
        # Keep 255 values only (set all other to 0).
        global world_obstacles
        world_obstacles = (self.world_obstacles == 255) * np.uint8(255)

#.........................................................
# --------------------------------------------------------------------------
# A* algorithm.
# --------------------------------------------------------------------------

# Allowed movements and costs on the grid.
# Each tuple is: (movement_x, movement_y, cost).
s2 = np.sqrt(2)
movements = [ # Direct neighbors (4N).
              (1,0, 1.), (0,1, 1.), (-1,0, 1.), (0,-1, 1.),
              # Diagonal neighbors.
              # Comment this out to play with 4N only (faster).
              (1,1, s2), (-1,1, s2), (-1,-1, s2), (1,-1, s2),
            ]

def distance(p, q):
    """Helper function to compute distance between two points."""
    return np.sqrt((p[0]-q[0])**2 + (p[1]-q[1])**2)

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
                if visited[new_pos] == 0 and obstacles[new_pos] < 255:
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

#.........................................................
def draw(connection):
    quary = "SELECT * FROM Data_ref WHERE is_New = '1' LIMIT 1"
    result = SQL_CMD_getData(connection, quary)
    if result != "":

        quary_get = "SELECT * FROM SLAM WHERE No = '" + str(result[0][0]) + "'"
        result_data = SQL_CMD_getData(connection, quary_get)
        data = Data()
        data.read(result_data)

        # .....................................
        clear_obstacles()  # clean old obstacle data
        self.x_base, self.y_base = base(data.filtered_positions[0], data.filtered_positions[1], self.x_base,
                                             self.y_base)

        for i in data.world_cylinders[0]:
            (x, y) = i
            self.x_base, self.y_base = base(x, y, self.x_base, self.y_base)

            global world_obstacles
            add_obstacle(world_obstacles,
                              (int(x * self.x_base), int(y * self.y_base)))  # add new obstacle data

        a = (int(data.filtered_positions[0] * self.x_base), int(data.filtered_positions[1] * self.y_base))
        # a = tuple([(int(i*0.05)) )
        global goal
        print goal
        b = (int(goal[0] * self.x_base), int(goal[1] * self.y_base))
        path = update_callback(a, b)
        #
        ##################################
        quary_path = "INSERT INTO Path_planner( path, X_base, Y_base) values('" + str(path) + "','" + str(
            self.x_base) + "','" + str(self.y_base) + "')"
        self.SQL_CMD(connection, quary_path)
        ##################################
        #
        # ....................................
        quary2 = "UPDATE Data_ref SET is_New = '0' WHERE No = '" + str(result[0][0]) + "'"
        self.SQL_CMD(connection, quary2)
    else:
        time.sleep(0.2)
    master.after(100, self.draw, connection)
#.........................................................


def loop():
   global goal
   print('loop')
   print (": Coordinate---->")
   print(goal)
   # Infinite loop without delay is bad idea.
   master.after(100, loop)

if __name__ =='__main__':

    master = Tk()
    pressed = False
    canvas= Canvas(master, width=200, height=200)
    canvas.bind("<Button-1>", set_goal)
    canvas.bind("<Motion>", codinate)
    canvas.pack()
    mainloop()