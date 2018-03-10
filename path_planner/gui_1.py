import numpy as np
from math import sin, cos, atan2
import heapq
from Tkinter import *
from PIL import Image, ImageTk
# For Tkinter error handler.
import traceback
import tkMessageBox
import MySQLdb
import scipy.ndimage
import time
# The world extents in units.
world_extents = (200, 150)

# The array of visited cells during search.
visited_nodes = None
#array for world obstacle


# Switch which determines if visited nodes shall be drawn in the GUI.
show_visited_nodes = True

# Switch which determines if potential function should be used.
use_potential_function = True

# The optimal path between start and goal. This is a list of (x,y) pairs.
optimal_path = []

#.......................................goal coordinate
goal = []
cod = [50,50]

set_goal = False
set_cod = [50,50]
#.......................................
orignal = True
#.......................................

# Tkinter error handler (set below in main class).
#..................................................
def show_error(self, *args):
    err = traceback.format_exception(*args)
    tkMessageBox.showerror('Exception',err)
#..................................................
def coordinate(event):
    global cod
    cod =[event.x,event.y]
#..................................................
class GUI(Frame):

    def __init__(self,display_factor,start_goal_mode = None,
                 window_title = "Path Planning",*args,**kwargs):
        Frame.__init__(self, *args, **kwargs)

        # Set error handler.
        report_callback_exception = show_error

        # Setup GUI elements.
        self.world_extents = world_extents
        self.display_factor = display_factor
        self.extents = tuple(map(lambda v: v*self.display_factor,
                                 self.world_extents))
        self.root = Toplevel()
        self.root.title(window_title)
        self.frame1 = Frame(self.root)
        self.frame1.pack()
        self.world_canvas = Canvas(self.frame1,width=self.extents[0],
                                   height=self.extents[1], bg="black")


        #...............................................
        self.root.bind("<Button-1>", self.set_goal)
        self.root.bind("<Motion>", coordinate)
        self.root.bind("<Button-2>", self.drag_goal)
        #...............................................
        self.world_canvas.pack()


        self.frame2 = Frame(self.root)
        self.frame2.pack()
        self.max  = [1,1]
        self.x_base = 0
        self.y_base = 0

        self.start_goal_coordinates = [None, None]
        self.start_goal_canvas_ids = [None, None]
        self.start_goal_mode = "not"
        # Make optional buttons.
        self.world_obstacles = np.zeros(world_extents, dtype=np.uint8)

        # update_callback is the method doing the path planing
        extra_buttons = [("Use Potential Function", self.toggle_potential_function),
                   ("Show Visited", self.toggle_visited_nodes)]

        for b in extra_buttons:
            button = Button(self.frame2, text=b[0], command=b[1])
            button.pack(side=LEFT, padx=2)

        # Make quit button.
        self.quit_button = Button(self.frame2, text="Quit", command=quit)
        self.quit_button.pack(side=LEFT, padx=2)

        # No background object.
        self.background_image_id = None

        #get connection
        #........................
        #connection = self.connect()
        #self.draw(connection)
        #.........................

#..................................................
    def set_goal(self,*args):
        global orignal

        orignal = False
        self.frame1.update()
        self.world_obstacles = np.zeros(world_extents, dtype=np.uint8)
        self.world_canvas.delete(self.background_image_id)
        # No background object.
        self.background_image_id = None
        global  goal
        global cod
        goal =  cod
        connection = self.connect()
        orignal = True


        self.draw(connection)
# .................................................
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

# .............................................................................
    def draw_background(self, obstacle_array, visited_array, path,
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
        self.set_path(path)
        self.set_background(np.dstack((red.T, green.T, blue.T)), color=True)


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
#..................................................
    def set_path(self, path, color="white"):
        """Add a path to plot. If path is empty or None, delete old path."""
        self.world_canvas.delete("path")
        if path:

            disp_path = [self.to_display(p) for p in path]
            i = self.world_canvas.create_line(disp_path, tag="path",
                                              fill=color, width=3)
            # Make sure drawing order is correct.
            self.set_display_order()
#...........................................................................
# .....................................................................add_obstacle
    def add_obstacle(self,obstacle_array,pos):
        self.set_obstacle(obstacle_array, pos, True)

    # .............................................remove_obstacle
    def remove_obstacle(self,obstacle_array, pos):
        self.set_obstacle(obstacle_array, pos, False)

    # .............................................clear_map
    def clear_obstacles(self):
        self.world_obstacles = np.zeros(world_extents, dtype=np.uint8)

    # .............................................show visited_node
    def toggle_visited_nodes(self):
        global show_visited_nodes
        show_visited_nodes = not show_visited_nodes

    # ..............................................toggle potential function
    def toggle_potential_function(self):
        global use_potential_function
        use_potential_function = not use_potential_function
        self.update_callback()
    #...........................................................draw
    #...........................................................
    def draw(self,connection):

        quary = "SELECT * FROM Data_ref WHERE is_New = '1' LIMIT 1"
        result = self.SQL_CMD_getData(connection, quary)
        if result != "":

            quary_get = "SELECT * FROM SLAM WHERE No = '" + str(result[0][0]) + "'"
            result_data = self.SQL_CMD_getData(connection, quary_get)
            data = Data()
            data.read(result_data)

            #.....................................
            self.clear_obstacles() # clean old obstacle data
            self.x_base,self.y_base =self.base(data.filtered_positions[0],data.filtered_positions[1],self.x_base,self.y_base)

            for i in  data.world_cylinders[0]:
                (x,y) = i
                self.x_base,self.y_base = self.base(x,y,self.x_base,self.y_base)
                self.add_obstacle(self.world_obstacles,(int(x*self.x_base),int(y*self.y_base))) # add new obstacle data

            a =(int(data.filtered_positions[0]*self.x_base),int(data.filtered_positions[1]*self.y_base))
            global goal
            gl = self.to_world(goal)
            path = self.update_callback(a,gl,self.x_base,self.y_base)

            ##################################
            quary_path = "INSERT INTO Path_planner( path, X_base, Y_base) values('"+str(path)+"','"+str(self.x_base)+"','"+str(self.y_base)+"')"
            self.SQL_CMD(connection,quary_path)
            ##################################
            #
            #....................................
            quary2 = "UPDATE Data_ref SET is_New = '0' WHERE No = '" + str(result[0][0]) + "'"
            self.SQL_CMD(connection, quary2)
        else:
            time.sleep(0.2)
        global orignal
        if orignal:
            self.after(100, self.draw, connection)
#...................................................
    def base(self,x,y,x_base_p,y_base_p):
        x_base = x_base_p
        y_base = y_base_p
        if x > self.max[0]:
            self.max[0] = x + 10
            x_base = float(200.0 / float(self.max[0] + 1))
        if y > self.max[1] :
           self.max[1] = y + 10
           y_base = float(150.0 / float(self.max[1] + 1))

        return x_base,y_base
#...................................................connect
    def connect(self):
        con = MySQLdb.connect(host='localhost', user='root', passwd='DUKS1992', db='ROBOT')
        try:

            cur = con.cursor()
            con.commit()
            # data = cur.fetchall()e
        except MySQLdb.Error, e:
            if con:
                con.rollback()
        return con

# ..................................................
    def SQL_CMD(self, con, query):
        cur = con.cursor()
        cur.execute(query)
        con.commit()

# ..................................................
    def SQL_CMD_getData(self, con, query):
        cur = con.cursor()
        cur.execute(query)
        result = cur.fetchall()
        return result
#...................................................
#...................................................

    def to_world(self, coord):
        """Transform display coordinates to world."""
        if coord:
            return (coord[0] / self.display_factor,
                    (self.extents[1]-coord[1]-1) / self.display_factor)
        else:
            return None
    #...............................................
    def to_display(self, coord):
        """Transform display coordinates to world."""
        if coord:
            return (coord[0] * self.display_factor,
                    self.extents[1]-coord[1]*self.display_factor-1)
        else:
            return None
    #...............................................
    #...............................................
    def set_display_order(self):
        self.world_canvas.tag_lower("start")
        self.world_canvas.tag_lower("goal")
        self.world_canvas.tag_lower("path")
        self.world_canvas.tag_lower("background")
    #...............................................
    def run(self):
        """Enter the main loop (will not return)."""
        self.root.mainloop()
        #self.root.destroy()
    #...................................................................
        #self.set_display_order()
    #...............................................
    # ..............................................apply distance transform
    def apply_distance_transform(self):
        global world_obstacles
        if use_potential_function and np.max(self.world_obstacles) == 255:
            # Compute distance transform.
            dist_transform = 255 - np.minimum(
                64 * scipy.ndimage.morphology.distance_transform_edt(
                    255 - self.world_obstacles), 255)
            m = max(np.max(dist_transform), 1)  # Prevent m==0.
            self.world_obstacles = np.uint8((dist_transform * 255) / m)
        else:
            # Keep 255 values only (set all other to 0).
            self.world_obstacles = (self.world_obstacles == 255) * np.uint8(255)

    #..............................................
    def update_callback(self,start,goal,x_base,y_base):
        # First apply distance transform to world_obstacles.
        self.apply_distance_transform()
        # Call path planning algorithm.
        self.place_start(start)
        self.place_goal(goal)
        if not (start == None or goal == None):
            global optimal_path
            global visited_nodes

            try:
                optimal_path, visited_nodes = astar(start, goal, self.world_obstacles)
                print optimal_path
                #print visited_nodes
            except Exception, e:
                print traceback.print_exc()
        # Draw new background.
        self.draw_background(self.world_obstacles, visited_nodes, optimal_path,
                               show_visited_nodes)
        return optimal_path
    #..............................................
    # ..............................................
    def place_start(self, pos):
        self.place_start_goal(0, pos)

    # ..............................................
    def place_goal(self, pos):
        self.place_start_goal(1, pos)

    # ..............................................
    def place_start_goal(self, start_goal, pos):
        self.set_start_goal(start_goal, self.to_display(pos), 0.0)

    # ..............................................
    def set_start_goal(self, start_goal, pos, theta):
        x, y = pos
        if not (0 <= x < self.extents[0] and 0 <= y < self.extents[1]):
            return
        self.start_goal_coordinates[start_goal] = (x, y, theta)

        # Delete old ids used to draw start or goal.
        if self.start_goal_canvas_ids[start_goal]:
            for element in self.start_goal_canvas_ids[start_goal]:
                self.world_canvas.delete(element)
        self.start_goal_canvas_ids[start_goal] = []

        # Draw new start or goal.
        colors = ["yellow", "magenta"]
        radius = 6
        self.start_goal_canvas_ids[start_goal].append(
            self.world_canvas.create_oval(x - radius, y - radius,
                                          x + radius, y + radius,
                                          outline=colors[start_goal],
                                          width=2,
                                          tag="start"))
        if self.start_goal_mode == "oriented":
            self.start_goal_canvas_ids[start_goal].append(
                self.world_canvas.create_line(x, y,
                                              x + 3 * radius * cos(theta),
                                              y - 3 * radius * sin(theta),
                                              fill=colors[start_goal],
                                              width=2,
                                              tag="goal"))

        # Make sure drawing order is correct.
        self.set_display_order()
    #................................................
    def drag_goal(self,*args):
        global  cod
        self.drag_start_goal(1, cod)
    #................................................
    def drag_start_goal(self, start_goal, pos):
        x, y = self.to_display(pos)
        old_x, old_y = self.start_goal_coordinates[start_goal][0:2]
        dx = x - old_x
        dy = y - old_y
        theta = atan2(-dy, dx)
        self.set_start_goal(start_goal, (old_x, old_y), theta)
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


    #..............................................
# Main program.
#..................................................
if __name__ == '__main__':
    # Link functions.
    # Init GUI.
    gui = GUI( 4,
                  "on", "Path planner")
    # Start GUI main loop.
    gui.run()
#..................................................