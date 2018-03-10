import Tkinter as tk
import ttk
import matplotlib
import Pyro4
import time
import pandas as pd
import urllib
import json
import numpy as np
from math import sin, cos, pi, ceil
import matplotlib.animation as animation

matplotlib.use("TkAgg")

from matplotlib import style
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg,NavigationToolbar2TkAgg
from matplotlib.figure import Figure
#................................................
LARGE_FONT = ("Verdana",12)# font type
style.use("ggplot")        #matplotlib plot type
#...............................................

# The canvas and world extents of the scene.
# Canvas extents in pixels, world extents in millimeters.
canvas_extents = (600, 600)
world_extents = (2000.0, 2000.0)

# The extents of the sensor canvas.
sensor_canvas_extents = canvas_extents

# The maximum scanner range used to scale scan measurement drawings,
# in millimeters.
max_scanner_range = 2200.0
#............................................................................
class DrawableObject(object):
    def draw(self, at_step):
        print "To be overwritten - will draw a certain point in time:", at_step

    def background_draw(self):
        print "Background draw."

    @staticmethod
    def get_ellipse_points(center, main_axis_angle, radius1, radius2,
                           start_angle = 0.0, end_angle = 2 * pi):
        """Generate points of an ellipse, for drawing (y axis down)."""
        points = []
        ax = radius1 * cos(main_axis_angle)
        ay = radius1 * sin(main_axis_angle)
        bx = - radius2 * sin(main_axis_angle)
        by = radius2 * cos(main_axis_angle)
        N_full = 40  # Number of points on full ellipse.
        N = int(ceil((end_angle - start_angle) / (2 * pi) * N_full))
        N = max(N, 1)
        increment = (end_angle - start_angle) / N
        for i in xrange(N + 1):
            a = start_angle + i * increment
            c = cos(a)
            s = sin(a)
            x = c*ax + s*bx + center[0]
            y = - c*ay - s*by + center[1]
            points.append((x,y))
        return points
#...................................................................................

class Trajectory(DrawableObject):
    def __init__(self, points, canvas,
                 world_extents, canvas_extents,
                 standard_deviations = [],
                 point_size2 = 2,
                 background_color = "gray", cursor_color = "red",
                 position_stddev_color = "green", theta_stddev_color = "#ffc0c0"):
        self.points = points
        self.standard_deviations = standard_deviations
        self.canvas = canvas
        self.world_extents = world_extents
        self.canvas_extents = canvas_extents
        self.point_size2 = point_size2
        self.background_color = background_color
        self.cursor_color = cursor_color
        self.position_stddev_color = position_stddev_color
        self.theta_stddev_color = theta_stddev_color
        self.cursor_object = None
        self.cursor_object2 = None
        self.cursor_object3 = None
        self.cursor_object4 = None

    def background_draw(self):
        if self.points:
            p_xy_only = []
            for p in self.points:
                self.canvas.create_oval(\
                    p[0]-self.point_size2, p[1]-self.point_size2,
                    p[0]+self.point_size2, p[1]+self.point_size2,
                    fill=self.background_color, outline="")
                p_xy_only.append(p[0:2])
            self.canvas.create_line(*p_xy_only, fill=self.background_color)

    def draw(self, at_step):
        if self.cursor_object:
            self.canvas.delete(self.cursor_object)
            self.cursor_object = None
            self.canvas.delete(self.cursor_object2)
            self.cursor_object2 = None
        if at_step < len(self.points):
            p = self.points[at_step]
            # Draw position (point).
            self.cursor_object = self.canvas.create_oval(\
                p[0]-self.point_size2-1, p[1]-self.point_size2-1,
                p[0]+self.point_size2+1, p[1]+self.point_size2+1,
                fill=self.cursor_color, outline="")
            # Draw error ellipse.
            if at_step < len(self.standard_deviations):
                stddev = self.standard_deviations[at_step]
                # Note this assumes correct aspect ratio.
                factor = canvas_extents[0] / world_extents[0]
                points = self.get_ellipse_points(p, stddev[0],
                    stddev[1] * factor, stddev[2] * factor)
                if self.cursor_object4:
                    self.canvas.delete(self.cursor_object4)
                self.cursor_object4 = self.canvas.create_line(
                    *points, fill=self.position_stddev_color)
            if len(p) > 2:
                # Draw heading standard deviation.
                if at_step < len(self.standard_deviations) and\
                   len(self.standard_deviations[0]) > 3:
                    angle = min(self.standard_deviations[at_step][3], pi)
                    points = self.get_ellipse_points(p, p[2], 30.0, 30.0,
                                                     -angle, angle)
                    points = [p[0:2]] + points + [p[0:2]]
                    if self.cursor_object3:
                        self.canvas.delete(self.cursor_object3)
                    self.cursor_object3 = self.canvas.create_polygon(
                        *points, fill=self.theta_stddev_color)
                # Draw heading.
                self.cursor_object2 = self.canvas.create_line(p[0], p[1],
                    p[0] + cos(p[2]) * 50,
                    p[1] - sin(p[2]) * 50,
                    fill = self.cursor_color)
#.............................................................................

class ScannerData(DrawableObject):
    def __init__(self, list_of_scans, canvas, canvas_extents, scanner_range):
        self.canvas = canvas
        self.canvas_extents = canvas_extents
        self.cursor_object = None

        # Convert polar scanner measurements into xy form, in canvas coords.
        # Store the result in self.scan_polygons.
        self.scan_polygons = []
        for s in list_of_scans:
            poly = [ to_sensor_canvas((0,0), canvas_extents, scanner_range) ]
            i = 0
            for m in s:
                angle = beam_index_to_angle(i)
                x = m * cos(angle)
                y = m * sin(angle)
                poly.append(to_sensor_canvas((x,y), canvas_extents, scanner_range))
                i += 1
            poly.append(to_sensor_canvas((0,0), canvas_extents, scanner_range))
            self.scan_polygons.append(poly)

    def background_draw(self):
        # Draw x axis.
        self.canvas.create_line(
            self.canvas_extents[0]/2, self.canvas_extents[1]/2,
            self.canvas_extents[0]/2, 20,
            fill="black")
        self.canvas.create_text(
            self.canvas_extents[0]/2 + 10, 20, text="x" )
        # Draw y axis.
        self.canvas.create_line(
            self.canvas_extents[0]/2, self.canvas_extents[1]/2,
            20, self.canvas_extents[1]/2,
            fill="black")
        self.canvas.create_text(
            20, self.canvas_extents[1]/2 - 10, text="y" )
        # Draw big disk in the scan center.
        self.canvas.create_oval(
            self.canvas_extents[0]/2-20, self.canvas_extents[1]/2-20,
            self.canvas_extents[0]/2+20, self.canvas_extents[1]/2+20,
            fill="gray", outline="")

    def draw(self, at_step):
        if self.cursor_object:
            self.canvas.delete(self.cursor_object)
            self.cursor_object = None
        if at_step < len(self.scan_polygons):
            self.cursor_object = self.canvas.create_polygon(self.scan_polygons[at_step], fill="blue")
#........................................................................................
class Landmarks(DrawableObject):
    # In contrast other classes, Landmarks stores the original world coords and
    # transforms them when drawing.
    def __init__(self, landmarks, canvas, canvas_extents, world_extents, color = "gray"):
        self.landmarks = landmarks
        self.canvas = canvas
        self.canvas_extents = canvas_extents
        self.world_extents = world_extents
        self.color = color

    def background_draw(self):
        for l in self.landmarks:
            if l[0] =='C':
                x, y = l[1:3]
                ll = to_world_canvas((x - l[3], y - l[3]), self.canvas_extents, self.world_extents)
                ur = to_world_canvas((x + l[3], y + l[3]), self.canvas_extents, self.world_extents)
                self.canvas.create_oval(ll[0], ll[1], ur[0], ur[1], fill=self.color)

    def draw(self, at_step):
        # Landmarks are background only.
        pass
#..................................................................................................
class Points(DrawableObject):
    # Points, optionally with error ellipses.
    def __init__(self, points, canvas, color = "red", radius = 5, ellipses = [], ellipse_factor = 1.0):
        self.points = points
        self.canvas = canvas
        self.color = color
        self.radius = radius
        self.ellipses = ellipses
        self.ellipse_factor = ellipse_factor
        self.cursor_objects = []

    def background_draw(self):
        pass

    def draw(self, at_step):
        if self.cursor_objects:
            map(self.canvas.delete, self.cursor_objects)
            self.cursor_objects = []
        if at_step < len(self.points):
            for i in xrange(len(self.points[at_step])):
                # Draw point.
                c = self.points[at_step][i]
                self.cursor_objects.append(self.canvas.create_oval(
                    c[0]-self.radius, c[1]-self.radius,
                    c[0]+self.radius, c[1]+self.radius,
                    fill=self.color))
                # Draw error ellipse if present.
                if at_step < len(self.ellipses) and i < len(self.ellipses[at_step]):
                    e = self.ellipses[at_step][i]
                    points = self.get_ellipse_points(c, e[0], e[1] * self.ellipse_factor,
                                                     e[2] * self.ellipse_factor)
                    self.cursor_objects.append(self.canvas.create_line(
                        *points, fill=self.color))
#...................................................................................
# Particles are like points but add a direction vector.
class Particles(DrawableObject):
    def __init__(self, particles, canvas, color = "red", radius = 1.0,
                 vector = 8.0):
        self.particles = particles
        self.canvas = canvas
        self.color = color
        self.radius = radius
        self.vector = vector
        self.cursor_objects = []

    def background_draw(self):
        pass

    def draw(self, at_step):
        if self.cursor_objects:
            map(self.canvas.delete, self.cursor_objects)
            self.cursor_objects = []
        if at_step < len(self.particles):
            for c in self.particles[at_step]:
                self.cursor_objects.append(self.canvas.create_oval(
                    c[0]-self.radius, c[1]-self.radius,
                    c[0]+self.radius, c[1]+self.radius,
                    fill=self.color, outline=self.color))
                self.cursor_objects.append(self.canvas.create_line(
                    c[0], c[1],
                    c[0] + cos(c[2]) * self.vector,
                    c[1] - sin(c[2]) * self.vector,
                    fill = self.color))
#.......................................................................................

#...........................................................................Tkinter face
class Display (tk.Tk):

#............................................................
    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)

        tk.Tk.wm_title(self,"Sea of BTC client") # set title

        container = tk.Frame(self)   # creating container for Frame

        container.pack(side="top",fill="both", expand = True)
        container.grid_rowconfigure(0,weight=1)
        container.grid_columnconfigure(0,weight=1)


        self.frames = {}

        for F in (StartPage,Graps): # create the frames

            frame = F(container,self)
            self.frames[F] = frame

            frame.grid(row=0,column= 0, sticky="nsew")

        self.show_frame(StartPage) # begin the main page

#...........................................................genarate frame

    def show_frame(self,frame_name ):
        frame = self.frames[frame_name]
        frame.tkraise()
#.................................................................
class StartPage(tk.Frame):

    def __init__(self,parent,controler):
        tk.Frame.__init__(self,parent)
        label = ttk.Label(self,text="System", font=LARGE_FONT)
        label.pack(pady = 10 , padx=10)
        button2 = ttk.Button(self, text="Agree",
                             command=lambda: controler.show_frame(Graps))

        button2.pack()
#..................................................................
class Graps(tk.Frame):

    def __init__(self,parent,controller):
        tk.Frame.__init__(self,parent)
        label = ttk.Label(self, text="Grap Page ", font=LARGE_FONT)
        label.pack(pady=10, padx=10)
        button1 = ttk.Button(self, text="Go Home",
                            command=lambda: controller.show_frame(StartPage))
        button1.pack()




        canvas1 = FigureCanvasTkAgg(f1,self)
        canvas1.show()
        canvas1.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        canvas2 = FigureCanvasTkAgg(f2, self)
        canvas2.show()
        canvas2.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        toolbar = NavigationToolbar2TkAgg(canvas1,self)
        toolbar.update()
        canvas1._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)


f1 = Figure(figsize=(10, 5), dpi=100)
f2 = Figure(figsize=(10, 5), dpi=100)
a1 = f1.add_subplot(111)
a2 = f2.add_subplot(222)
#..........................................................
# World canvas is x right, y up, and scaling according to canvas/world extents.
def to_world_canvas(world_point, canvas_extents, world_extents):
    """Transforms a point from world coord system to world canvas coord system."""
    x = int(world_point[0] / world_extents[0] * canvas_extents[0])
    y = int(canvas_extents[1] - 1 - world_point[1] / world_extents[1] * canvas_extents[1])
    return (x, y)
#..........................................................
# Sensor canvas is "in driving direction", with x up, y left, (0,0) in the center
# and scaling according to canvas_extents and max_scanner_range.
def to_sensor_canvas(sensor_point, canvas_extents, scanner_range):
    """Transforms a point from sensor coordinates to sensor canvas coord system."""
    scale = canvas_extents[0] / 2.0 / scanner_range
    x = int(canvas_extents[0] / 2.0 - sensor_point[1] * scale)
    y = int(canvas_extents[1] / 2.0 - 1 - sensor_point[0] * scale)
    return (x, y)
#..........................................................

#............beam index angle
def beam_index_to_angle(i, mounting_angle=-0.06981317007977318):
    """Convert a beam index to an angle, in radians."""
    return (i - 330.0) * 0.006135923151543 + mounting_angle
#...........get connection
def get_connection():
    data_line = Pyro4.Proxy("PYRONAME:example.greeting")
    return data_line
#.......... get Data from the connection
def get_Data():
    data = " "
    if connection.is_avilable():
        data = connection.get_Data()
        connection.false_avilable()
    return data
#...........get_partcal_data
def get_particals(data):
    f = []
    f = data.split(' ')
    i = 1
    particle_list = []
    while i < len(f):
        particle_list.append(tuple(map(float, f[i:i + 3])))
        i += 3
    return particle_list
#..........get_filted_position
def get_position(data):
    f = []
    f = data.split(' ')
    return tuple(map(float, f[1:]))
#..........get_filtered_stddev
def get_filtered_stddev(data):
    f = []
    f = data.split(' ')
    return map(float, f[1:])
#..........get_world_cylinders
def get_world_cylinders(data):
    f = []
    f = data.split(' ')
    cyl = map(float, f[2:])
    return [(cyl[2 * i], cyl[2 * i + 1]) for i in range(len(cyl) / 2)]
#..........get_world_elipeces
def get_world_elipses(data):
    f = []
    f = data.split(' ')
    ell = map(float, f[2:])
    return [(ell[3 * i], ell[3 * i + 1], ell[3 * i + 2]) for i in xrange(len(ell) / 3)]
#.........draw Animation method
def draw(index):
    print(connection)
    data = get_Data()
    #data[0]..partical_ position
    partical = get_particals(data[0])
    particles.append(partical)

    #data[1]..filtered position
    postion = get_position(data[1])
    filtered_postion.append(postion)
    #data[2].filtered stddev
    stddev = tuple(get_filtered_stddev(data[2]))
    filtered_stddev.append(stddev)
    #data[3]..world cylinders
    cylinders = get_world_cylinders(data[3])
    world_cylinders.append(cylinders)
    #data[4]..world elipsces
    ellipses = get_world_elipses(data[4])
    world_ellipses.append(ellipses)
    #.............................................
    print (particles[-1])
    print(filtered_postion)
    print(filtered_stddev[-1])
    print(world_cylinders[-1])
    #.............................................
    #.............................................processing data as drawable item

    global draw_objects
    draw_objects = []


    # Insert: scanner data.-data need to be get
    #draw_objects.append(ScannerData(logfile.scan_data, sensor_canvas,
    #                               sensor_canvas_extents, max_scanner_range))


    # Insert: world objects, cylinders and corresponding world objects, ellipses.
    if cylinders:
        positions = [to_world_canvas(pos, canvas_extents, world_extents)
                      for pos in cylinders]

        # Also setup cylinders if present.
        # Note this assumes correct aspect ratio.
        factor = canvas_extents[0] / world_extents[0]
        draw_objects.append(Points(positions, world_canvas, "#DC23C5",
                                   ellipses=ellipses,
                                   ellipse_factor=factor))

    # Insert: particles.
    if partical:
        positions = [(to_world_canvas(pos, canvas_extents, world_extents) + (pos[2],))
             for pos in partical]

        draw_objects.append(Particles(positions, world_canvas, "#80E080"))

    # Insert: filtered trajectory.
    if filtered_postion:
        if len(filtered_postion[0]) > 2:
            positions = [tuple(list(to_world_canvas(pos, canvas_extents, world_extents)) + [pos[2]]) for pos in
                         filtered_postion]
        else:
            positions = [to_world_canvas(pos, canvas_extents, world_extents) for pos in filtered_postion]
        # If there is error ellipses, insert them as well.
        draw_objects.append(Trajectory(positions, world_canvas, world_extents, canvas_extents,
                                       standard_deviations=filtered_stddev,
                                       cursor_color="blue", background_color="lightblue",
                                       position_stddev_color="#8080ff", theta_stddev_color="#c0c0ff"))

    # Start new canvas and do all background drawing.
    i = int(index)
    world_canvas.delete(ALL)
    #sensor_canvas.delete(ALL)
    for d in draw_objects:
        d.background_draw()
        d.draw(1)
#...................................................
def animation1(i):
    #print(connection)
    pullData = open("sampleData.txt") .read()
    dataList = pullData.split('\n')
    xList = []
    yList = []

    for eachLine in dataList:
        x,y = eachLine.split(',')
        xList.append(int(x))
        yList.append(int(y))

    a1.clear()
    a2.clear()
    a1.plot(xList,yList)
    a2.plot(xList, yList)
#......................................................

#......................................................
if __name__ =='__main__':
    particles = []
    filtered_postion = []
    filtered_stddev = []
    world_cylinders = []
    world_ellipses = []
    #................
    draw_objects = []
    #................

    connection = get_connection()
    time.sleep(0.2)
    # Setup GUI stuff.
    app = Display()
    ani = animation.FuncAnimation(f1,animation1,interval = 1000)
    ani2 = animation.FuncAnimation(f1, draw, interval=500)
    app.mainloop()
#.........................................................

