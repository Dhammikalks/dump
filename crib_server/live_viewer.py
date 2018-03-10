#............................live_viewer_for_criber_mapping_server 1.0
#.... Auother : Dhammika Samarasinghe
#.....email   : dhammika.lks@gmail.com
#.....Date    : 2017.06.14
#......................................................................

from Tkinter import *
from math import sin , cos ,pi,ceil


# The canvas and world extents of the scene.
# Canvas extents in pixels, world extents in millimeters.
canvas_extents = (800, 800)
world_extents = (2000.0, 2000.0)

# The extents of the sensor canvas.
sensor_canvas_extents = canvas_extents

# The maximum scanner range used to scale scan measurement drawings,
# in millimeters.
max_scanner_range = 2200.0


class DrawableObject:

    @staticmethod
    def get_ellipse_points(center, main_axis_angle, radius1, radius2,
                           start_angle = 0.0, end_angle = 2 * pi):
        """Generate points of an ellipse, for drawing (y axis down)."""
        """this is retuning elliptical point according to the prvided codinate"""

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



