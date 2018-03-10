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