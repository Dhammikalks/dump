import Pyro4
from multiprocessing import Pipe , Process
import time
@Pyro4.expose
class Control(object):

    global Goal
    Goal = (10,10)
    global visitedset
    visitedset = [0,0]
    global obstacle
    obstacle = [0,0]
    isvisitednew = False


    potential = False
    @staticmethod
    def set_Goal(data):
        global Goal
        Goal = data

    @staticmethod
    def get_Goal():
        global Goal
        return Goal

    @staticmethod
    def set_Potential(data):
        global potential
        potential = data

    @staticmethod
    def is_Potential():
        global potential
        return potential

    @staticmethod
    def setVisitedSet(data):
        global visitedset
        visitedset = data
        print data

    @staticmethod
    def getVisistedSet():
        global visitedset
        return visitedset

    @staticmethod
    def setObstacleSet(data):
        global obstacle
        obstacle = data
        print data

    @staticmethod
    def getObstacleSet():
        global obstacle
        return obstacle

    @staticmethod
    def isVisitedNew():
        global isvisitednew
        return isvisitednew

    @staticmethod
    def setVisisted(data):
        global isvisitednew
        isvisitednew = data

# ..............................................................................
def comuincation(conn):
    conn.send("bringing dispaly server up ............")  # give the message connection is up
    daemon = Pyro4.Daemon()  # make a Pyro daemon
    ns = Pyro4.locateNS()  # find the name server
    uri = daemon.register(Control)  # register the greeting maker as a Pyro object
    ns.register("example.control", uri)  # register the object with a name in the name server
    print("Ready.")
    daemon.requestLoop()  # start the event loop of the server to wait for calls
    # ..............................................................................

if __name__ == '__main__':

    parent_conn, child_conn = Pipe()
    p = Process(target=comuincation, args=(child_conn,))
    p.start()
    ##extra_pararell_jobs
    p.join()
