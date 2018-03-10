import MySQLdb
import numpy
import sys,time
class Action(object):
    #..............................
    def __init__(self):

        self.position = []
        self.next_codinate = []
        self.x_factor = 1;
        self.y_factor = 1;
        self.con = 0
        self.get_Data()
    #..............................
    def connection(self):
        con = MySQLdb.connect(host='localhost', user='root', passwd='DUKS1992', db='ROBOT')
        try:

            cur = con.cursor()
            con.commit()
            # data = cur.fetchall()e
        except MySQLdb.Error, e:
            if con:
                con.rollback()
        self.con = con
    #...............................
    def get_Data(self):
        self.connection()
        quary = "SELECT * FROM Path_planner ORDER BY No DESC LIMIT 1"
        result = self.SQL_CMD_getData(self.con, quary)
        data = Data()
        data.read(result)
        self.set_xfactor(data.x_base)
        self.set_yfactor(data.y_base)
        self.set_position(data.position)
        self.set_nextpose(data.next_pos)
    #...............................
    def calculate_move(self,pos,next):
        angle = numpy.arctan2((next[0]-pos[0]),(next[1]-pos[1]))*180/numpy.pi
        distance = numpy.sqrt((next[0]-pos[0])**2+(next[1]-pos[1])**2)
        return (angle,distance)
    #...............................

    def to_world(self):
        return 0
    #...............................
    def act(self):
        return 0
    #...............................
    def set_Data(self):
        return 0
    #...............................
    def set_xfactor(self,x):
        self.x_factor = x
    #...............................
    def set_yfactor(self,y):
        self.y_factor =y
    #...............................
    def set_position(self,position):
        self.position =position
    #...............................
    def set_nextpose(self,next):
        self.next_codinate = next
    #...............................
    def SQL_CMD(self, con, query):
        cur = con.cursor()
        cur.execute(query)
        con.commit()
    #..............................
    def SQL_CMD_getData(self, con, query):
        cur = con.cursor()
        cur.execute(query)
        result = cur.fetchall()
        return result
    #...............................

class Data(object):
    def __init__(self):
        self.position = []
        self.next_pos = []
        self.goal = []
        self.path = []
        self.No = 0
        self.x_base = 0
        self.y_base = 0

    def read(self,data):
        self.No = int(data[0][0])

        path = []
        for i in data[0][1].strip('([])').split('), ('):
            path.append(tuple(map(int,i.split(','))))

        self.path = path
        self.position = path[0]
        self.next_pos = path[1]
        self.goal = path[len(path)-1]

        self.x_base = (float,data[0][2])
        self.y_base = (float,data[0][3])


if __name__ == '__main__':
    while True:
        act = Action()
        print act.calculate_move(act.position,act.next_codinate)
        time.sleep(1)