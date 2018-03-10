class Data(object):

    def __init__(self):
        self.path = []
        self.poisition = (0,0)

    def fed_data(self,data):
        self.path = data

    def get_path(self):
        return self.path

    def get_position(self):
        return self.poisition

class Calc(object):

      def __init__(self):
          self.position = (0,0)
          self.next_pos = (0,0)

      def set_pos(self,pos):
          self.position = pos

      def set_nextpos(self,pos):
          self.next_pos = pos

      def next_direction(self):
          return 0.0
      def next_lenght(self):
          return 0.0

class Control:
    def run(self):
        return 0

if __name__ =='__main__':
    run()


