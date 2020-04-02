#
# Created by Niraj Basnet on 3/18/2020.
#

class Env:
    def __init__(self,world_id=1):
        self.x_range = 80  # default size of background
        self.y_range = 60
        self.motions = None
        self.obs = set()
        self.load_obstaclemap(world_id)

    def set_obsmap(self, obs):
        self.obs = obs

    def get_obsmap(self):
        return self.obs

    def load_obstaclemap(self,world_id):
        if world_id == 1:
            self.x_range = 80  # size of background
            self.y_range = 60
            self.obstaclemap_world1()
        elif world_id == 2:
            self.x_range = 40  # size of background
            self.y_range = 30
            self.obstaclemap_world2()


    def obstaclemap_world1(self):
        #Allowed motions
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),(1, 0), (1, -1), (0, -1), (-1, -1)]

        # Initialize positions of obstacles in the environment
        self.add_rectangle((0, 0), (self.x_range, self.y_range))  # main boundary of environment
        self.add_line((20, 0), (20, 15))
        self.add_line((20, 40), (20, 60))
        self.add_line((0, 30), (30, 30))
        self.add_line((40, 30), (40, 60))
        self.add_line((50, 0), (50, 20))
        self.add_line((50, 30), (80, 30))
        self.add_line((50, 30), (50, 40))
        self.add_line((60, 50), (60, 60))

    def obstaclemap_world2(self):
        # Allowed motions
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1)]

        # Initialize positions of obstacles in the environment
        self.add_rectangle((0, 0), (self.x_range, self.y_range))  # main boundary of environment
        self.add_line((10, 0), (10, 8))
        self.add_line((10, 20), (10, 30))
        self.add_line((0, 15), (15, 15))
        self.add_line((20, 15), (20, 30))
        self.add_line((25, 0), (25, 10))
        self.add_line((25, 15), (40, 15))
        self.add_line((25, 15), (25, 20))
        self.add_line((30, 25), (30, 30))


    def add_line(self,start_pos,end_pos):
        '''draw a horiziontal/vertical line from start pos(x1,y1) to end pos(x2,y2)'''
        min_x = min(start_pos[0],end_pos[0])
        max_x = max(start_pos[0],end_pos[0])
        min_y = min(start_pos[1], end_pos[1])
        max_y = max(start_pos[1], end_pos[1])
        if min_x == max_x:
            for y in range(min_y,max_y+1):
                self.obs.add((min_x,y))
        else:
            for x in range(min_x,max_x+1):
                self.obs.add((x,min_y))

    def add_rectangle(self,lower_right_pos,uppper_left_pos):
        '''draw a rectangle with lower right diagonal point and upper left diagonal point'''
        self.add_line(lower_right_pos,(uppper_left_pos[0],lower_right_pos[1]))
        self.add_line(lower_right_pos, (lower_right_pos[0], uppper_left_pos[1]))
        self.add_line(uppper_left_pos, (lower_right_pos[0], uppper_left_pos[1]))
        self.add_line(uppper_left_pos, (uppper_left_pos[0], lower_right_pos[1]))

    def read_from_occgridmap_file(filename):
        '''update obstacle map by reading occupancy grid map'''

