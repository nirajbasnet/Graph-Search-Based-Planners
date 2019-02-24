#!/usr/bin/env python3

import abc

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.ticker import MultipleLocator


class Maze(abc.ABC):
    """ Base Maze Class """

    def __init__(self, maze_array, start_index=None, goal_index=None):
        """
            maze_array - 2D numpy array with 1s representing free space
                        0s representing occupied space
        """
        self.maze_array = maze_array
        self.cols, self.rows = self.maze_array.shape
        self.start_index = start_index
        self.goal_index = goal_index

    def __repr__(self):
        if isinstance(self, Maze2D):
            output = "2D Maze\n"
        elif isinstance(self, Maze4D):
            output = "4D Maze\n"
        output += str(self.maze_array)
        return output

    @classmethod
    def from_pgm(cls, filename):
        """
            Initializes the Maze from a (8 bit) PGM file

            Usage: new_maze = Maze2D.from_pgm("maze.pgm")
        """
        with open(filename, 'r', encoding='latin1') as infile:
            # Get header information and move file pointer past header
            header = infile.readline()
            width, height, _ = [int(item) for item in header.split()[1:]]
            # Read the rest of the image into a numpy array and normalize
            image = np.fromfile(infile, dtype=np.uint8).reshape((height, width))/255

        return cls(image.T)

    def plot_maze(self):
        """ Visualizes the maze """
        self.plot_path([], "Maze")

    def plot_path(self, path, title_name=None):
        """
            Plots the provided path on the maze
        """
        fig = plt.figure(1)
        ax1 = fig.add_subplot(1,1,1)

        spacing = 1.0 # Spacing between grid lines
        minor_location = MultipleLocator(spacing)

        # Set minor tick locations.
        ax1.yaxis.set_minor_locator(minor_location)
        ax1.xaxis.set_minor_locator(minor_location)

        # Set grid to use minor tick locations. 
        ax1.grid(which='minor')

        colors = ['b', 'r']
        plt.imshow(self.maze_array.T, cmap=plt.get_cmap('bone'))
        if title_name is not None:
            fig.suptitle(title_name, fontSize=20)

        # cast path to numpy array so indexing is nicer
        path = np.array(path)
        for i in range(len(path)-1):
            cidx = i % 2
            plt.plot([path[i, 0], path[i+1, 0]], [path[i, 1], path[i+1, 1]],\
                     color=colors[cidx], linewidth=4)
        plt.show()

    def get_goal(self):
        """
            Returns the index of the goal
        """
        return self.goal_index

    def get_start(self):
        """
            Returns the index of the start state
        """
        return self.start_index

    def check_hit(self, start, deltas):
        """
            Returns True if there are any occupied states between:
            start[0] to start[0]+dx and start[1] to start[1]+dy
            Inputs:
            start = (x,y)
            deltas = (dx, dy)
        """
        x, y = start
        dx, dy = deltas

        # Went off the maze
        if (x < 0) or (y < 0) or (x >= self.cols) or (y >= self.rows):
            return True

        # Starting in an obstacle
        if self.maze_array[int(round(start[0])), int(round(start[1]))] == 0:
            return True

        if dx == 0.0 and dy == 0.0: # we don't actually move, so we are done
            return False

        # discretize movement into steps
        norm = max(abs(dx), abs(dy))
        dx = dx/norm
        dy = dy/norm

        # advance the robot one step at a time
        for i in range(int(norm)):
            x += dx
            y += dy
            # Went off the maze
            if (x < 0) or (y < 0) or (x >= self.cols) or (y >= self.rows):
                return True
            # Went into collision cell
            if self.maze_array[int(x),int(y)] == 0:
                return True
        return False

    def check_occupancy(self, state):
        """ 
            Returns True if there is an obstacle at state

        Args:
            state = (x,y)
        """
        return self.maze_array[int(state[0]), int(state[1])]==0


class Maze2D(Maze):
    """ Maze2D Class """

    def __init__(self, maze_array, start_state=None, goal_state=None):
        super().__init__(maze_array, start_state, goal_state)

        if start_state is None:
            start_state = (0,0)
        self.start_state = start_state
        self.start_index = self.index_from_state(self.start_state)

        if goal_state is None:
            goal_state = (self.cols-1, self.rows-1) 
        self.goal_state = goal_state
        self.goal_index = self.index_from_state(self.goal_state)

    def index_from_state(self, state):
        """
            Gets a unique index for the state 
        """
        return state[0]*self.rows + state[1]

    def state_from_index(self, state_id):
        """
            Returns the state at a given index
        """
        x = int(np.floor(state_id/self.rows))
        y = state_id % self.rows
        return (x,y)

    def get_neighbors(self, state_id):
        """
            Returns a List of indices corresponding to
            neighbors of a given state
        """
        state = self.state_from_index(state_id)
        deltas = [[0, -1], [0, 1], [-1, 0], [1, 0]]
        # deltas = [[0, -1], [0, 1], [-1, 0], [1, 0],[1,1],[1,-1],[-1,1],[-1,-1]]
        neighbors = []
        # Try each motion and add to neighbors if feasible
        for delta in deltas:
            if not self.check_hit(state, delta):
                new_state = (state[0]+delta[0], state[1]+delta[1])
                neighbors.append(self.index_from_state(new_state))
        return neighbors


class Maze4D(Maze):
    """ Maze4D Class """

    def __init__(self, maze_array, start_state=None, goal_state=None, max_vel=2):
        super().__init__(maze_array, start_state, goal_state)

        self.max_vel = max_vel

        if start_state is None:
            start_state = np.array((0, 0, 0, 0))
        self.start_state = start_state
        self.start_index = self.index_from_state(self.start_state)

        if goal_state is None:
            goal_state = np.array((self.cols-1, self.rows-1, 0, 0) )
        self.goal_state = goal_state
        self.goal_index = self.index_from_state(self.goal_state)

    def index_from_state(self, state):
        """
            Gets a unique index for the state 
        """
        velocities = self.max_vel + 1
        return state[3]*self.rows*self.cols*velocities + \
               state[2]*self.rows*self.cols + \
               state[0]*self.rows + \
               state[1]

    def state_from_index(self, state_id):
        """
            Returns the state at a given index
        """
        velocities = self.max_vel + 1
        idx = state_id
        dy = int(np.floor(idx/(self.rows*self.cols*velocities)))
        idx -= dy*self.rows*self.cols*velocities
        dx = int(np.floor(idx / (self.rows*self.cols)))
        idx -= dx*self.rows*self.cols
        x = int(np.floor(idx/self.rows))
        y = idx % self.rows
        return (x, y, dx, dy)

    def get_neighbors(self, state_id):
        """
            Returns a List of indices corresponding to
            neighbors of a given state
        """
        state = self.state_from_index(state_id)
        deltas = [[0, -1], [0, 1], [-1, 0], [1, 0], [0, 0]]
        neighbors = []
        # Try each motion and add to neighbors if feasible
        for delta in deltas:
            new_delta = (state[2] + delta[0], state[3] + delta[1])
            # enforce speed limits
            if (new_delta[0] > self.max_vel) or (new_delta[1] > self.max_vel) or \
               (new_delta[0] < 0) or (new_delta[1] < 0):
                continue
            # ensure collisions
            if not self.check_hit(state[0:2], new_delta):
                new_state = (state[0] + new_delta[0], state[1] + new_delta[1],
                             new_delta[0], new_delta[1])
                neighbors.append(self.index_from_state(new_state))
        return neighbors
