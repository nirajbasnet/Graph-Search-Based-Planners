#
# Created by Niraj Basnet on 3/18/2020.
#

import math
import heapq
import env
from plot_tools import Plotter


class AStar:

    def __init__(self, environment, heuristic_type):
        self.env = environment
        self.s_start = 0
        self.s_goal = 0
        self.heuristic_type = heuristic_type

        self.motion_set = self.env.motions  # feasible input set(can be 4-connected or 8-connected motion)
        self.obs = self.env.obs  # obstacles map

        self.OPEN = []  # priority queue for maintaining OPEN set
        self.CLOSED = []  # CLOSED list for maintaining order of VISITED nodes
        self.PARENT = dict()  # parent map
        self.g = dict()  # cost to reach any node n from start node

    def weighted_Astar(self, s_start, s_goal, e):
        '''
        A* with inflated heuristic e.
        e=1 results in normal A*
        e=0 results in Dijkstra's algorithm
        e>1, the solution becomes sub-optimal by factor(upper bound) of e
        :return: path and visited order.
        '''
        self.s_start = s_start
        self.s_goal = s_goal

        g = {s_start: 0, s_goal: float("inf")}
        PARENT = {s_start: s_start}
        OPEN = []
        CLOSED = []
        heapq.heappush(OPEN,(g[s_start] + e * self.heuristic(s_start), s_start))

        while OPEN:
            _, s = heapq.heappop(OPEN)
            CLOSED.append(s)

            if s == s_goal:
                break

            for s_n in self.get_neighbor(s):
                motion_cost = math.hypot(s_n[0] - s[0], s_n[1] - s[1]) # Cost(s,s_n)
                new_cost_sn = g[s] + motion_cost
                if s_n not in g:
                    g[s_n] = math.inf
                if new_cost_sn < g[s_n]:  # updating Cost if it is better than existing one
                    g[s_n] = new_cost_sn
                    PARENT[s_n] = s
                    heapq.heappush(OPEN, (g[s_n] + e * self.heuristic(s_n), s_n))

        return self.trace_path(PARENT), CLOSED

    def get_neighbor(self, s):
        '''
        find neighbors of state s that not in obstacles.
        '''
        neighbors = []
        for motion in self.motion_set:
            neighbor = (s[0] + motion[0], s[1] + motion[1])
            if self.is_collision(s, neighbor):
                continue
            neighbors.append(neighbor)
        return neighbors

    def is_collision(self, s_start, s_end):
        '''
        check if the path (s_start, s_end) is in collision or not.
        Also considers collision during diagonal movement
        Returns true for collision
        '''

        if s_start in self.obs or s_end in self.obs:
            return True

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

            if s1 in self.obs or s2 in self.obs:
                return True

        return False

    def trace_path(self, PARENT):
        '''
        Trace the path from goal to start using PARENT set.
        '''

        path = [self.s_goal]
        s = self.s_goal
        while s != self.s_start:
            s = PARENT[s]
            path.append(s)
        return list(path)

    def heuristic(self, s):
        '''
        Calculates  heuristic of provided node s using goal position
        Euclidean and Manhattan heuristics are available
        '''

        if self.heuristic_type == "manhattan":
            return abs(self.s_goal[0] - s[0]) + abs(self.s_goal[1] - s[1])
        elif self.heuristic_type == "euclidean":
            return math.hypot(self.s_goal[0] - s[0], self.s_goal[1] - s[1])

if __name__ == '__main__':
    #Config params
    world = 1  # two worlds available: 1,2
    algorithm_id = 3     #3 algorithms : 1=Dijkstra, 2= A* , 3 = Weighted A*

    if world ==1:
        s_start = (10, 10)
        s_goal = (70, 50)
    elif world ==2:
        s_start = (5,5)
        s_goal = (35, 25)

    if algorithm_id==1:
        e=0   # Dijkstra
        label = "2D Dijkstra"
    elif algorithm_id==2:
        e=1   # A*
        label = "2D A*"
    elif algorithm_id==3:
        e=1.15 # Weighted A*
        label = "2D Weighted-A* with e="+str(e)

    environment = env.Env(world_id=1)

    astar = AStar(environment, "euclidean")
    plot = Plotter(environment, s_start, s_goal)

    path, visited_nodes = astar.weighted_Astar(s_start, s_goal, e)
    plot.animate_path_and_visited(label, path, visited_nodes)
