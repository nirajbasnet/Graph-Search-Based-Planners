#
# Created by Niraj Basnet on 3/18/2020.
#

import math
import heapq
import env
from plot_tools import Plotter
from Astar2D import AStar

class Dijkstra(AStar):
    def get_path(self,s_start,s_goal):
        """
        Breadth-first Searching.
        :return: path, visited order
        """

        self.s_start = s_start
        self.s_goal = s_goal

        g = {s_start: 0, s_goal: float("inf")}
        PARENT = {s_start: s_start}
        OPEN = []
        CLOSED = []
        heapq.heappush(OPEN,(g[s_start], s_start))

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
                    heapq.heappush(OPEN, (g[s_n],s_n))

        return self.trace_path(PARENT), CLOSED


def main():
    #Config params
    world = 1  # two worlds available: 1,2

    if world ==1:
        s_start = (10, 10)
        s_goal = (70, 50)
    elif world ==2:
        s_start = (5,5)
        s_goal = (35, 25)
    label = "2D Dijkstra"

    environment = env.Env(world_id=1)

    dijkstra = Dijkstra(environment,"euclidean")
    plot = Plotter(environment, s_start, s_goal)

    path, visited_nodes = dijkstra.get_path(s_start, s_goal)
    plot.animate_path_and_visited(label, path, visited_nodes)


if __name__ == '__main__':
    main()
