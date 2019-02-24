import numpy as np
from maze import Maze2D, Maze4D
from priority_queue import PriorityQueue
import time


def compute_heuristic(mode, start, goal):
    """compute heuristic given start and goal state"""
    if mode == "Manhattan":
        velocity = np.sqrt(start[2]*start[2] + start[3]*start[3])
        if velocity != 0:
            return (abs(goal[0] - start[0]) + abs(goal[1] - start[1]))/velocity
        else:
            return (abs(goal[0] - start[0]) + abs(goal[1] - start[1]))
    elif mode == "Euclidean":
        return np.sqrt(np.power(goal[0] - start[0], 2) + np.power(goal[1] - start[1], 2))


def compute_path(start, goal, parent):
    path = [goal]
    while goal != start:
        goal = parent[goal]
        path.append(goal)
    return path


def get_path_vectors(path):
    path_state = []
    for p in path:
        path_state.append(maze.state_from_index(p))
    return path_state


def Astar(maze, start, goal, heuristic, epsilon):
    """start and goal state index should be given
    """
    map_size = maze.rows * maze.cols * (maze.max_vel+1)*(maze.max_vel+1)
    closed_set = []
    open_set = PriorityQueue()
    open_set.insert(start, 0)
    parent = [x for x in range(map_size)]
    g_value = [float('inf') for i in range(map_size)]
    f_value = [float('inf') for i in range(map_size)]
    g_value[start] = 0
    f_value[start] = epsilon * compute_heuristic(heuristic, maze.state_from_index(start), maze.state_from_index(goal))
    parent[start] = start
    current_index = start
    current_state = maze.state_from_index(start)
    while len(open_set) != 0:
        current_index = open_set.pop()
        if current_index == goal:
            path_a = compute_path(start, goal, parent)
            print(epsilon, len(closed_set), len(path_a) - 1)
            return path_a
        # open_set._remove_item(current_index)
        closed_set.append(current_index)
        for neighbour in maze.get_neighbors(current_index):
            if neighbour in closed_set:
                continue
            neighbour_state=maze.state_from_index(neighbour)
            current_index_state = maze.state_from_index(current_index)
            cost_current_neighbour=abs(neighbour_state[0]-current_index_state[0])+abs(neighbour_state[1]-current_index_state[1])
            g_value_current = g_value[current_index] + 1
            if g_value[neighbour] >= g_value_current:
                parent[neighbour] = current_index
                g_value[neighbour] = g_value_current
                f_value[neighbour] = g_value[neighbour] + epsilon * compute_heuristic(heuristic,
                                                                                      maze.state_from_index(neighbour),
                                                                                      maze.state_from_index(goal))
                open_set.insert(neighbour, f_value[neighbour])


if __name__ == "__main__":
    maze = Maze4D.from_pgm('maze1.pgm')

    print(maze.maze_array.shape)
    print('Goal: {}'.format(maze.goal_state))
    print('Goal index: {}'.format(maze.goal_index))
    print('Test state_from_index: {}'.format(maze.state_from_index(maze.get_goal())))
    print('Test index_from_state: {}'.format(maze.index_from_state(maze.goal_state)))
    print('Neighbors of start position:')
    print([maze.state_from_index(pos) for pos in maze.get_neighbors(maze.start_index)])
    total_path = Astar(maze,maze.start_index,maze.goal_index,"Manhattan",1.0)
    # total_path = Astar(maze, maze.start_index, maze.goal_index, "Euclidean", 1.0)
    print("Shortest path cost=", len(total_path) - 1)
    maze.plot_path(get_path_vectors(total_path), 'Maze2D')

    past_time = time.time()
    epsilon = 10
    given_time = 1.0
    while True:
        if epsilon == 1 or (time.time() - past_time) >= given_time:
            break
        total_path = Astar(maze, maze.start_index, maze.goal_index, "Manhattan", epsilon)
        epsilon = epsilon - 0.5 * (epsilon - 1)
        if epsilon < 1.001:
            epsilon = 1
            total_path = Astar(maze, maze.start_index, maze.goal_index, "Manhattan", epsilon)
