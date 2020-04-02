#
# Created by Niraj Basnet on 3/18/2020.
#

import os
import imageio
import matplotlib.pyplot as plt


class Plotter:
    def __init__(self, environment, source, goal):
        self.env = environment
        self.xS, self.xG = source, goal
        self.obs_map = self.env.get_obsmap()
        self.gif_file_mode = False

    def set_gif_file_mode(self,gif_mode=True, filename="output.gif"):
        self.gif_file_mode = gif_mode

    def update_obs_map(self, obs_map):
        self.obs_map = obs_map

    def animate_path_and_visited(self,name, path, visited, ):
        plt.figure(figsize=(9, 7))
        self.plot_grid(name)
        self.plot_visited(visited)
        self.plot_path(path)
        plt.show()

    def plot_grid(self, name):
        obs_x = [x[0] for x in self.obs_map]
        obs_y = [x[1] for x in self.obs_map]
        plt.plot(self.xS[0], self.xS[1], "rs")
        plt.plot(self.xG[0], self.xG[1], color='orange',marker= 'p')
        plt.plot(obs_x, obs_y, "sk")
        plt.title(name)
        plt.axis("equal")

    def plot_visited(self, visited, cl='gray'):
        if self.xS in visited:
            visited.remove(self.xS)
        if self.xG in visited:
            visited.remove(self.xG)

        count = 1
        filenames=[]
        pause_interval = 25

        print("Press Enter to continue...")
        key=False
        while not key:key=plt.waitforbuttonpress()   ##Useful for recording plots
        print("Press Esc to quit.")

        for x in visited:
            plt.plot(x[0], x[1], color='lightgreen', marker='H')
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])
            if self.gif_file_mode:
                filename = f'{count}.png'
                filenames.append(filename)
                # save frame
                plt.savefig(filename)
            else:
                if count % pause_interval == 0:
                    plt.pause(0.0005)
                    pause_interval += 20

            count += 1

        if self.gif_file_mode:
            self.write_to_gif_file(filenames)

    def write_to_gif_file(self,filenames):
        # build gif
        with imageio.get_writer('Astar2D.gif', mode='I') as writer:
            for filename in filenames:
                image = imageio.imread(filename)
                writer.append_data(image)

        # Remove files
        for filename in set(filenames):
            os.remove(filename)

    def plot_path(self, path):
        path_x = [path[i][0] for i in range(len(path))]
        path_y = [path[i][1] for i in range(len(path))]
        plt.plot(path_x, path_y, linewidth='2', color='b')
        plt.pause(0.01)

