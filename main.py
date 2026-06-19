from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation 
import numpy as np
from matplotlib.patches import Circle
import random

class Boids:
    def __init__(self):

        with plt.ioff():
            self.fig = plt.figure()
        self.fig.set_dpi(100)
        self.fig.set_size_inches(10, 10)

        with plt.ioff():
            self.ax = plt.axes([0.1, 0.25, 0.8, 0.70], xlim=(0, 10), ylim=(0, 10))         # type: ignore

        self.x_width, self.y_width = 10.0, 10.0
        
        self.group_circles = []
        self.boids = []
        for i in range(0, 10):
            self.boids.append({"shape": Circle((5, -5), 0.1, fc='black', zorder=3),"pos": np.array([self.x_width//2 + i, self.y_width//2]), "vel": np.array([random.randint(1, 3)/3, random.randint(1, 3)/3])})

        self.dt = 0.1
        self.cohesion_strength = 0.1
 

    def init(self):
        for i in range(0, len(self.boids)):
            self.ax.add_patch(self.boids[i]["shape"])
        return []

    def check_boundaries(self):
        for i in range(0, len(self.boids)):
            pos = self.boids[i]["pos"]
            pos_x, pos_y = pos[0], pos[1]
            if pos_x <= 0:
                self.boids[i]["pos"][0] = self.x_width
            if pos_x >= self.x_width:
                self.boids[i]["pos"][0] = 0
            if pos_y <= 0:
                self.boids[i]["pos"][1]  = self.y_width
            if pos_y >= self.y_width:
                self.boids[i]["pos"][1] = 0

    def find_groups(self):
        self.adjacency_list = {}
        for i in range(0, len(self.boids)):
            self.boids[i]["shape"].set_facecolor('black') #reset each frame
            self.adjacency_list[i] = [] #reset adjacency list

        for i in range(0, len(self.boids)):
            for j in range(i+1, len(self.boids)):
                pos_i = self.boids[i]["pos"]
                pos_j = self.boids[j]["pos"]
                displacement = pos_i - pos_j
                mod_r = np.linalg.norm(displacement)

                if mod_r < 1:
                    self.adjacency_list[i].append(j)
                    self.adjacency_list[j].append(i)
                    self.boids[i]["shape"].set_facecolor('red')
                    self.boids[j]["shape"].set_facecolor('red')
        groups = self.create_groups(self.adjacency_list)
        return groups        

    def create_groups(self, adj_list):
        groups = []
        i = 0
        grouped = set()
        for i in range(len(self.boids)):
            if i in grouped:
                continue
            current_group = []
            groups.append(current_group)
            
            search = [i]
            while len(search) > 0:
                current_boid = search.pop(0)
                
                if current_boid not in grouped:
                    grouped.add(current_boid)
                    current_group.append(current_boid)
                    
                    for j in adj_list[current_boid]:
                        if j not in grouped:
                            search.append(j)           
        return groups

    def find_average_pos(self, groups):
        group_positions = []
        
        for group in groups:
            if len(group) <= 1:
                group_positions.append(None)
                continue
            sum_x, sum_y = 0.0, 0.0
            group_count = len(group)
            if len(group) > 1:
                for boid in group:
                    sum_x += self.boids[boid]["pos"][0]
                    sum_y += self.boids[boid]["pos"][1]
                    
                avg_x = sum_x / group_count
                avg_y = sum_y / group_count
                group_positions.append(np.array([avg_x, avg_y]))

        return group_positions

    def cohesion(self, groups, centers, boid):

            
        for i, group in enumerate(groups):
            if boid in group:
                if centers[i] is None:
                    return np.array([0.0, 0.0])
                difference = np.array(centers[i] - self.boids[boid]["pos"])
                return (difference * self.cohesion_strength)
            
        return np.array([0.0, 0.0])

    def step(self):
        self.check_boundaries()
        groups = self.find_groups()
        group_averages = self.find_average_pos(groups)
        for i in range(0, len(self.boids)):
            cohesion_force = self.cohesion(groups, group_averages, i)
            self.boids[i]["vel"] += cohesion_force
            self.boids[i]["pos"] += self.boids[i]["vel"] * self.dt

        return group_averages
    


    def animate(self, j):
        group_averages = self.step()

        for i in range(0, len(self.boids)):
            self.boids[i]["shape"].center = self.boids[i]["pos"]

        for circle in self.group_circles:
            circle.remove()
        self.group_circles.clear()
        
        for pos in group_averages:
            if pos is None:
                continue
            circle = Circle((pos[0], pos[1]), 0.1, fc='blue', alpha=0.6, zorder=2)
            self.ax.add_patch(circle)
            self.group_circles.append(circle)
        

        return []
    
        

    def run(self):
        self.anim = FuncAnimation(self.fig, self.animate, init_func=self.init, frames=360, interval=20, blit=False)
        try:
            plt.show()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    sim = Boids()
    sim.run()