from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation 
import numpy as np
from matplotlib.patches import Circle, Polygon
import random
from matplotlib.widgets import Button, Slider
from boids_bandit import ContextualBandit


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
        for i in range(0, 20):
            vertices = np.array([[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])
            triangle = Polygon(vertices.tolist(), fc='black', zorder=3)
            self.boids.append({"shape": triangle,"pos": np.array([self.x_width//2 + i, self.y_width//2]), "vel": np.array([random.randint(1, 3)/3, random.randint(1, 3)/3])})

        self.dt = 0.1
        self.group_radius = 1
        self.cohesion_strength = 0.134
        self.seperation_strength = 0.045
        self.alignment_strength = 0.038
        self.max_vel = 1

        self.setup_widgets()

        self.bandit = ContextualBandit(self)

        self.bandit_K = 20
        self.bandit_window = []
        self.pending_action = None
        self.pending_context = None
        self.dist_before_window = None


    def setup_widgets(self):
        cohesion = plt.axes([0.08, 0.08, 0.2, 0.03]) # type: ignore
        self.cohesion_slider = Slider(cohesion, 'Cohesion', 0, 0.2, valinit=self.cohesion_strength, valfmt='%1.3f')
        self.cohesion_slider.on_changed(self.update_cohesion)

        seperation = plt.axes([0.41, 0.08, 0.2, 0.03]) # type: ignore
        self.seperation_slider = Slider(seperation, 'Separation', 0, 0.5, valinit=self.seperation_strength, valfmt='%1.3f')
        self.seperation_slider.on_changed(self.update_seperation)

        alignment = plt.axes([0.75, 0.08, 0.2, 0.03]) # type: ignore
        self.alignment_slider = Slider(alignment, 'Alignment', 0, 0.2, valinit=self.alignment_strength, valfmt='%1.3f')
        self.alignment_slider.on_changed(self.update_alignment)

    def update_cohesion(self, n):
        self.cohesion_strength = n

    def update_seperation(self, n):
        self.seperation_strength = n
    
    def update_alignment(self, n):
        self.alignment_strength = n

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
            self.boids[i]["shape"].set_facecolor('black') # muts be reset to stop boids staying red when leaving group
            self.adjacency_list[i] = [] 

        for i in range(0, len(self.boids)):
            for j in range(i+1, len(self.boids)):
                pos_i = self.boids[i]["pos"]
                pos_j = self.boids[j]["pos"]
                displacement = pos_i - pos_j
                mod_r = np.linalg.norm(displacement)

                if mod_r < self.group_radius:  # group radius limit can be adjusted
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
            
            search = [i] # search all neighbours of each group member
            while len(search) > 0:
                current_boid = search.pop(0)
                
                if current_boid not in grouped:
                    grouped.add(current_boid)
                    current_group.append(current_boid)
                    
                    for j in adj_list[current_boid]:
                        if j not in grouped:
                            search.append(j)           
        return groups

    def find_average(self, groups, target):
        group_positions = []
        
        for group in groups: 
            if len(group) <= 1: # filter out boids not in groups
                group_positions.append(None)
                continue

            sum_x, sum_y = 0.0, 0.0
            group_count = len(group)

            if len(group) > 1:
                for boid in group:
                    sum_x += self.boids[boid][target][0]
                    sum_y += self.boids[boid][target][1]
                    
                avg_x = sum_x / group_count
                avg_y = sum_y / group_count
                group_positions.append(np.array([avg_x, avg_y]))
        return group_positions
    
    def find_distances(self):
        distances = []
        boid_count = len(self.boids)
        for i in range(0, boid_count):
            for j in range(i + 1, boid_count):
                    displacement = self.boids[i]["pos"] - self.boids[j]["pos"]  
                    distances.append( np.linalg.norm(displacement))  # vector to each other boid
        if len(distances) == 0:
            return 0.0
        return sum(distances)/len(distances)


    def cohesion(self, groups, centers, boid):
        for i, group in enumerate(groups):
            if boid in group:
                if centers[i] is None:
                    return np.array([0.0, 0.0])
                
                difference = np.array(centers[i] - self.boids[boid]["pos"]) # get vector in direction of average point
                return (difference * self.cohesion_strength)
            
        return np.array([0.0, 0.0])
    
    def seperation(self, groups, boid):
        for group in groups:
            if boid in group and len(group) > 1:
                vec = np.array([0.0, 0.0])

                for boid_id in group:
                    if boid_id != boid:
                        displacement = self.boids[boid]["pos"] - self.boids[boid_id]["pos"]  
                        mod_r = np.linalg.norm(displacement)  # vector to each other boid

                        if 0 < mod_r < 1:
                            vec += displacement / mod_r  

                return vec * self.seperation_strength
        return np.array([0.0, 0.0])
    

    def allignment(self, groups, avg_vels, boid):
        
        for i, group in enumerate(groups):
            if boid in group:
                if avg_vels[i] is None:
                    return np.array([0.0, 0.0])
                
                difference = np.array(avg_vels[i] - self.boids[boid]["vel"]) # get vector of average vels
                return (difference * self.alignment_strength)
            
        return np.array([0.0, 0.0])
    
    def cap_speed(self, i):
        boid_vel = self.boids[i]["vel"]
        mag_vel = np.linalg.norm(boid_vel)
        if  mag_vel > self.max_vel:
            self.boids[i]["vel"] = (boid_vel/mag_vel) * self.max_vel

    def step(self):
        self.check_boundaries()
        groups = self.find_groups()
        group_avg_pos = self.find_average(groups, "pos")
        group_avg_vel = self.find_average(groups, "vel")
        for i in range(0, len(self.boids)):
            cohesion_force = self.cohesion(groups, group_avg_pos, i)
            seperation_force = self.seperation(groups, i)
            alignment_force = self.allignment(groups, group_avg_vel, i)

            self.boids[i]["vel"] += cohesion_force + seperation_force + alignment_force
            self.cap_speed(i)
            self.boids[i]["pos"] += self.boids[i]["vel"] * self.dt

        return group_avg_pos
    
    def get_triangle_vertices(self, pos, vel, size):
        angle = np.arctan2(vel[1], vel[0])
        
        vertices = np.array([[size, 0.0],[-size, -size / 1.5],[-size, size / 1.5]])
        
        cos, sin = np.cos(angle), np.sin(angle)
        rotation= np.array([[cos, -sin],[sin, cos]]) # rotation matrix
        
        new_vertices = np.dot(vertices, rotation.T) + pos # rotate the vertices + shift pos
        return new_vertices

    def animate(self, j):
        # change the values less often
        if j % self.bandit_K == 0:
            if self.pending_action is not None:
                dist_after = sum(self.bandit_window) / len(self.bandit_window)
                self.bandit.update(self.pending_context, self.pending_action, self.dist_before_window, dist_after)

            self.pending_context = self.bandit.get_context()
            self.dist_before_window = self.find_distances()
            self.pending_action = self.bandit.thomson_sample(self.pending_context)
            self.bandit.pull(self.pending_action)
            self.bandit_window = []

        group_averages = self.step()
        self.bandit_window.append(self.find_distances())

        for i in range(0, len(self.boids)):
            pos = self.boids[i]["pos"]
            vel = self.boids[i]["vel"]
            
            new_vertices = self.get_triangle_vertices(pos, vel, 0.15)
            self.boids[i]["shape"].set_xy(new_vertices)

        for circle in self.group_circles: # clear each frame or they are left behind
            circle.remove()
        self.group_circles.clear()
        
        for pos in group_averages:
            if pos is None: # ignore single boids (groups set to None)
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