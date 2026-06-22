import numpy as np
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from boids import Boids

class ContextualBandit:
    def __init__(self, env: "Boids"):
        self.env = env
        self.contexts = ["scattered", "overcrowded", "flocking", "static"]
        
        self.alpha = {c: [1, 1, 1, 1] for c in self.contexts}
        self.beta = {c: [1, 1, 1, 1] for c in self.contexts}
        self.N = {c: [0, 0, 0, 0] for c in self.contexts}
        self.Q = {c: [0.0, 0.0, 0.0, 0.0] for c in self.contexts}

    def get_context(self):
        avg_dist = self.env.find_distances()
        avg_vel = np.mean([np.linalg.norm(b["vel"]) for b in self.env.boids])
        if avg_vel < 0.05:
            return "static"
        elif avg_dist >= 3.5:
            return "scattered"
        elif avg_dist <= 1.5:
            return "overcrowded"
        else:
            return "flocking"
            
    def get_reward(self, old_context, dist_before, dist_after):
        if old_context == "scattered":
            if dist_after < dist_before:
                return 1
        elif old_context == "overcrowded":
            if dist_after > dist_before:
                return 1
        elif old_context == "flocking":
            if 1.5 <= dist_after <= 3.5:
                return 1
        return 0

    def pull(self, action):
        if action == 0:
            self.env.cohesion_strength = min(0.2, self.env.cohesion_strength + 0.002)
            self.env.seperation_strength = max(0.0, self.env.seperation_strength - 0.002)
            self.env.cohesion_slider.set_val(self.env.cohesion_strength)
            self.env.seperation_slider.set_val(self.env.seperation_strength)
        elif action == 1:
            self.env.cohesion_strength = max(0.0, self.env.cohesion_strength - 0.002)
            self.env.seperation_strength = min(0.5, self.env.seperation_strength + 0.002)
            self.env.cohesion_slider.set_val(self.env.cohesion_strength)
            self.env.seperation_slider.set_val(self.env.seperation_strength)
        elif action == 2:
            self.env.alignment_strength = min(0.2, self.env.alignment_strength + 0.002)
            self.env.alignment_slider.set_val(self.env.alignment_strength)
        elif action == 3:
            self.env.alignment_strength = max(0.0, self.env.alignment_strength - 0.001)
            self.env.alignment_slider.set_val(self.env.alignment_strength)

    def thomson_sample(self, context):
        P = [0.0, 0.0, 0.0, 0.0]
        for j in range(0, 4):
            P[j] = np.random.beta(self.alpha[context][j], self.beta[context][j])
        return( np.argmax(P))
    
    def update(self, context, action, dist_before, dist_after):
        reward = self.get_reward(context, dist_before, dist_after)
        
        if reward == 1:
            self.alpha[context][action] += 1
        else:
            self.beta[context][action] += 1
            
        self.N[context][action] += 1
        
        current_Q = self.Q[context][action]
        self.Q[context][action] = current_Q + (reward - current_Q) / self.N[context][action]
        
