import math
import random

class Boid:
    def __init__(self, x, y, vx, vy, biasval=0):
        """
        Initialize a Boid object.

        Args:
            x (float): Initial x-coordinate of the boid.
            y (float): Initial y-coordinate of the boid.
            vx (float): Initial velocity component in the x-direction.
            vy (float): Initial velocity component in the y-direction.
            biasval (float, optional): Initial bias value. Defaults to 0.
        """
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.biasval = biasval

    def update_position(self):
        """
        Update the position of the boid based on its velocity.
        """
        self.x += self.vx
        self.y += self.vy

    def update_velocity(self, centering_factor, matching_factor, avoidfactor, close_dx, close_dy, turnfactor, maxbias, bias_increment, minspeed, maxspeed, is_scout_group1, is_scout_group2):
        """
        Update the velocity of the boid based on its neighbors and environment.

        Args:
            centering_factor (float): Factor for centering behavior.
            matching_factor (float): Factor for matching velocity with neighbors.
            avoidfactor (float): Factor for avoiding collision with nearby boids.
            close_dx (float): Accumulated difference in x-coordinate to nearby boids.
            close_dy (float): Accumulated difference in y-coordinate to nearby boids.
            turnfactor (float): Factor for turning when near an edge.
            maxbias (float): Maximum bias value for scout group.
            bias_increment (float): Incremental value for bias update.
            minspeed (float): Minimum speed limit.
            maxspeed (float): Maximum speed limit.
            is_scout_group1 (function): Function to determine if the boid belongs to scout group 1.
            is_scout_group2 (function): Function to determine if the boid belongs to scout group 2.
        """
        # Add the centering/matching contributions to velocity
        self.vx += (self.xpos_avg - self.x) * centering_factor + (self.xvel_avg - self.vx) * matching_factor
        self.vy += (self.ypos_avg - self.y) * centering_factor + (self.yvel_avg - self.vy) * matching_factor

        # Add the avoidance contribution to velocity
        self.vx += close_dx * avoidfactor
        self.vy += close_dy * avoidfactor

        # If the boid is near an edge, make it turn
        if outside_top_margin(self):
            self.vy += turnfactor
        if outside_right_margin(self):
            self.vx -= turnfactor
        if outside_left_margin(self):
            self.vx += turnfactor
        if outside_bottom_margin(self):
            self.vy -= turnfactor

        # Dynamically update bias value
        if is_scout_group1(self):
            if self.vx > 0:
                self.biasval = min(maxbias, self.biasval + bias_increment)
            else:
                self.biasval = max(bias_increment, self.biasval - bias_increment)
        elif is_scout_group2(self):
            if self.vx < 0:
                self.biasval = min(maxbias, self.biasval + bias_increment)
            else:
                self.biasval = max(bias_increment, self.biasval - bias_increment)

        # If the boid has a bias, bias it
        if is_scout_group1(self):
            self.vx = (1 - self.biasval) * self.vx + self.biasval * 1
        elif is_scout_group2(self):
            self.vx = (1 - self.biasval) * self.vx + self.biasval * (-1)

        # Calculate the boid's speed
        speed = math.sqrt(self.vx * self.vx + self.vy * self.vy)

        # Enforce min and max speeds
        if speed < minspeed:
            self.vx = (self.vx / speed) * minspeed
            self.vy = (self.vy / speed) * minspeed
        elif speed > maxspeed:
            self.vx = (self.vx / speed) * maxspeed
            self.vy = (self.vy / speed) * maxspeed

    def set_neighboring_params(self, xpos_avg, ypos_avg, xvel_avg, yvel_avg, neighboring_boids):
        """
        Set parameters related to neighboring boids.

        Args:
            xpos_avg (float): Average x-coordinate of neighboring boids.
            ypos_avg (float): Average y-coordinate of neighboring boids.
            xvel_avg (float): Average velocity component in the x-direction of neighboring boids.
            yvel_avg (float): Average velocity component in the y-direction of neighboring boids.
            neighboring_boids (int): Number of neighboring boids.
        """
        self.xpos_avg = xpos_avg
        self.ypos_avg = ypos_avg
        self.xvel_avg = xvel_avg
        self.yvel_avg = yvel_avg
        self.neighboring_boids = neighboring_boids

# Assuming some functions for detecting edges
def outside_top_margin(boid):
    if boid.y >= 100:
        return True
    return False

def outside_right_margin(boid):
    if boid.x >= 100:
        return True
    return False

def outside_left_margin(boid):
    if boid.x <= 0:
        return True
    return False

def outside_bottom_margin(boid):
    if boid.y <= 0:
        return True
    return False

# Assuming some function to determine scout group
def is_scout_group1(boid):
    pass

def is_scout_group2(boid):
    pass

# Function to initialize a group of boids
def initialize_boids(num_boids):
    boids = []
    for _ in range(num_boids):
        boid = Boid(
            x=random.uniform(0, 100),  # Example range, adjust as needed
            y=random.uniform(0, 100),
            vx=random.uniform(-1, 1),
            vy=random.uniform(-1, 1)
        )
        boids.append(boid)
    return boids