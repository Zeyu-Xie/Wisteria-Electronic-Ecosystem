import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import Boid
import time
import os

class Map:
    def __init__(self, width=100, height=100, birds_number=100, fig_size_x=10, fig_size_y=10,
                 visual_range=20, protected_range_squared=15,
                 centering_factor=0.05, matching_factor=0.1,
                 avoidfactor=0.2, turnfactor=1, maxbias=0.1,
                 bias_increment=0.01, minspeed=0.1, maxspeed=1.0):
        self.width = width
        self.height = height
        self.birds_number = birds_number
        self.boids = Boid.initialize_boids(birds_number, width, height)
        self.counter = 0
        self.fig_size_x = fig_size_x
        self.fig_size_y = fig_size_y
        self.visual_range = visual_range
        self.protected_range_squared = protected_range_squared
        self.centering_factor = centering_factor
        self.matching_factor = matching_factor
        self.avoidfactor = avoidfactor
        self.turnfactor = turnfactor
        self.maxbias = maxbias
        self.bias_increment = bias_increment
        self.minspeed = minspeed
        self.maxspeed = maxspeed

    def update_boids(self, frame):
        self.update_boid()
        self.counter += 1

    def update_boid(self):
        self.update_boid_properties()
        self.plot_boids()

    def update_boid_properties(self):
        for boid in self.boids:
            self.update_boid_velocity(boid)
            self.update_boid_position(boid)

    def update_boid_velocity(self, boid):
        xpos_avg, ypos_avg, xvel_avg, yvel_avg, neighboring_boids, close_dx, close_dy = 0, 0, 0, 0, 0, 0, 0

        for otherboid in self.boids:
            dx = boid.x - otherboid.x
            dy = boid.y - otherboid.y

            if abs(dx) < self.visual_range and abs(dy) < self.visual_range:
                squared_distance = dx * dx + dy * dy

                if squared_distance < self.protected_range_squared:
                    close_dx += boid.x - otherboid.x
                    close_dy += boid.y - otherboid.y

                elif squared_distance < self.visual_range ** 2:
                    xpos_avg += otherboid.x
                    ypos_avg += otherboid.y
                    xvel_avg += otherboid.vx
                    yvel_avg += otherboid.vy
                    neighboring_boids += 1

        if neighboring_boids > 0:
            xpos_avg /= neighboring_boids
            ypos_avg /= neighboring_boids
            xvel_avg /= neighboring_boids
            yvel_avg /= neighboring_boids

            boid.vx += (xpos_avg - boid.x) * self.centering_factor + \
                (xvel_avg - boid.vx) * self.matching_factor
            boid.vy += (ypos_avg - boid.y) * self.centering_factor + \
                (yvel_avg - boid.vy) * self.matching_factor

        boid.vx += close_dx * self.avoidfactor
        boid.vy += close_dy * self.avoidfactor

        if Boid.outside_top_margin(boid):
            boid.vy -= self.turnfactor
        if Boid.outside_right_margin(boid):
            boid.vx -= self.turnfactor
        if Boid.outside_left_margin(boid):
            boid.vx += self.turnfactor
        if Boid.outside_bottom_margin(boid):
            boid.vy += self.turnfactor

        if Boid.is_scout_group1(boid):
            if boid.vx > 0:
                boid.biasval = min(
                    self.maxbias, boid.biasval + self.bias_increment)
            else:
                boid.biasval = max(self.bias_increment,
                                   boid.biasval - self.bias_increment)
        elif Boid.is_scout_group2(boid):
            if boid.vx < 0:
                boid.biasval = min(
                    self.maxbias, boid.biasval + self.bias_increment)
            else:
                boid.biasval = max(self.bias_increment,
                                   boid.biasval - self.bias_increment)

        if Boid.is_scout_group1(boid):
            boid.vx = (1 - boid.biasval) * boid.vx + boid.biasval * 1
        elif Boid.is_scout_group2(boid):
            boid.vx = (1 - boid.biasval) * boid.vx + boid.biasval * (-1)

        speed = math.sqrt(boid.vx * boid.vx + boid.vy * boid.vy)

        if speed < self.minspeed:
            boid.vx = (boid.vx / speed) * self.minspeed
            boid.vy = (boid.vy / speed) * self.minspeed
        elif speed > self.maxspeed:
            boid.vx = (boid.vx / speed) * self.maxspeed
            boid.vy = (boid.vy / speed) * self.maxspeed

    def update_boid_position(self, boid):
        boid.x += boid.vx
        boid.y += boid.vy

    def print_boids(self):

        os.system('clear')

        print("Frame {}".format(self.counter))

        tmp_ground = [[0 for i in range(self.width+2)] for j in range(self.height+2)]
        for i in range(self.birds_number):
            x = math.floor(self.boids[i].x+1)
            y = math.floor(self.boids[i].y+1)
            
            print(f"index: {i:<7} x: {x:<4} y: {y:<4}", end="")

            if x < 0 or x >= self.width+2 or y < 0 or y >= self.height+2:
                print("Out of range")
                continue
            
            print("")

            tmp_ground[x][y] = 1
        
        print("")

        print("Frame {}".format(self.counter))
        print("|", end="")
        for i in range(self.width+2):
            print("--", end="")
        print("|")

        for i in range(self.height+2):
            print("|", end="")
            for j in range(self.width+2):
                if tmp_ground[j][self.width+1-i] == 1:
                    print("o ", end="")
                else:
                    print("  ", end="")
            print("|")

        print("|", end="")
        for i in range(self.width+2):
            print("--", end="")
        print("|")

    def plot_boids(self):
        plt.clf()
        plt.xlim(-1, self.width + 1)
        plt.ylim(-1, self.height + 1)
        for boid in self.boids:
            plt.plot(boid.x, boid.y, 'o', color = "red")

        # Add dashed lines
        plt.axvline(x=(self.width+2)/3-1, linestyle='--', color='gray')
        plt.axvline(x=self.width + 1 - (self.width+2) /
                    3, linestyle='--', color='gray')
        plt.axhline(y=(self.height+2)/3-1, linestyle='--', color='gray')
        plt.axhline(y=self.height + 1 - (self.height+2) /
                    3, linestyle='--', color='gray')

        plt.title('Boids Simulation - Frame {}'.format(self.counter))
        # Add watermark
        plt.text(5, 5, '©️ 2024 Acan Xie', fontsize=self.width/10, color='gray')
    
    def auto_run(self, interval=12, type = "text"):

        if type == "text":
            while True:
                self.update_boid()
                self.counter += 1
                self.print_boids()
                time.sleep(1/interval)
        elif type == "figure":
            fig = plt.figure(figsize=(self.fig_size_x, self.fig_size_y))
            ani = FuncAnimation(plt.gcf(), self.update_boids, interval = interval)
            plt.show()
    
    def create_gif(self, frames, fps, fileName="output.gif"):
        fig = plt.figure(figsize=(self.fig_size_x, self.fig_size_y))
        ani = FuncAnimation(plt.gcf(), self.update_boids, frames = frames)
        ani.save(fileName, writer='imagemagick', fps=fps)