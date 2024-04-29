import math
import imageio
import os
import matplotlib.pyplot as plt
import Boid


class Map:
    def __init__(self, width = 100, height = 100, time_steps = 1000, birds_number=100, fig_size_x=10, fig_size_y=10,
                 visual_range=20, protected_range_squared=15,
                 centering_factor=0.05, matching_factor=0.1,
                 avoidfactor=0.2, turnfactor=1, maxbias=0.1,
                 bias_increment=0.01, minspeed=0.1, maxspeed=1.0):
        self.width = width
        self.height = height
        self.time_steps = time_steps
        self.boids = Boid.initialize_boids(birds_number)
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

        # Create img directory if it doesn't exist
        if not os.path.exists(os.path.join(os.path.dirname(__file__), "img")):
            os.makedirs(os.path.join(os.path.dirname(__file__), "img"))

    def update_boids(self):
        for i in range(self.time_steps):
            self._update_boid()

        # Create GIF
        images = []
        for i in range(self.time_steps):
            images.append(imageio.imread(os.path.join(
                os.path.dirname(__file__), "img", "boids{}.png".format(i))))

        imageio.mimsave(os.path.join(
            os.path.dirname(__file__), "boids.gif"), images)
        print("GIF generated!")

    def _update_boid(self):
        # For every boid . . .
        for boid in self.boids:

            # Zero all accumulator variables
            xpos_avg, ypos_avg, xvel_avg, yvel_avg, neighboring_boids, close_dx, close_dy = 0, 0, 0, 0, 0, 0, 0

            # For every other boid in the flock . . .
            for otherboid in self.boids:

                # Compute differences in x and y coordinates
                dx = boid.x - otherboid.x
                dy = boid.y - otherboid.y

                # Are both those differences less than the visual range?
                if abs(dx) < self.visual_range and abs(dy) < self.visual_range:

                    # If so, calculate the squared distance
                    squared_distance = dx * dx + dy * dy

                    # Is squared distance less than the protected range?
                    if squared_distance < self.protected_range_squared:

                        # If so, calculate difference in x/y-coordinates to nearfield boid
                        close_dx += boid.x - otherboid.x
                        close_dy += boid.y - otherboid.y

                    # If not in protected range, is the boid in the visual range?
                    elif squared_distance < self.visual_range ** 2:

                        # Add other boid's x/y-coord and x/y vel to accumulator variables
                        xpos_avg += otherboid.x
                        ypos_avg += otherboid.y
                        xvel_avg += otherboid.vx
                        yvel_avg += otherboid.vy

                        # Increment number of boids within visual range
                        neighboring_boids += 1

            # If there were any boids in the visual range . . .
            if neighboring_boids > 0:

                # Divide accumulator variables by number of boids in visual range
                xpos_avg /= neighboring_boids
                ypos_avg /= neighboring_boids
                xvel_avg /= neighboring_boids
                yvel_avg /= neighboring_boids

                # Add the centering/matching contributions to velocity
                boid.vx += (xpos_avg - boid.x) * self.centering_factor + \
                    (xvel_avg - boid.vx) * self.matching_factor
                boid.vy += (ypos_avg - boid.y) * self.centering_factor + \
                    (yvel_avg - boid.vy) * self.matching_factor

            # Add the avoidance contribution to velocity
            boid.vx += close_dx * self.avoidfactor
            boid.vy += close_dy * self.avoidfactor

            # If the boid is near an edge, make it turn by turnfactor
            if Boid.outside_top_margin(boid):
                boid.vy -= self.turnfactor
            if Boid.outside_right_margin(boid):
                boid.vx -= self.turnfactor
            if Boid.outside_left_margin(boid):
                boid.vx += self.turnfactor
            if Boid.outside_bottom_margin(boid):
                boid.vy += self.turnfactor

            # Dynamically update bias value
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

            # If the boid has a bias, bias it!
            if Boid.is_scout_group1(boid):
                boid.vx = (1 - boid.biasval) * boid.vx + boid.biasval * 1
            elif Boid.is_scout_group2(boid):
                boid.vx = (1 - boid.biasval) * boid.vx + boid.biasval * (-1)

            # Calculate the boid's speed
            speed = math.sqrt(boid.vx * boid.vx + boid.vy * boid.vy)

            # Enforce min and max speeds
            if speed < self.minspeed:
                boid.vx = (boid.vx / speed) * self.minspeed
                boid.vy = (boid.vy / speed) * self.minspeed
            elif speed > self.maxspeed:
                boid.vx = (boid.vx / speed) * self.maxspeed
                boid.vy = (boid.vy / speed) * self.maxspeed

            # Update boid's position
            boid.x += boid.vx
            boid.y += boid.vy

        # Plot the updated boids and save the plot
        plt.figure(figsize=(self.fig_size_x, self.fig_size_y))
        plt.xlim(-1, self.width + 1)
        plt.ylim(-1, self.height + 1)
        for boid in self.boids:
            plt.plot(boid.x, boid.y, 'bo')
        plt.savefig(os.path.join(os.path.dirname(__file__),
                    "img", "boids{}.png".format(self.counter)))
        plt.close()
        self.counter += 1
