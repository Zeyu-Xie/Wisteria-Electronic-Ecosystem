import math
import imageio
import os
import Boid
import matplotlib.pyplot as plt

# Define constants
visual_range = 10
protected_range_squared = 3
visual_range_squared = visual_range ** 2
centering_factor = 0.1
matching_factor = 0.05
avoidfactor = 0.1
turnfactor = 0.1
maxbias = 0.1
bias_increment = 0.01
minspeed = 0.1
maxspeed = 1.0

# Example: Initialize a group of 100 boids
boids = Boid.initialize_boids(100)

def update_boid():
    global boids
    # For every boid . . .
    for boid in boids:

        # Zero all accumulator variables
        xpos_avg, ypos_avg, xvel_avg, yvel_avg, neighboring_boids, close_dx, close_dy = 0, 0, 0, 0, 0, 0, 0

        # For every other boid in the flock . . .
        for otherboid in boids:

            # Compute differences in x and y coordinates
            dx = boid.x - otherboid.x
            dy = boid.y - otherboid.y

            # Are both those differences less than the visual range?
            if abs(dx) < visual_range and abs(dy) < visual_range:

                # If so, calculate the squared distance
                squared_distance = dx * dx + dy * dy

                # Is squared distance less than the protected range?
                if squared_distance < protected_range_squared:

                    # If so, calculate difference in x/y-coordinates to nearfield boid
                    close_dx += boid.x - otherboid.x
                    close_dy += boid.y - otherboid.y

                # If not in protected range, is the boid in the visual range?
                elif squared_distance < visual_range_squared:

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
            boid.vx += (xpos_avg - boid.x) * centering_factor + (xvel_avg - boid.vx) * matching_factor
            boid.vy += (ypos_avg - boid.y) * centering_factor + (yvel_avg - boid.vy) * matching_factor

        # Add the avoidance contribution to velocity
        boid.vx += close_dx * avoidfactor
        boid.vy += close_dy * avoidfactor

        # If the boid is near an edge, make it turn by turnfactor
        if Boid.outside_top_margin(boid):
            boid.vy += turnfactor
        if Boid.outside_right_margin(boid):
            boid.vx -= turnfactor
        if Boid.outside_left_margin(boid):
            boid.vx += turnfactor
        if Boid.outside_bottom_margin(boid):
            boid.vy -= turnfactor

        # Dynamically update bias value
        if Boid.is_scout_group1(boid):
            if boid.vx > 0:
                boid.biasval = min(maxbias, boid.biasval + bias_increment)
            else:
                boid.biasval = max(bias_increment, boid.biasval - bias_increment)
        elif Boid.is_scout_group2(boid):
            if boid.vx < 0:
                boid.biasval = min(maxbias, boid.biasval + bias_increment)
            else:
                boid.biasval = max(bias_increment, boid.biasval - bias_increment)

        # If the boid has a bias, bias it!
        if Boid.is_scout_group1(boid):
            boid.vx = (1 - boid.biasval) * boid.vx + boid.biasval * 1
        elif Boid.is_scout_group2(boid):
            boid.vx = (1 - boid.biasval) * boid.vx + boid.biasval * (-1)

        # Calculate the boid's speed
        speed = math.sqrt(boid.vx * boid.vx + boid.vy * boid.vy)

        # Enforce min and max speeds
        if speed < minspeed:
            boid.vx = (boid.vx / speed) * minspeed
            boid.vy = (boid.vy / speed) * minspeed
        elif speed > maxspeed:
            boid.vx = (boid.vx / speed) * maxspeed
            boid.vy = (boid.vy / speed) * maxspeed

        # Update boid's position
        boid.x += boid.vx
        boid.y += boid.vy

    # Plot the updated boids and save the plot
    plt.figure()
    plt.xlim(0, 100)
    plt.ylim(0, 100)
    for boid in boids:
        plt.plot(boid.x, boid.y, 'bo')
    plt.savefig(os.path.join(os.path.dirname(__file__), "img", "boids{}.png".format(update_boid.counter)))
    plt.close()
    update_boid.counter += 1

update_boid.counter = 0

if __name__ == "__main__":

    # Create img directory if it doesn't exist
    if not os.path.exists(os.path.join(os.path.dirname(__file__), "img")):
        os.makedirs(os.path.join(os.path.dirname(__file__), "img"))

    for i in range(1000):
        update_boid()

    # Create GIF
    images = []
    for i in range(1000):
        images.append(imageio.imread(os.path.join(os.path.dirname(__file__), "img", "boids{}.png".format(i))))
    
    imageio.mimsave(os.path.join(os.path.dirname(__file__), "boids.gif"), images)
    print("GIF generated!")
