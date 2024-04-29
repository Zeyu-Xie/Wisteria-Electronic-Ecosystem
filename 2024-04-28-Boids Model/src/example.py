from Map import Map, auto_run
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Define basic constants
fig_size_x = 10
fig_size_y = 10
width = 100
height = 100
birds_number = 100
time_steps = 1000

# Define constants of Boids model
visual_range = 20
protected_range_squared = 15
visual_range_squared = visual_range ** 2
centering_factor = 0.05
matching_factor = 0.1
avoidfactor = 0.2
turnfactor = 1
maxbias = 0.1
bias_increment = 0.01
minspeed = 0.1
maxspeed = 1.0

if __name__ == "__main__":
    
    auto_run(Map(fig_size_x=8, fig_size_y=8))