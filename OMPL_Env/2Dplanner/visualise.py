#!/usr/bin/env python

import matplotlib
import argparse
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import os 
import sys
from matplotlib import patches
import math

def get_obstacles_from_file(filepath):
### Function to read the obstacles from the file and return the list of obstacles
### Each obstacle is represented as a list of 4 elements: x, y, width, height
### Input: filepath - path to the file containing the obstacles
### Output: List of obstacles
    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            lines = f.readlines()
            obstacles = []
            
            for line in lines:
                obstacle = line.split()
                obstacles.append([float(obstacle[i]) for i in range(len(obstacle))])
        obstacles = list(filter(lambda x: x, obstacles))
        return obstacles
    
    else:
        print("Obstacle file not found - Please provide the right path")
        sys.exit(0)

def get_path_from_file(filepath):
### Function to read the path from the file and return the path
### The path is represented as a list of poses
### The first line of the file contains the configuration space and the size of the robot (if not a point robot)
### Input: filepath - path to the file containing the path
### Output: Configuration space, robot size (if not a point robot), List of poses
    robot_size = []
    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            lines = f.readlines()
            path = []
            robot_size = 0.3

            for i in range(len(lines)):
                cspace_coords = lines[i].split()
                path.append([float(cspace_coords[i]) for i in range(len(cspace_coords))])

        path = list(filter(lambda x: x, path))
        return robot_size, path
    else:
        print(f"{filepath} file not found - Please provide the right path")
        sys.exit(0)

def get_corners(pose, robot_size):
### Function to get the corners of the robot given the pose and the size of the square robot
### Input: pose - (x, y, theta) of the robot, robot_size - size of the robot
### Output: List of corners of the robot for given pose in SE2
    x, y, theta,_,_,_ = pose
    corners = [(x + robot_size/2 * math.cos(theta) - robot_size/2 * math.sin(theta), y + robot_size/2 * math.sin(theta) + robot_size/2 * math.cos(theta)), \
                (x - robot_size/2 * math.cos(theta) - robot_size/2 * math.sin(theta), y - robot_size/2 * math.sin(theta) + robot_size/2 * math.cos(theta)), \
                (x - robot_size/2 * math.cos(theta) + robot_size/2 * math.sin(theta), y - robot_size/2 * math.sin(theta) - robot_size/2 * math.cos(theta)), \
                (x + robot_size/2 * math.cos(theta) + robot_size/2 * math.sin(theta), y + robot_size/2 * math.sin(theta) - robot_size/2 * math.cos(theta))]
    return corners

def set_plot_properties(ax, max_x, max_y):
### Function to set the properties of the plot
### Input: ax - axis object, max_x - maximum x coordinate, max_y - maximum y coordinate
    ax.grid(True)
    #adjustable, box
    ax.set_aspect('equal', 'box')
    ax.set_facecolor('#f0f0f0')
    ax.set_xlim(0, max_x + 1)
    ax.set_ylim(0, max_y + 1)
    ax.set_title('Path Visualization')
    ax.set_xlabel('X position')
    ax.set_ylabel('Y position')
    # ax.set_xlabel('Theta')
    # ax.set_ylabel('Omega')
    ax.legend(loc='upper right')

def plot_environment_and_path(obstacles, path, robot_size):

    _, ax = plt.subplots()
    for obstacle in obstacles:
        ax.add_patch(patches.Rectangle((obstacle[0], obstacle[1]), obstacle[2], obstacle[3], fill=True, color='black'))
    ax.add_patch(patches.Rectangle((0, 0), 0, 0, fill=True, color='black', label='Obstacle'))

    for pose in path:
        corners = get_corners(pose, robot_size)
        polygon = patches.Polygon(corners, fill=False, color='green')
        ax.add_patch(polygon)
    
    max_x = max([pose[0] for pose in path])
    max_y = max([pose[1] for pose in path])

    max_x = max(max_x, max([obstacle[0] + obstacle[2] for obstacle in obstacles]))
    max_y = max(max_y, max([obstacle[1] + obstacle[3] for obstacle in obstacles]))

    set_plot_properties(ax, max_x, max_y)
    plt.savefig('drone.png', dpi=300)
    plt.close()

def animate_environment_and_path(obstacles, path, robot_size):

    fig, ax = plt.subplots()

    for obstacle in obstacles:
        ax.add_patch(patches.Rectangle((obstacle[0], obstacle[1]), obstacle[2], obstacle[3], fill=True, color='black'))
    ax.add_patch(patches.Rectangle((0, 0), 0, 0, fill=True, color='black', label='Obstacle'))

    corners = get_corners(path[0], robot_size)
    polygon = patches.Polygon(corners, fill=True, color='green', label='Robot')
    ax.add_patch(polygon)

    max_x = max([pose[0] for pose in path])
    max_y = max([pose[1] for pose in path])

    max_x = max(max_x, max([obstacle[0] + obstacle[2] for obstacle in obstacles]))
    max_y = max(max_y, max([obstacle[1] + obstacle[3] for obstacle in obstacles]))

    set_plot_properties(ax, max_x, max_y)

    def update(frame):
        pose = path[frame]
        corners = get_corners(pose, robot_size)
        polygon.set_xy(corners)
        return polygon,

    ani = FuncAnimation(fig, update, frames=len(path), blit=True, repeat=False)
    ani.save('drone.gif', writer='imagemagick', fps=15)
    plt.close()

    
def main():
    
    parser = argparse.ArgumentParser(description='Visualize the path of a robot in an environment with obstacles.')
    parser.add_argument('--obstacles', type=str, default='example_obstacles.txt', help='Name of the obstacles file')
    parser.add_argument('--path', type=str, default='example_path.txt', help='Name of the path file') 
    args = parser.parse_args()

    print("***" * 19 + "\n Visualising the environment and path for generated data\n" + "***" * 19)
    print("Instructions: \n1. Please ensure that the obstacles text file should contain the obstacle data in the format: x y width height")
    print("2. The path text file should specify the configuration space and the size of the robot (if not a point robot) on the first line and the poses in subsequent lines \n")

    obs_path = os.path.join(os.getcwd(), args.obstacles)
    path_path = os.path.join(os.getcwd(), args.path )
    obstacles = get_obstacles_from_file(obs_path)
    robot_size, path = get_path_from_file(path_path)

    print("Path and Obstacle data loaded successfully")

    plot_environment_and_path(obstacles, path, robot_size)
    animate_environment_and_path(obstacles, path, robot_size)

if __name__ == "__main__":
    main()
