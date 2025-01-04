import pygame
import math
import numpy as np


def distance(point1,point2):    

    # convert points into numpy array (to perform element-wise operations)
    point1 = np.array(point1)
    point2 = np.array(point2)
    # return the euclidean distance 
    return np.linalg.norm(point1- point2)




class Robot:
    def __init__(self, startpos, width, map_width, map_height):
        self.m2p = 3779.52  # Conversion factor from meters to pixels (for visualization)
        self.w = width * self.m2p

        # Starting position (x, y) and orientation (in radians)
        self.x = startpos[0]
        self.y = startpos[1]
        self.orientation = 0 # in radians

        # Wheel speeds (meters/sec to pixels/sec)
        self.vl = 0.01 * self.m2p
        self.vr = 0.01 * self.m2p

        # Speed limits
        self.maxspeed = 0.02 * self.m2p
        self.minspeed = 0.01 * self.m2p

        # Obstacle avoidance parameters
        self.min_obs_dist = 50  # Minimum distance from an obstacle (pixels)
        self.count_down = 5  # Time to control avoidance behavior (seconds)
        
         # Map dimensions
        self.map_width = map_width
        self.map_height = map_height
        
    def move_backward(self):
        self.vl = -self.minspeed   # Reverse right wheel
        self.vr = -self.minspeed/4 # Reverse left wheel slower to make a turn 

    def move_forward(self):
        self.vr = self.minspeed
        self.vl = self.minspeed

    def avoid_obstacle(self,point_cloud,t): #t: elapsed time since the last update (seconds).
        closest_obs = None
        dist = np.inf #initialize with infinity

        #find the closest obstacle in the point cloud
        if len(point_cloud) > 1:
            for point in point_cloud:
                if dist > distance([self.x, self.y], point):
                    dist = distance([self.x,self.y], point)
                    closest_obs = (point,dist)

    # if there's a closest obstacle and it's within the minimum distance, avoid it
        if closest_obs and closest_obs[1] < self.min_obs_dist and self.count_down > 0:
            self.count_down -= t
            print(f"Obstacle within range. avoiding...")  # Debug output
            self.move_backward() # Move backward to avoid obstacle
        else:
            self.count_down = 5 #reset count down
            print("No obstacles detected. Moving forward...")  # Debug output
            self.move_forward()#move forward


    def kinematics(self, dt):
        """Update the robot's position and orientation."""
        self.x += ((self.vl + self.vr) / 2) * math.cos(self.orientation) * dt
        self.y -= ((self.vl + self.vr) / 2) * math.sin(self.orientation) * dt
        self.orientation += (self.vr - self.vl) / self.w * dt

        # Normalize orientation
        if self.orientation > 2 * math.pi or self.orientation < -2 * math.pi:
            self.orientation = 0

        # Clamp speeds
        self.vr = max(min(self.maxspeed, self.vr), self.minspeed)
        self.vl = max(min(self.maxspeed, self.vl), self.minspeed)

class Graphics:
    def __init__(self, dimentions, robot_img_path, map_img_path):
        pygame.init() # Initialize Pygame

        #color definitions
        self.red = (255, 0, 0)
        

        # Load and scale images
        self.robot = pygame.image.load(robot_img_path)
        self.robot = pygame.transform.scale(self.robot, (50, 50))  # Scale to 50x50 pixels
        self.map_img = pygame.image.load(map_img_path)
        # self.map_img = pygame.image.load(map_img_path)
        # self.map_img = pygame.transform.scale(self.map_img, (0,0))  # Scale map to window dimensions


        #map dimentions
        self.height, self.width = dimentions

        #window settings
        pygame.display.set_caption("Obstacle Avoidance") #set window title
        self.map = pygame.display.set_mode((self.width, self.height)) #set window size
        self.map.blit(self.map_img, (0,0)) #draw map on window

    def draw_robot(self, x, y, orientation):

        #rotate and scale the robot image
        robo = pygame.transform.rotozoom(self.robot, math.degrees(orientation), 1) 
        #get the bounding rectangle of the image and set its center
        rect = robo.get_rect(center=(x, y))
        #draw the robot image on the map
        self.map.blit(robo, rect)

    def draw_sensor_data(self, point_cloud):
        #draw red circles for each point in the point cloud (detected obstacles)
        for point in point_cloud:
            pygame.draw.circle(self.map, self.red, point, 3, 0)  #radius of 3 pixels



class Ultrasonic:
    def __init__(self, sensor_range, map):

        self.sensor_range = sensor_range #sensor_range is a tuple: (max_distance, angle_range)
        self.map_width, self.map_height = pygame.display.get_surface().get_size() #get the width and height of the display surface for boundary checks
        self.map = map #store map surface reference for pixel color checks
        
        #color definitions
        self.black = (0, 0, 0)
        self.green =(0, 255, 0)
        
    def sense_obstacles(self, x, y, orientation):

        obstacles = [] #list to store detected obstacle positions
        
        x1, y1 = x, y #starting coordinates of the robot

        #calculate starting and finishing angles based on the robot's orientation

        start_angle = orientation - self.sensor_range[1]  #left bound of sensor range
        finish_angle = orientation + self.sensor_range[1]  #right bound of sensor range
        
        #iterate over the range of angles (10 evenly spaced points between start_angle and finish_angle)
        for angle in np.linspace(start_angle, finish_angle , 10, False):
            #calculate the end coordinates of the sensor line at each angle
            x2 = x1 + self.sensor_range[0] * math.cos(angle)
            y2 = y1 - self.sensor_range[0] * math.sin(angle)
            
            #iterate along the line from (x1, y1) to (x2, y2) to check for obstacles
            for i in range(0, 100):
            # Compute intermediate points between the start and end points
                u = i / 100
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))
                
                #ensure that the point is within the bounds of the map
                if 0 < x < self.map_width and 0 < y < self.map_height:
                    #get the color of the pixel at the current point
                    color = self.map.get_at((x, y))
                    #change the pixel color to green for visualization 
                    self.map.set_at((x, y), self.green)
                    
                    #check if the pixel color is black (indicating an obstacle)
                    if (color[0], color[1], color[2]) == self.black:
                        #add the obstacle position to the list and stop further checks for this line
                        obstacles.append([x, y])
                        break
        #return list of detected obstacle positions
        return obstacles




# MAIN

MAP_DIMENSIONS = (600, 1200)

#initialize graphics
gfx = Graphics(MAP_DIMENSIONS, r'C:\code\obstacle avoidance system\car.png', r'C:\code\obstacle avoidance system\map.png')

#initialize robot

start = (50, 300)  
robot = Robot(start, 0.01, MAP_DIMENSIONS[1], MAP_DIMENSIONS[0])

#initialize sensor
sensor_range = (250, math.radians(40))
ultra_sonic = Ultrasonic(sensor_range, gfx.map)

#time tracking for simulation
dt = 0
last_time = pygame.time.get_ticks()

running = True

# Simulation loop
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    #calculate the elapsed time (dt)
    current_time = pygame.time.get_ticks()
    dt = (current_time - last_time) / 1000.0  # Convert milliseconds to seconds
    last_time = current_time

    #reset the map to the initial state
    gfx.map.blit(gfx.map_img, (0, 0))

    #sense obstacles
    point_cloud = ultra_sonic.sense_obstacles(robot.x, robot.y, robot.orientation)
    gfx.draw_sensor_data(point_cloud)

    #avoid obstacles and update kinematics
    robot.avoid_obstacle(point_cloud, dt)
    robot.kinematics(dt)

    #draw the robot on the updated map
    gfx.draw_robot(robot.x, robot.y, robot.orientation)

    #update display
    pygame.display.update()

#quit Pygame when simulation ends
pygame.quit()
