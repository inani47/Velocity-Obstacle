import numpy as np
import matplotlib.pyplot as plt
from IPython import display
from math import sin, cos, atan2, pi
from matplotlib.pyplot import pause


class Point:
    def __init__(self, x: float, y: float):
        self.x = float(x)
        self.y = float(y)


class Entity:
    def __init__(self, radius: float, center: Point, speed: float, heading: float ):
        self.radius = radius
        self.center = center # this is x, y
        self.heading = heading
        self.color = 'red'
        self.speed = speed # this is xp, yp
        self.acceleration = 0 # this is vp (or speedp)
        self.angular_velocity = 0 # this is headingp
        self.max_speed = 5.0
        self.min_speed = 0.0

    # def set_velocity(self, new_vel: Point):
    #     self.velocity = min(new_vel, self.max_speed)

    def update(self, dt: float):
        self.center.x = self.center.x + self.speed*cos(self.heading)*dt
        self.center.y = self.center.y + self.speed*sin(self.heading)*dt

    def get_pos(self):
        return self.center

    def get_radius(self):
        return self.radius

    def get_heading(self):
        return self.heading

    def get_speed(self):
        return self.speed



class Ego:
    def __init__(self, radius: float, center: Point, speed: float, heading: float, goal:Point ):
        self.radius = radius
        self.center = center # this is x, y
        self.heading = heading
        self.color = 'red'
        self.speed = speed # this is xp, yp
        self.acceleration = 0 # this is vp (or speedp)
        self.angular_velocity = 0 # this is headingp
        self.max_speed = 5.0
        self.min_speed = 0.0
        self.goal = goal

    # def set_velocity(self, new_vel: Point):
    #     self.velocity = min(new_vel, self.max_speed)

    def update(self, dt: float):
        self.center.x = self.center.x + self.speed*cos(self.heading)*dt
        self.center.y = self.center.y + self.speed*sin(self.heading)*dt

    def get_pos(self):
        return self.center

    def get_radius(self):
        return self.radius

    def get_heading(self):
        return self.heading

    def get_speed(self):
        return self.speed

    def obst_avoid(self, obs_heading: float, obs_rad: float, obs_pos: Point, obs_speed: float):


        # get direction to goal
        dir_to_goal = atan2(self.goal.y - self.center.y , self.goal.x - self.center.x)

        # sample candidate directions in increments of 10 deg
        dirs = []
        for i in range(0,35):
            dirs.append(dir_to_goal + (i*10* (pi/180)))

        r = np.array([obs_pos.x -self.center.x, obs_pos.y -self.center.y])

        v_obs = np.array([obs_speed*cos(obs_heading) , obs_speed*sin(obs_heading)])


        # filter down to velocities that lie outside the VO
        filtered_dirs = []
        for i in range(0,35):
            v_ego = np.array([self.speed*cos(dirs[i]) , self.speed*sin(dirs[i])])
            v_rel = v_obs - v_ego

            f = (np.dot(r,v_ego-v_obs)**2/(np.linalg.norm(v_ego-v_obs)**2) ) - (np.linalg.norm(r)**2) + (self.radius + obs_rad + 0.2)**2 

            effective_dist = np.linalg.norm(r) - (self.radius + obs_rad)
            t_lim = 2.0

            if(np.linalg.norm(v_rel) <= effective_dist/t_lim):
                f = -1
            if(f < 0 or np.dot(r,v_rel)) > 0:
                filtered_dirs.append(dirs[i])


        
        # select the velocity that minimizes the cost function
        # cost function min || goal - pos_after_forward_propogation||  or
        # min |goal_dir - filtered_dir_i| for all i
        best_heading = filtered_dirs[0]
        fwd_prop_x = self.center.x + self.speed*cos(self.heading)*0.1
        fwd_prop_y = self.center.y = self.center.y + self.speed*sin(self.heading)*0.1
        j_min = (self.goal.x - fwd_prop_x)**2 + (self.goal.y - fwd_prop_y)**2 
        for i in range(i,len(filtered_dirs)):
        #     #update center
             
            fwd_prop_x = self.center.x + self.speed*cos(filtered_dirs[i])*0.1
            fwd_prop_y = self.center.y + self.speed*sin(filtered_dirs[i])*0.1
            j = (self.goal.x - fwd_prop_x)**2 + (self.goal.y - fwd_prop_y)**2 
            if (j<j_min):
                j_min = j
                best_heading = filtered_dirs[i]

        self.heading = best_heading
        self.center.x = self.center.x + self.speed*cos(self.heading)*0.1
        self.center.y = self.center.y + self.speed*sin(self.heading)*0.1





obstacle = Entity(radius=0.5, center = Point(17,10), speed=1.0, heading = -3.14159)

ego = Ego(radius=0.5, center = Point(0,10), speed=1.0, heading = 0.0, goal = Point(10.0,10.0))




for x in range(0,200):
    obstacle.update(0.1)
    
    obs_x = obstacle.get_pos().x
    obs_y = obstacle.get_pos().y
    obs_rad = obstacle.get_radius()
    obs_heading = obstacle.get_heading()
    obs_speed = obstacle. get_speed()

    

    ego.obst_avoid(obs_heading, obs_rad, Point(obs_x,obs_y), obs_speed)

    # r = np.array[obs_x-ego_x, obs_y - ego_y]
    # v_ego = np.array[1.0*cos(ego_heading) , 1.0*sin(ego_heading)]  

    # v_obs = np.array[1.0*cos(obs_heading) , 1.0*sin(obs_heading)]  

    ego_x = ego.get_pos().x
    ego_y = ego.get_pos().y
    ego_rad = ego.get_radius()
    ego_heading = ego.get_heading()

    print(ego_x)
    circle1 = plt.Circle((obs_x, obs_y), obs_rad, color='r')
    circle2 = plt.Circle((ego_x, ego_y), ego_rad, color='b')

    
    ax = plt.gca()
    ax.cla() # clear things for fresh plot

    # change default range so that new circles will work
    ax.set_xlim((0, 20))
    ax.set_ylim((0, 20))

        
    ax.add_patch(circle1)
    ax.add_patch(circle2)
    ax.arrow(obs_x,obs_y, cos(obs_heading),  sin(obs_heading), head_width = 0.2, width = 0.05) 
    ax.arrow(ego_x,ego_y, cos(ego_heading),  sin(ego_heading), head_width = 0.2, width = 0.05) 


    plt.pause(0.1)