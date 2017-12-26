# -*- coding: utf-8 -*-
"""
Created on Sat Dec 23 01:41:40 2017

@author: JustinSanJuan
"""
import numpy as np
from constants import area_definition #Area as matrix and matrix blocks
from constants import constant_speed
from constants import length_of_runway, max_value, intersection_x, intersection_y, intersection_w, intersection_h
from constants import intersection_x1, intersection_x2, intersection_y1, intersection_y2
from constants import car_width, car_length, a_max, s_max
import matplotlib.pyplot as plt
from helper_functions import print_image_bw
from constants import vertical_straight_threshold, horizontal_straight_threshold
from constants import intersection_speed
intersection_coords_x1x2y1y2 = (intersection_x1, intersection_x2, intersection_y1, intersection_y2)
from constants import safety_distance
class CarClass(object):
    def __init__(self,x,y,w,h,source, target, intersection_collision_map,v):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.area_to_clear_coords_x1x2y1y2 = (0,0,0,0)
        self.collision_possible = 0
        self.v = v # in [x,y]
        self.a = [0,0]
        
        self.s = np.sqrt(self.v[0]*self.v[0]+self.v[1]*self.v[1])

        
        self.r = 0
        self.omega = 0
        self.alpha = 0
        
        self.source = source
        self.target = target
        
        print(self.s)
        self.a_scalar = 0
        self.d = 0
        self.t = 0
        self.at_intersection = 0
        self.cleared_intersection = 0
        self.current_location_x1x2y1y2 = (self.x, self.x+self.w,self.y,self.y+self.h)
        self.direction = ''
        self.intersection_collision_map = intersection_collision_map
        self.collision_coords_x1x2y1y2 = ()
        self.absolute_collision_coords_x1x2y1y2 = (256,256,256+128,256+128)
        self.at_collision = 0
        
#    def restart_car(self,x,y,w,h):
#        self.x = x
#        self.y = y
#        self.w = w
#        self.h = h
    def check_overlap_with_intersection(self):
#        self.at_intersection = 0
        #check that car is not in intersection
        if (self.current_location_x1x2y1y2[0] > intersection_coords_x1x2y1y2[1] or # car x1 (left) > intersection x2 (right)
           self.current_location_x1x2y1y2[1] < intersection_coords_x1x2y1y2[0] or # car x2 (right) < intersection x1 (left)
           self.current_location_x1x2y1y2[2] > intersection_coords_x1x2y1y2[3] or # car y1 (up) > intersection y2 (down)
           self.current_location_x1x2y1y2[3] < intersection_coords_x1x2y1y2[2]):# car y2 (down) < intersection y2 (up)
            if self.at_intersection == 1:
                self.cleared_intersection = 1
                self.at_intersection = 0
            self.at_intersection = 0
        else: 
            self.at_intersection = 1
            
        return self.at_intersection
    def check_overlap_with_collision(self):
#        self.at_intersection = 0
        #check that car is not in intersection
        if (self.current_location_x1x2y1y2[0] > self.absolute_collision_coords_x1x2y1y2[1] or # car x1 (left) > intersection x2 (right)
           self.current_location_x1x2y1y2[1] < self.absolute_collision_coords_x1x2y1y2[0] or # car x2 (right) < intersection x1 (left)
           self.current_location_x1x2y1y2[2] > self.absolute_collision_coords_x1x2y1y2[3] or # car y1 (up) > intersection y2 (down)
           self.current_location_x1x2y1y2[3] < self.absolute_collision_coords_x1x2y1y2[2]):# car y2 (down) < intersection y2 (up)
            self.at_collision= 0
        else: 
            self.at_collision = 1
            
        return self.at_intersection
    
    def update_car(self):
        #change speeds due to acceleration
        self.current_location_x1x2y1y2 = (self.x, self.x+self.w,self.y,self.y+self.h)
        self.check_overlap_with_intersection()
        if self.cleared_intersection == 1 and self.at_intersection == 0 and self.direction == 'horizontal': # at_intersection should be 0 when cleared-intersection == 1
            self.a_scalar = a_max/2 #accelerate when out of intersection
        if self.v[0] > 0:
            self.a[0] = self.a_scalar
        elif self.v[0] < 0:
            self.a[0] = -self.a_scalar
#        self.a[0] = -self.a_scalar # to be changed when vectorized
        self.v[0] += self.a[0]
        self.v[1] += self.a[1]
        self.omega += self.alpha
        #once at collision, start accelerating
        self.check_overlap_with_collision()
        if self.at_collision == 1: #constant speed at intersection
            if self.v[0] > 0:
                self.v[0] = intersection_speed
            elif self.v[0] < 0:
                self.v[0] = -intersection_speed
            elif self.v[1] > 0:
                self.v[1] = intersection_speed
            elif self.v[1] < 0:
                self.v[1] = -intersection_speed
            self.a = [0,0]
        
        #set cap on speed
        if self.v[0] >= s_max: self.v[0] = s_max
        if self.v[0] <= -s_max: self.v[0] = -s_max
        if self.v[1] >= s_max: self.v[1] = s_max
        if self.v[1] <= -s_max: self.v[1] = -s_max
        #change displacements due to speed
        self.x += self.v[0]
        self.y += self.v[1]
        self.s = np.sqrt(self.v[0]*self.v[0]+self.v[1]*self.v[1]) # speed
        self.r += self.omega
        
        if self.x < 0: self.x = -60
        if self.y < 0: self.y = -60        
        self.current_location_x1x2y1y2 = (self.x, self.x+self.w,self.y,self.y+self.h)
        
    def plot_car(self,image):
        '''
        From an image, add the car on top
        Input: Current image as array
        Output: Current image as array with car superimposed
        Notes: To be generalized to arrays of pixels for ability to handle rotated cars
        '''
        image[int(self.y):int(self.y+self.h),int(self.x):int(self.x+self.w)] = max_value
#        print(self.x)
#        print(self.y)
#        print(self.w)
#        print(self.h)
#        print('plotting_car')
        return image
    def cleared_area(self):
        x1,x2,y1,y2 = self.area_to_clear_coords_x1x2y1y2
        
        if self.direction == 'vertical':
            for j in range(len(self.intersection_collision_map)): # number of rows (y)
                for i in range(int(self.source[0]*intersection_w/4),int((self.source[0]+1)*intersection_w/4)):
                    self.intersection_collision_map[j][i].remove(self)
        elif self.direction == 'horizontal':
            for j in range(int(self.source[1]*intersection_w/4),int((self.source[1]+1)*intersection_w/4)): # number of rows
                for i in range(len(self.intersection_collision_map)): # (x) number of columns assuming square intersection area
                    self.intersection_collision_map[j][i].remove(self)
                    
    def area_to_clear_straight(self,intersection_collision_map):
        print('area_to_clear_straight')
        #direction is 1 or 0, 1 = horizontal, 0 = vertical
        if self.direction == 'vertical':
            print('vertical')
#            self.area_to_clear_coords_x1x2y1y2 = (int(self.source[0]*intersection_w/4),int((self.source[0]+1)*intersection_w/4),0,intersection_w)
            for j in range(len(intersection_collision_map)): # number of rows (y)
                for i in range(int(self.source[0]*intersection_w/4),int((self.source[0]+1)*intersection_w/4)):
                    if not(self in intersection_collision_map[j][i]):
                        intersection_collision_map[j][i].append(self)
                        
        elif self.direction == 'horizontal':
            print('horizontal')
#            self.area_to_clear_coords_x1x2y1y2 = (0,intersection_h,int(self.source[1]*intersection_w/4),int((self.source[1]+1)*intersection_w/4))
            for j in range(int(self.source[1]*intersection_w/4),int((self.source[1]+1)*intersection_w/4)): # number of rows
                for i in range(len(intersection_collision_map)): # (x) number of columns assuming square intersection area
                    if not(self in intersection_collision_map[j][i]):
                        intersection_collision_map[j][i].append(self)
                        
        return intersection_collision_map
    
    def get_area_to_clear(self, intersection_collision_map, source,target):
        print('get_area_to_clear')
        #source is in [y,x]
        self.source = source
        self.target = target
        
        if self.target[1] <= self.source[1] + vertical_straight_threshold and self.target[1] >= self.source[1] - vertical_straight_threshold:
            print('horizontal')
            self.direction = 'horizontal'
            intersection_collision_map = self.area_to_clear_straight(intersection_collision_map)
        elif self.target[0] <= self.source[0] + horizontal_straight_threshold and self.target[0] >= self.source[0] - horizontal_straight_threshold:
            print('vertical')
            self.direction = 'vertical'
            intersection_collision_map = self.area_to_clear_straight(intersection_collision_map)
#        self.area_to_clear.flatten().tolist()
#        self.area_to_clear = [y for x in self.area_to_clear for y in x]
        return intersection_collision_map # in coordinates data of intersection_collision_map
        
#    def get_collision_coords(self):
#        '''
#        Iterates through intersection_collision_map to find where the collision with other objects are
#        Input: None, self.intersection_collision_map
#        Output: x1 x2 y1 y2 coordinates of collision area
#        '''
    def take_action(self,car_list):
        print('take_action')
        '''
        Given crash possibilities, update acceleration and/or speed as necessary
        Input: Collision area, crash pair
        Output: Modified acceleration or remove excess speed
        '''

        if self.collision_possible == 1: #if collision is possible, update acceleration as needed
            #self.collision_coords_x1x2y1y2 must be set
#            for other_car in self.collision_pairs:
            other_car = self.get_crash_pair(car_list) # temporarily set to car_1
#            d = self.get_distance_to_intersection(self.current_location_x1x2y1y2)
            d = self.get_distance_to_collision(self.absolute_collision_coords_x1x2y1y2, self.current_location_x1x2y1y2)
            print('d')
            print(d)
            t0 = self.get_time_to_collision()
            print('t0')
            print(t0)
            t1 = self.get_time_to_clear(other_car, self.absolute_collision_coords_x1x2y1y2)
            print('t1')
            print(t1)
            t_to_pass_over = (30+60+safety_distance)/self.s
            t2 = t0 + t_to_pass_over
            if t1 > t0:
                self.a_scalar = self.get_acceleration(d,self.s,t1)
                print('set acceleration to: ' + str(self.a_scalar))
    
            elif t1 < t2: 
                self.a_scalar = a_max/2
        
    def is_collision_possible(self,car_list, intersection_collision_map):
        self.collision_possible = 0
        self.collision_pairs = []
#        self.get_area_to_clear(self.intersection_collision_map,self.source,self.target)
        x1,x2,y1,y2 = self.area_to_clear_coords_x1x2y1y2
        collision_coord_x1 = int(len(intersection_collision_map))
        collision_coord_x2 = 0
        collision_coord_y1 = int(len(intersection_collision_map))
        collision_coord_y2 = 0
        
        #check if current car's area to be cleared (on intersection_collision_map) is to be occupied by another car 
        # can be optimized by checking only current car's area to be cleared (currently checking whole map)
        for j in range(len(intersection_collision_map)):
#            print('j = '+str(j))
            for i in range(len(intersection_collision_map)):
#                print('i = ' +str(i))
#                print(intersection_collision_map[j][i])
                if self in intersection_collision_map[j][i] and len(intersection_collision_map[j][i]) > 1:
                    self.collision_possible = 1
                    if j < collision_coord_y1: collision_coord_y1 = j
                    if j > collision_coord_y2: collision_coord_y2 = j
                    if i < collision_coord_x1: collision_coord_x1 = i
                    if i > collision_coord_x2: collision_coord_x2 = i
                    for k in intersection_collision_map[j][i]:
                        if not(k in self.collision_pairs) and k != self:
                            print(k)
                            self.collision_pairs.append(k)
                            
        self.collision_coords_x1x2y1y2 = (collision_coord_x1,collision_coord_x2,collision_coord_y1, collision_coord_y2)
        self.absolute_collision_coords_x1x2y1y2 = (length_of_runway + collision_coord_x1,
                                                   length_of_runway + collision_coord_x2,
                                                   length_of_runway + collision_coord_y1,
                                                   length_of_runway + collision_coord_y2)

        print('collision_possible = '  +str(self.collision_possible))
        self.take_action(car_list)        
        return self.collision_possible
    def get_x1_x2_y1_x2_from_xywh(coords_tuple):
        x, y, w, h = coords_tuple
        x1 = x
        x2 = x + w
        y1 = y
        y2 = y + h
        new_coords_tuple = (x1, x2, y1, y2)
        return new_coords_tuple
    def get_collision_area(self, area_occupied_coords, area_to_clear_coords):
        aoc_test = (0,0,50,60)
        atc_test = (0,0,128,30)
        area_occupied_coords = get_x1_x2_y1_y2_from_xywh(area_occupied_coords)
        area_to_clear_coords = get_x1_x2_y1_y2_from_xywh(area_to_clear_coords)
        # Overlap percentage of regions defined as (A1 intersect with A2)/(A1 union A2) * 100%
        # Create 2x4 matrix called overlap set, which has test row 'u' and temp_set row 'n' data
        overlap_set = []
        overlap_set.append(area_occupied_coords.tolist())
        overlap_set.append(area_to_clear_coords.tolist())
        overlap_set = np.asarray(overlap_set)
        
            # get overlap area
        overlap_x1 = max(overlap_set[:,0])
        overlap_x2 = min(overlap_set[:,1])
        overlap_y1 = max(overlap_set[:,2])
        overlap_y2 = min(overlap_set[:,3])
#        
#        overlap_w = overlap_x2 - overlap_x1
#        overlap_h = overlap_y2 - overlap_y1
        collision_coords_x1x2y1y2 = (overlap_x1, overlap_x2, overlap_y1, overlap_y2)
        return collision_coords_x1x2y1y2
    def get_distance_to_intersection(self, current_location_x1x2y1y2):
        #distance of right of intersection to left of current location
        
        right_left = np.absolute((intersection_x+intersection_w)-current_location_x1x2y1y2[0])
        
        #distance of left of intersection to right of current location
        left_right = np.absolute((intersection_x)-current_location_x1x2y1y2[1])
        
        #distance of top of intersection to bottom of current location
        up_down = np.absolute((intersection_y)-current_location_x1x2y1y2[4])
        
        #distance of bottom of intersection to top of current location
        down_up = np.absolute((intersection_y+intersection_h)-current_location_x1x2y1y2[3])
        if self.v[0] > 0 and self.v[1]== 0: # going right
            self.d = left_right
        elif self.v[0] < 0 and self.v[1] == 0: # going left
            self.d = right_left
        elif self.v[0] == 0 and self.v[1] > 0: # going down
            self.d = up_down
        elif self.v[0] == 0 and self.v[1] < 0: # going up
            self.d = down_up
        return self.d 
    
    def get_distance_to_collision(self,absolute_collision_coords_x1x2y1y2, current_location_x1x2y1y2):   
#        collision_coords_x1x2y1y2 = (257, 257+30, 257, 257+30) # temporary data
        print(absolute_collision_coords_x1x2y1y2)
        right_left = np.absolute(absolute_collision_coords_x1x2y1y2[0] - current_location_x1x2y1y2[1])
        left_right = np.absolute(absolute_collision_coords_x1x2y1y2[1] - current_location_x1x2y1y2[0])
        up_down = np.absolute(absolute_collision_coords_x1x2y1y2[2] - current_location_x1x2y1y2[3])
        down_up = np.absolute(absolute_collision_coords_x1x2y1y2[3] - current_location_x1x2y1y2[2])
        if self.v[0] != 0 and self.v[1]==0:
            self.d = min(right_left,left_right)
        elif self.v[0] == 0 and self.v[1] != 0:
            self.d = min(up_down,down_up)
        return self.d 
    
    def get_time_to_collision(self):
        self.t = 0
        if self.d != 0:
            self.t = self.d/self.s         
        return self.t
    def get_time_to_clear(self,other_car,absolute_collision_coords_x1x2y1y2):
        print('get_time_to_clear')
        '''
        time to arrive to the collision_area + time to pass over collision_area
        temporary data
        '''
        other_car.get_distance_to_collision(absolute_collision_coords_x1x2y1y2,other_car.current_location_x1x2y1y2)
        print('other_car.d')
        print(other_car.d)
        t_to_arrive = other_car.get_time_to_collision()
        print('t_to_arrive of other car')
        print(t_to_arrive)
        t_to_pass_over = (30+60+safety_distance)/other_car.s #distance to clear is length of collision area + car length
        print('t_to_pass_over')
        print(t_to_pass_over)
        self.t1 = t_to_arrive+t_to_pass_over
        print('t1')
        print(self.t1)
        return self.t1
    def get_crash_pair(self,car_list): #in car_list, find other car to crash with
#        for car in car_list:
#        and crash car cannot be self
        return car_list[0] #return car_1 first
    def get_acceleration(self,d,v0,t):
        print(d)
        print(v0)
        print(t)
        self.a_scalar = (2*float(d) - 2*float(v0)*float(t))/(float(t)*float(t))
        return self.a_scalar
    
    
#    def get_action(self,source,target):
#        pass
    
    
    
    
    
#    def define_actions(A):
#        width = A.shape[1]
#        height = A.shape[0]
#        #assume equal lanes for forward/backward
#        #right-hand or left-hand lane # assume right-hand lane
#        action_dictionary = {}
#        action_dictionary.append()
#        
#        return action_dictionary
        