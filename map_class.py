# -*- coding: utf-8 -*-
"""
Created on Sat Dec 23 01:52:44 2017

@author: JustinSanJuan
"""
import numpy as np
from constants import constant_speed
from constants import length_of_runway, max_value, intersection_x, intersection_y, intersection_w, intersection_h
from constants import car_width, car_length
from constants import intersection_area
import matplotlib.pyplot as plt
from helper_functions import print_image_bw
import time
import csv

class MapClass(object):
    def __init__(self,area_definition):
        self.area_definition = area_definition
        self.intersection_area = intersection_area
        self.collision_map = np.zeros((area_definition.shape[0],area_definition.shape[1]))
        self.intersection_collision_map = [[[] for i in range(intersection_w)] for j in range(intersection_h) ]        
        self.background_map = area_definition
        self.car_map = np.zeros((area_definition.shape[0],area_definition.shape[1]))
        self.area_occupied = [] #list
        self.image = []
        self.car_occupancy_blank = np.zeros((area_definition.shape[0],area_definition.shape[1]))
        self.car_occupancy = np.zeros((area_definition.shape[0],area_definition.shape[1]))
        
    def plot_cars(self, car_list):
        '''
        Plots cars in background_map
        Input: car_list
        Output: image (plot) of background_map with cars superimposed
        '''
        self.image = np.copy(self.background_map)
        #cars must be CarClass objects
        for car in car_list:
            self.image = car.plot_car(self.image)
        return self.image
    
    def get_area_occupied(self,car_list):
        '''
        Updates intersection_collision_map, which is a 2D list of lists,
        by adding the car object into the coordinates where the car needs to be clear
        Input: car_list
        Output: None, modified own intersection_collision_map with cars added into specific coordinates
        '''
        for car in car_list:
            car.get_area_to_clear(self.intersection_collision_map,car.source, car.target)
        return self.intersection_collision_map
    def update_cars(self,car_list):
        '''
        Runs update for each car (modifies velocity with current acceleration, modifies location with current velocity)
        and uses plot_cars to get plot of background_map with cars superimposed
        Input: car_list
        Output: image (plot) of background_map with cars superimposed with cars moved to updated locations based on acceleration & velocity
        '''
        for car in car_list:
            car.update_car()
        self.image = self.plot_cars(car_list)
        return self.image
    def save_image(self):
        '''
        Saves plot as a unique image
        Input: None, self.image
        Output: numpy file of self.image saved into current directory
        '''
        np.save('test_'+str(int(time)),self.image)