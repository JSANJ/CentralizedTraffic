# -*- coding: utf-8 -*-
"""
Created on Fri Dec 22 19:03:42 2017

@author: JustinSanJuan
"""
%load_ext autoreload
%autoreload 2
import numpy as np
from constants import area_definition #Area as matrix and matrix blocks
from constants import constant_speed
from constants import length_of_runway,max_value, intersection_x, intersection_y, intersection_w, intersection_h
from constants import car_width, car_length
import matplotlib.pyplot as plt
from helper_functions import print_image_bw
from car_class import CarClass
from map_class import MapClass
from constants import playback_speed

PATH = "C:/Users/JustinSanJuan/Desktop/Workspace/python/Centralized Traffic/" #path to files folder
#Define Areas
#area_definition
A = MapClass(area_definition)
#print_image_bw(A,5,5)
#%%
#Define Actions
#action_dictonary = define_actions(A)

#Define Areas to Clear


#Randomly generated source & target points

start_1 = (int(1+length_of_runway),
           int(0),
           int(car_width),
           int(car_length))
start_2 = (int(1+length_of_runway+intersection_w/4),
           int(0),
           int(car_width),
           int(car_length))
start_3 = (int(length_of_runway+intersection_w+length_of_runway-car_length),
           int(1+length_of_runway),
           int(car_length),
           int(car_width))
start_4 = (int(length_of_runway+intersection_w+length_of_runway-car_length),
           int(1+length_of_runway+intersection_h/4),
           int(car_length),
           int(car_width))
start_5 = (int(-1+length_of_runway+intersection_w-car_width),
           int(length_of_runway+intersection_h+length_of_runway-car_length),
           int(car_width),
           int(car_length))
start_6 = (int(-1+length_of_runway+intersection_w-car_width-intersection_w/4),
           int(length_of_runway+intersection_h+length_of_runway-car_length),
           int(car_width),
           int(car_length))
start_7 = (int(0),
           int(-1+length_of_runway+intersection_h-car_width),
           int(car_length),
           int(car_width))
start_8 = (int(0),
           int(-1+length_of_runway+intersection_h-car_width-intersection_h/4),
           int(car_length),
           int(car_width))
car_1 = CarClass(*start_1, [0,0],[0,3],A.intersection_collision_map,[0,9*playback_speed])
#car_2 = CarClass(*start_2)
car_3 = CarClass(*start_3,[3,0],[0,0],A.intersection_collision_map,[-10*playback_speed,0])
car_4 = CarClass(*start_4,[3,1],[0,1],A.intersection_collision_map,[-10*playback_speed,0])
car_7 = CarClass(*start_7,[0,3],[3,3],A.intersection_collision_map,[8*playback_speed,0])
car_8 = CarClass(*start_8,[0,2],[3,2],A.intersection_collision_map,[8*playback_speed,0])
#car_5 = CarClass(*start_5)
#car_1.v = [0,9]
#car_1.s = 9 #v in x, y (+'ve y is downwards)
##car_2.v = [0,25]
##car_3.v = [int(-30/4),0]
#car_3.v = [-10,0]
#car_3.s = 10
#car_4.v = [-10,0]
#car_4.s = 10

#car_list = [car_1, car_2,car_3,car_4]
global car_list
car_list = [car_1, car_4,car_3,car_7,car_8]
A = MapClass(area_definition)
for car in car_list:
    A.intersection_collision_map = car.get_area_to_clear(A.intersection_collision_map,car.source,car.target)

car_3.is_collision_possible(car_list, A.intersection_collision_map)
#car_3.take_action(car_list)
car_4.is_collision_possible(car_list, A.intersection_collision_map)
car_7.is_collision_possible(car_list, A.intersection_collision_map)
car_8.is_collision_possible(car_list, A.intersection_collision_map)
#car_4.take_action(car_list)

#Simulate


import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
global image
image = np.copy(A.background_map)
def animate(i):
    ax1.clear()
    ax1.imshow(A.background_map)
    image = A.update_cars(car_list)
#    A.save_image
    ax1.imshow(image,cmap='binary')
ani = animation.FuncAnimation(fig, animate, interval=100*playback_speed)
plt.show()
#start 1,2,3,4,5,6,7,8

#%%
area_occupied = A.get_area_occupied(car_list)
image = A.update_cars(car_list)


car_1.get_area_to_clear(A.intersection_area,(0,0),(0,3))

car_1.is_collision_possible(A.area_occupied)

car_1.area_to_clear
A.area_occupied
