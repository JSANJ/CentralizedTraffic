# -*- coding: utf-8 -*-
"""
Created on Fri Dec 22 19:08:41 2017

@author: JustinSanJuan
"""
import numpy as np
import csv
playback_speed = 0.2
s_max = 25*playback_speed
a_max = 5*playback_speed

decela_max = -10
intersection_speed = 10
constant_speed = 1
vertical_straight_threshold = 0
horizontal_straight_threshold = 0
intersection_width = 128
intersection_height = 128
safety_distance = 10

start_p = 1
end_p = start_p + (intersection_width/2) * (intersection_height/2)
start_q = end_p
end_q = start_q + (intersection_width/2) * (intersection_height/2)
start_l = end_q 
end_l = start_l + (intersection_width/2) * (intersection_height/2)
start_m = end_l
end_m = start_m + (intersection_width/2) * (intersection_height/2)
p = np.reshape(np.arange(start_p, end_p), (int(intersection_height/2), int(intersection_width/2)))
q = np.reshape(np.arange(start_q, end_q), (int(intersection_height/2), int(intersection_width/2)))
l = np.reshape(np.arange(start_l, end_l), (int(intersection_height/2), int(intersection_width/2)))
m = np.reshape(np.arange(start_m, end_m), (int(intersection_height/2), int(intersection_width/2)))
#intersection_area = np.bmat([[p,q],[l,m]]).astype(np.int)


length_of_runway = 256
max_value = end_m #empty space is darkest
file = csv.reader(open("Intersection Map.csv",), delimiter = ',')
intersection_area = np.reshape(np.array(list(file)).astype(np.int)*max_value,(128,128))

empty_space = np.ones((length_of_runway,length_of_runway))*max_value
runway_vertical = np.zeros((length_of_runway,intersection_width)) #zero is runway
runway_horizontal = np.zeros((intersection_height,length_of_runway)) #zero is runway

area_definition = np.bmat([[empty_space, runway_vertical, empty_space],
                           [runway_horizontal,intersection_area,runway_horizontal],
                           [empty_space, runway_vertical, empty_space]]).astype(np.int)

intersection_x = length_of_runway
intersection_y = length_of_runway
intersection_w = intersection_width
intersection_h = intersection_height

car_l_to_w_ratio = 2
car_width = 30
car_length = car_width*car_l_to_w_ratio

intersection_x1 = intersection_x
intersection_x2 = intersection_x + intersection_w
intersection_y1 = intersection_y
intersection_y2 = intersection_y + intersection_h


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
