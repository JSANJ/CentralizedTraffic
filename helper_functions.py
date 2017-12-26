# -*- coding: utf-8 -*-
"""
Created on Fri Dec 22 23:29:16 2017

@author: JustinSanJuan
"""

import matplotlib.pyplot as plt

def print_image_bw(image,l,w):
    fig,ax=plt.subplots(ncols=1,nrows=1,figsize = (l,w))
    ax.imshow(image,cmap = 'binary')
    plt.show()