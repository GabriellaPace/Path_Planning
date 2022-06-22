#!/usr/bin/python3
import sys, os
import cv2
import math
import numpy as np
from noise import pnoise2, snoise2
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import random
import faulthandler

faulthandler.enable()

gui = False
scale = 1
#amplitude = 1
cmap = cm.twilight  #for gui
width = height = 500 #1000
out_image = np.zeros((height+2, width+2, 1), np.float32)

sign = lambda a: (a>0) - (a<0)
ridged = lambda a: 2*(0.5-abs(0.5-a))

for sample in range(3):  #for sample in range(40):
  randBytes = os.urandom(3)
  rand1 = int.from_bytes(randBytes, byteorder='big')%100 #%1000
  randBytes = os.urandom(3)
  rand2 = int.from_bytes(randBytes, byteorder='big')%100 #%1000
  randBytes = os.urandom(3)
  rand3 = int.from_bytes(randBytes, byteorder='big')%100 #%1000


  for y in range(height+2):
    for x in range(width+2):
          a = pnoise2(    x/ 1000.0/ scale, y    / 1000.0/ scale, base=rand1)
          b = pnoise2((y+x)/ 500.0 / scale, (y-x)/ 500.0 / scale, base=rand2)
          c = pnoise2(    y/ 250.0 / scale, x    / 250.0 / scale, base=rand3)
          a = (a+1)/2.0
          b = (b+1)/2.0
          c = (c+1)/2.0
          a = a*a*a
          b = b*b
          a = a * 60
          b = b * 40
          c = c * 20
          out_image[y][x] = (a+b+c) #*amplitude

  out_image = cv2.GaussianBlur(out_image, (7,7), 0)
             # Sobel = calculate the derivatives from an image (to obtain the slope from the height?):
             # Sobel:(src,      ddepth,    dx, dy, dts, ksize,   scale,      delta, borderType) -> Any
  sobelx = cv2.Sobel(out_image, cv2.CV_32F, 1, 0,       ksize=3, scale=0.25,        borderType=cv2.BORDER_ISOLATED)   
  sobely = cv2.Sobel(out_image, cv2.CV_32F, 0, 1,       ksize=3, scale=0.25,        borderType=cv2.BORDER_ISOLATED)

  gradient = np.sqrt(np.square(sobelx)+np.square(sobely))
  slope = np.degrees(np.arctan(gradient))
  slope = slope[1:-1, 1:-1]
  slope = np.clip(slope,0,15)/15
  slope = np.sqrt(slope)
  slope = np.uint8(slope*255)

  if gui:
    fig = plt.figure()
    X = range(height+2)
    Y = range(width+2)
    X, Y = np.meshgrid(X, Y)
    Y = Y[::-1]
    ax = fig.add_subplot(2, 2, 1, projection='3d')
    ax.view_init(elev=45, azim=-70)
    ax.set_zlim(0, 55)
    surf = ax.plot_surface(X, Y, out_image, cmap=cmap, antialiased=True)
    plt.title("Height map")
    ax = fig.add_subplot(2, 2, 3)
    ax.imshow(out_image, cmap=cmap)

    X = range(height)
    Y = range(width)
    X, Y = np.meshgrid(X, Y)
    Y = Y[::-1]
    ax = fig.add_subplot(2, 2, 2, projection='3d')
    ax.set_zlim(0, 255)
    ax.view_init(elev=45, azim=-70)
    surf2 = ax.plot_surface(X, Y, slope, cmap=cmap, antialiased=True)
    plt.title("(Square root) Slope map - MAX 15 degrees")
    ax = fig.add_subplot(2, 2, 4)
    ax.imshow(slope, cmap=cmap)
    plt.tight_layout()
    plt.show()
  
  path = "C:/Dev/Path_Planning/Maps"
  #cv2.imwrite(f"{path}/{sample}_dem.tiff", out_image)     # cv2.imwrite("%03d_dem.tiff"%sample, out_image)    # out_image = height map
  cv2.imwrite(f"{path}/{sample}_gradient.bmp", ~slope)    # cv2.imwrite("%03d_gradient.bmp"%sample, ~slope)