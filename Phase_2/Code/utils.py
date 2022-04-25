# Importing Modules
import os
import cv2
import shutil
from glob import glob
import csv
import numpy as np


def generate_video(path, video_name="simulation_video", fps = 10):

    images = sorted(glob(path+"/*.png"))
    frame = cv2.imread(os.path.join(images[0]))
    height, width, channels = frame.shape
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video = cv2.VideoWriter(os.path.join(path, video_name + ".mp4"), fourcc, fps , (width,height))
    imgs = []
    for i in range(len(images)):
        image = cv2.imread(images[i])
        if i==0:
            h, w, _ = image.shape
        video.write(image)
    cv2.destroyAllWindows()
    video.release()

def deg2rad(rot): 
    return 3.14*rot/180
    
def rad2deg(theta_new):
    theta_new = 180*theta_new/3.14    
    if (theta_new >= 360): theta_new -= 360
    if (theta_new <= -360): theta_new += 360
    return theta_new

def foldercheck(path):
    if not os.path.exists(path):
        os.makedirs(path)
    return path

def half_round(x):
    x = round(2*x)/2    
    if x == 10 : x-=0.5
    return x
