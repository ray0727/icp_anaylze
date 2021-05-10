import math
import os
import rospy
import cv2
from sensor_msgs.msg import LaserScan, PointCloud2
import matplotlib.pyplot as plt
import open3d as o3d
import numpy as np

global num
num = 0

def laser_to_pc(laser):

    a = np.arange(start=-120, stop=121, step=1)
    a = np.radians(a)
    x = np.cos(a) * laser
    y = np.sin(a) * laser
    z = np.ones(a.shape)
    pc = np.dstack((x, y, z))
    pc = np.squeeze(pc)

    return pc

def scan_cb(msg):
    global num
    num+=1
    laser_range = msg.ranges
    pc_points = laser_to_pc(laser_range)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc_points)
    name = str(num)+".ply"
    o3d.io.write_point_cloud(name, pcd)


if __name__== "__main__":
    rospy.init_node('laserscan2pc')
    rospy.Subscriber("/husky1/RL/scan_mmwave", LaserScan, scan_cb, queue_size=1)
    rospy.spin()