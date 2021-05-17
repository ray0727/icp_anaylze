#!/usr/bin/env python3
import math
import os
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
import matplotlib.pyplot as plt
import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as orh
import numpy as np

global num
num = 0

def cb_map(msg):
    global num
    num+=1
    pcd = o3d.geometry.PointCloud()
    pcd = orh.rospc_to_o3dpc(msg)
    name = str(num)+".ply"
    o3d.io.write_point_cloud(name, pcd)

if __name__== "__main__":
    rospy.init_node('laserscan2pc')
    rospy.Subscriber("/husky1/laser_cloud_surround", PointCloud2, cb_map, queue_size=1)
    rospy.spin()