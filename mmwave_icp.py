import open3d as o3d
from pyntcloud import PyntCloud
import numpy as np
import os
import sys
import matplotlib.pyplot as plt
import copy
import pandas as pd

def draw(source, target):
    
    #yellow
    source_temp = copy.deepcopy(source)
    source_temp.paint_uniform_color([1, 0.706, 0])

    #blue
    target_temp = copy.deepcopy(target)
    target_temp.paint_uniform_color([0, 0.651, 0.929])

    #plot
    o3d.visualization.draw_geometries([source_temp, target_temp])

threshold = 8
source = o3d.io.read_point_cloud("./laser2pc_spherical/1622.ply")
target = o3d.io.read_point_cloud("./laser2pc_small_plate/1700.ply")
print("size source:", source)
print("size target:", target)
source = source.voxel_down_sample(voxel_size=0.05)
target = target.voxel_down_sample(voxel_size=0.05)

draw(source, target)

transform = np.asarray([[ 9.99979587e-01, 6.31990881e-03 ,9.40401255e-04 ,-1.15421668e+00],
                        [-6.32213845e-03, 9.99977166e-01 ,2.38716297e-03 ,-3.19260280e-01],
                        [-9.25293129e-04,-2.39305959e-03 ,9.99996709e-01 ,-3.20413616e-03],
                        [ 0.00000000e+00, 0.00000000e+00 ,0.00000000e+00 ,1.00000000e+00]])


source.transform(transform)
draw(source, target)