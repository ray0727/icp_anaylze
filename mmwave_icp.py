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
source = o3d.io.read_point_cloud("./mmwave_laserscan/laser2pc_small_plate/1630.ply")
target = o3d.io.read_point_cloud("./mmwave_laserscan/laser2pc_small_plate/800.ply")

print("size source:", source)
print("size target:", target)
source = source.voxel_down_sample(voxel_size=0.05)
target = target.voxel_down_sample(voxel_size=0.05)

draw(source, target)

transform = np.asarray([[ 0.99564175, -0.09317902,  0.00389495, -0.06181105],
                        [ 0.09313781,  0.99560615,  0.00968209,  0.01593541],
                        [-0.00478001, -0.00927713,  0.99994554, -0.00825953],
                        [ 0.        ,  0.        ,  0.        ,  1.        ]])


source.transform(transform)
draw(source, target)