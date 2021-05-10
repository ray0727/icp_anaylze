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
source = o3d.io.read_point_cloud("./spherical_pcd/spherical_door.pcd")
target = o3d.io.read_point_cloud("./small_plate_pcd/smallplate_door.pcd")
print("size source:", source)
print("size target:", target)
source = source.voxel_down_sample(voxel_size=0.05)
target = target.voxel_down_sample(voxel_size=0.05)

trans_init = np.asarray([[1, 0 ,0, 0],
                         [0, 1, 0, 0],
                         [0, 0, 1, 0], 
                         [0.0, 0.0, 0.0, 1.0]])
draw(source, target)
print("Initial alignment")

evaluation = o3d.pipelines.registration.evaluate_registration(source, target, threshold, trans_init)
print(evaluation)

print("Apply point-to-point ICP")
icp_p2p = o3d.pipelines.registration.registration_icp(
    source, target, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))

print(icp_p2p)
print("Transformation:")
print(icp_p2p.transformation)
source.transform(icp_p2p.transformation)
draw(source, target)
