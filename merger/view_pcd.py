import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import sys

file_pcd = sys.argv[1]

merged_pcd = o3d.io.read_point_cloud(file_pcd)
#down_pcd = merged_pcd.voxel_down_sample(voxel_size=0.05)

o3d.visualization.draw_geometries([merged_pcd])


