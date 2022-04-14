import open3d as o3d
import os
import sys
import numpy as np


path = sys.argv[1]

print("path: " + path)

if (path == ""):
	path = "."
	
files = os.listdir(path)

pcd0 = o3d.io.read_point_cloud(path + "/" + files[0])

point_array = np.asarray(pcd0.points)
color_array = np.asarray(pcd0.colors)

for i in range(len(files)):
	
	if (i>0):
		pcd = o3d.io.read_point_cloud(path + "/" + files[i])
		ps = np.asarray(pcd.points)
		cs = np.asarray(pcd.colors)
		point_array = np.concatenate((point_array, ps))
		color_array = np.concatenate((color_array, cs))
	
merged = o3d.geometry.PointCloud()
merged.points = o3d.utility.Vector3dVector(point_array)
merged.colors = o3d.utility.Vector3dVector(color_array)
pcd_down = merged.voxel_down_sample(voxel_size=0.05)
o3d.io.write_point_cloud("merged_cloud.pcd", pcd_down)
