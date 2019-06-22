import open3d as o3d
import numpy as np
import pptk

print("Testing IO for point cloud")
pcd1 = o3d.io.read_point_cloud("bun045.ply")
print(pcd1)
data1 = np.asarray(pcd1.points)
print(type(data1))
print(np.asarray(pcd1.points).shape)
v=pptk.viewer(data1)
v.attributes(data1+1)
# ==========9===================================================================
# v=pptk.viewer(data1)
# v.attributes(data1+1)
# =============================================================================


pcd2 = o3d.io.read_point_cloud("bun000.ply")
print(pcd2)
data2 = np.asarray(pcd2.points)
print(type(data2))
print(np.asarray(pcd2.points).shape)

v=pptk.viewer(data2)
v.attributes(data2+1)

def Average(array): 
    return sum(array) / len(array) 

x_data1 = data1[:,0]
y_data1 = data1[:,1]
z_data1 = data1[:,2]

x_avg = Average(x_data1)
y_avg = Average(y_data1)
z_avg = Average(z_data1)
