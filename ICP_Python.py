
import open3d as o3d
import numpy as np
import pptk


#E =[]
#P =[]
#Q=[]
print("Testing IO for point cloud")
pcd1 = o3d.io.read_point_cloud("bun045.ply")
print(pcd1)
data1 = np.asarray(pcd1.points)
print(type(data1))
print(np.asarray(pcd1.points).shape)
#v=pptk.viewer(data1)                            
#v.attributes(data1+1)
# ==========9===================================================================
# v=pptk.viewer(data1)
# v.attributes(data1+1)
# =============================================================================


pcd2 = o3d.io.read_point_cloud("bun000.ply")
print(pcd2)
data2 = np.asarray(pcd2.points)
print(type(data2))
print(np.asarray(pcd2.points).shape)

#v=pptk.viewer(data2)
#v.attributes(data2+1)

def Average(array): 
    return sum(array) / len(array) 

x_data1 = data1[:,0]
y_data1 = data1[:,1]
z_data1 = data1[:,2]

x_avg_data1 = Average(x_data1)
y_avg_data1 = Average(y_data1)
z_avg_data1 = Average(z_data1)
mean_data1 = [x_avg_data1, y_avg_data1, z_avg_data1] 


#=====================================

x_data2 = data2[:,0]
y_data2 = data2[:,1]
z_data2 = data2[:,2]

x_avg_data2 = Average(x_data2)
y_avg_data2 = Average(y_data2)
z_avg_data2 = Average(z_data2)

mean_data2 = [x_avg_data2, y_avg_data2, z_avg_data2]

data1_centroid = data1 - mean_data1
data2_centroid = data2 - mean_data2
#z_avg_data3 = Average(data1_centroid[:,2])
#for i in range(0, len(x_data1)-1):

dist = np.linalg.norm(data1_centroid[1,:]-data2_centroid[1,:])
dist2 = np.linalg.norm(C-D)


# =============================================================================
# # =============================================================================
# for k in range(0,len(x_data1)):
#     for i in range(0,len(x_data1)):
#        # ist2 = np.linalg.norm(C-D)
#         ist2 = np.linalg.norm(data1_centroid[k,:]-data2_centroid[i,:])
#         E.append(ist2)
#     P.append(E.index(min(E))) 
#     E=[]
#  
# =============================================================================

    
    
    
    
    
    
    


 
