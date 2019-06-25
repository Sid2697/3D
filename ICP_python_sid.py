# This file contains custom implementation for ICP algorithm
import Helper.helper_functions as helper
import Helper.data_loader as loader
import Helper.config as cfg

from tqdm import tqdm
import pptk
import numpy as np

Q=[]
# =============================================================================

# =============================================================================

if cfg.USER is True:
    static_cloud = str(input('Enter the path to the static point cloud: '))
    moving_cloud = str(input('Enter the path to the moving point cloud: '))
else:
    static_cloud = 'Data/bun045.ply'
    moving_cloud = 'Data/bun000.ply'

# Visualizing the input clouds
if cfg.VISUAL is False:
    print("[INFO] Visualizing the static point cloud")
    loader.display_beauty(static_cloud)

    print("[INFO] Visualizing the moving point cloud")
    loader.display_beauty(moving_cloud)
else:
    print('[INFO] Data not visualized.')

static_numpy = loader.down_sampled_numpy(static_cloud)
moving_numpy = loader.down_sampled_numpy(moving_cloud)
# =============================================================================
# v1 = pptk.viewer(static_numpy)
# v1.attributes(static_numpy + 100)
# =============================================================================

corresponding_points_dict = {}

for i in tqdm(range(len(moving_numpy))):
    point, minimum = helper.find_min_distance_point(moving_numpy[i], static_numpy)
    corresponding_points_dict[i] = [point, moving_numpy[i], minimum]
    Q.append(point)
    
y_i = np.float64(Q)   
# (corresponding points)

p_i = moving_numpy  
# =============================================================================
# A = np.array([1, 2, 3,8])
# B = np.array([9, 3, 1,2])
# =============================================================================
mu_y = helper.centroid(y_i)   
mu_p = helper.centroid(moving_numpy) 

y_i_dash = y_i - mu_y
p_i_dash = p_i - mu_p


# =============================================================================
# d = np.dot(A,B)
# =============================================================================
S_xx = helper.dot_product(p_i_dash,y_i_dash,0,0)
S_xy = helper.dot_product(p_i_dash,y_i_dash,0,1)
S_xz = helper.dot_product(p_i_dash,y_i_dash,0,2)
S_yx = helper.dot_product(p_i_dash,y_i_dash,1,0)
S_yy = helper.dot_product(p_i_dash,y_i_dash,1,1)
S_yz = helper.dot_product(p_i_dash,y_i_dash,1,2)
S_zx = helper.dot_product(p_i_dash,y_i_dash,2,0)
S_zy = helper.dot_product(p_i_dash,y_i_dash,2,1)
S_zz = helper.dot_product(p_i_dash,y_i_dash,2,2)

N = np.array([[S_xx +S_yy+S_zz, S_yz-S_zy, -S_xz+S_zx, S_xy-S_yz],
              [ S_yz-S_zy, S_xx -S_zz-S_yy, S_xy+S_yx, S_xz+S_zx],
              [-S_xz+S_zx, S_xy+S_yx, S_yy-S_zz-S_xx, S_yz+S_zy],
              [S_xy-S_yz, S_xz+S_zx, S_yz+S_zy, S_zz-S_yy-S_xx]])
n = np.linalg.eigh(N)
Eigen_values = n[0]
Eigen_vector = n[1]
#max_Eigen_value = Eigen_values.index(max(Eigen_values))
m=[]
for k in range (4):
    m.append(Eigen_values[k])

j = m.index(max(m))      
q = Eigen_vector[:,j]

q0 = q[0]
qx = q[1]
qy = q[2]
qz = q[3]

Qbar = np.array([[q0,-qx,-qy, -qz],
                 [qx, q0, qz, -qy],
                 [qy, -qz, q0, qx],
                 [qz, -qy, qx, q0]])
    
Q = np.array([[q0,-qx,-qy, -qz],
              [qx, q0, -qz, qy],
              [qy, qz, q0, -qx],
              [qz, -qy, qx, q0]])    

R = np.matmul(np.transpose(Qbar), Q)
R = R[1:, 1:]
print('Rotation matrix is: \n{}'.format(R))    
    
