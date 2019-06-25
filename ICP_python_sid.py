# This file contains custom implementation for ICP algorithm
import Helper.helper_functions as helper
import Helper.data_loader as loader
import Helper.config as cfg
from scipy.io import savemat

from tqdm import tqdm
# import pptk
import numpy as np


if cfg.USER is True:
    static_cloud = str(input('Enter the path to the static point cloud: '))
    moving_cloud = str(input('Enter the path to the moving point cloud: '))
else:
    static_cloud = 'Data/bun045.ply'
    moving_cloud = 'Data/bun000.ply'

# Visualizing the input clouds
if cfg.VISUAL is True:
    print("[INFO] Visualizing the static point cloud")
    loader.display_beauty(static_cloud)

    print("[INFO] Visualizing the moving point cloud")
    loader.display_beauty(moving_cloud)
else:
    print('[INFO] Data not visualized.')

static_numpy = loader.down_sampled_numpy(static_cloud)
moving_numpy = loader.down_sampled_numpy(moving_cloud)
# v1 = pptk.viewer(static_numpy)
# v1.attributes(static_numpy + 100)

correspondence_dict = {}

for i in tqdm(range(len(moving_numpy))):
    point, minimum = helper.find_min_distance_point(moving_numpy[i], static_numpy)
    if 'Q' in correspondence_dict.keys():
        correspondence_dict['Q'].append(point)
    else:
        correspondence_dict['Q'] = [point]
    if 'p' in correspondence_dict.keys():
        correspondence_dict['p'].append(moving_numpy[i])
    else:
        correspondence_dict['p'] = [moving_numpy[i]]

mu_y = helper.centroid(correspondence_dict['Q'])
mu_p = helper.centroid(correspondence_dict['p'])

y_i_dash = correspondence_dict['Q'] - mu_y
p_i_dash = correspondence_dict['p'] - mu_p


S_xx = helper.dot_product(p_i_dash, y_i_dash, 0, 0)
S_xy = helper.dot_product(p_i_dash, y_i_dash, 0, 1)
S_xz = helper.dot_product(p_i_dash, y_i_dash, 0, 2)
S_yx = helper.dot_product(p_i_dash, y_i_dash, 1, 0)
S_yy = helper.dot_product(p_i_dash, y_i_dash, 1, 1)
S_yz = helper.dot_product(p_i_dash, y_i_dash, 1, 2)
S_zx = helper.dot_product(p_i_dash, y_i_dash, 2, 0)
S_zy = helper.dot_product(p_i_dash, y_i_dash, 2, 1)
S_zz = helper.dot_product(p_i_dash, y_i_dash, 2, 2)

N = np.array([[S_xx + S_yy + S_zz, S_yz - S_zy, - S_xz + S_zx, S_xy - S_yz],
              [S_yz - S_zy, S_xx - S_zz - S_yy, S_xy + S_yx, S_xz + S_zx],
              [-S_xz + S_zx, S_xy + S_yx, S_yy - S_zz - S_xx, S_yz + S_zy],
              [S_xy - S_yz, S_xz + S_zx, S_yz + S_zy, S_zz - S_yy - S_xx]])

w, v = np.linalg.eig(N)
q = v[:, 0]

q0, q1, q2, q3 = q[0], q[1], q[2], q[3]

Q_bar = np.array([[q0, -q1, -q2, -q3],
                  [q1, q0, q3, -q2],
                  [q2, -q3, q0, q1],
                  [q3, q2, -q1, q0]])

Q = np.array([[q0, -q1, -q2, -q3],
              [q1, q0, -q3, q2],
              [q2, q3, q0, -q1],
              [q3, -q2, q1, q0]])

R = np.matmul(np.transpose(Q_bar), Q)
R = R[1:, 1:]
print('Rotation matrix is: \n{}'.format(R))

