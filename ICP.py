# This file contains custom implementation for ICP algorithm
import Helper.helper_functions as helper
import Helper.data_loader as loader
import Helper.config as cfg
from tqdm import tqdm
import numpy as np
import pptk

max_iterations = 100
thresh = 0.000001

if cfg.USER is True:
    static_cloud = str(input('Enter the path to the static point cloud: '))
    moving_cloud = str(input('Enter the path to the moving point cloud: '))
else:
    static_cloud = 'Data/bun000.ply'
    moving_cloud = 'Data/bun045.ply'

# Visualizing the input clouds
if cfg.VISUAL is True:
    print("[INFO] Visualizing the static point cloud")
    loader.display_beauty(static_cloud)

    print("[INFO] Visualizing the moving point cloud")
    loader.display_beauty(moving_cloud)
else:
    print('[INFO] Data not visualized.')

original_static_numpy = loader.data_numpy(static_cloud)
original_moving_numpy = loader.data_numpy(moving_cloud)

static_numpy = loader.down_sampled_numpy(static_cloud)
moving_numpy = loader.down_sampled_numpy(moving_cloud)

newP = moving_numpy
Y_inp = np.zeros(moving_numpy.shape)

for loop in range(max_iterations):
    for sub_loop in tqdm(range(min(len(moving_numpy), len(static_numpy)))):
        Y_inp[sub_loop], _ = helper.find_min_distance_point(newP[sub_loop], static_numpy)
    [sid, R_, T_, error] = helper.find_alignment(Y_inp, newP)

    for lol in tqdm(range(min(len(moving_numpy), len(static_numpy)))):
        newP[lol] = np.matmul(R_, newP[lol]) + T_
        e = Y_inp[lol] - newP[lol]
        error += np.matmul(np.transpose(e), e)
    error = error/len(static_numpy)
    print("Error in loop {} is {}.".format(loop, error))
    if loop % 10 == 0:
        print('Visualizing the data!')
        for data in range(len(original_moving_numpy)):
            original_moving_numpy[data] = np.matmul(R_, original_moving_numpy[data]) + T_
        new = np.concatenate((static_numpy, newP))
        pptk.viewer(new)
    if error < thresh:
        break
