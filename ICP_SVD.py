# This file contains code for implementing ICP using SVD
import Helper.helper_functions as help
import Helper.data_loader as loader
import Helper.config as cfg
import numpy as np
import pptk

max_iter = 100

if cfg.USER is True:
    static_cloud = str(input('Enter the path to the static point cloud: '))
    moving_cloud = str(input('Enter the path to the moving point cloud: '))
else:
    static_cloud = 'Data/bun000.ply'
    moving_cloud = 'Data/bun045.ply'

# original_static_numpy = loader.data_numpy(static_cloud)
# original_moving_numpy = loader.data_numpy(moving_cloud)

static_numpy = loader.down_sampled_numpy(static_cloud)
moving_numpy = loader.down_sampled_numpy(moving_cloud)

latest_moved = moving_numpy

for i in range(max_iter):
    static_centroid = help.centroid(static_numpy)
    moving_centroid = help.centroid(latest_moved)

    static_centered = static_numpy - static_centroid
    moving_centered = latest_moved - moving_centroid

    if len(moving_centered) > len(static_centered):
        moving_centered = moving_centered[:len(static_centered), :]
    else:
        static_centered = static_centered[:len(moving_centered), :]

    S = np.matmul(static_centered, np.transpose(moving_centered))

    U, Sym, V_T = np.linalg.svd(S)
    iden = np.identity(len(moving_centered))
    iden[-1, -1] = np.linalg.det(np.matmul(np.transpose(V_T), np.transpose(U)))

    R = np.matmul(np.matmul(np.transpose(V_T), iden), np.transpose(U))

    t = static_centered - np.matmul(R, moving_centered)

    latest_moved = np.matmul(R, moving_centered) + t

    if i % 10 == 0:
        display = np.concatenate(static_numpy, latest_moved)
        pptk.viewer(display)

