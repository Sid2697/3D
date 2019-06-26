# This file contains custom implementation for ICP algorithm
import Helper.helper_functions as helper
import Helper.data_loader as loader
import Helper.config as cfg
import numpy as np

max_iterations = 100
thresh = 0.000001

if cfg.USER is True:
    static_cloud = str(input('Enter the path to the static point cloud: '))
    moving_cloud = str(input('Enter the path to the moving point cloud: '))
else:
    static_cloud = 'Data/bun180.ply'
    moving_cloud = 'Data/bun090.ply'

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
#file1 = open("yFile.txt","w") 
#for i in range (len(static_numpy)):
#    file1.writelines(str(static_numpy [i]))
#file1.close() 

newP = moving_numpy
Y_inp = np.zeros(moving_numpy.shape)

for loop in range(max_iterations):
    for sub_loop in range(min (len(moving_numpy),len(static_numpy))):
        Y_inp[sub_loop], _ = helper.find_min_distance_point(newP[sub_loop], static_numpy)
    [s_, R_, T_, error] = helper.find_alignment(Y_inp, newP)

    for lol in range(min (len(moving_numpy),len(static_numpy))):
        newP[lol] = s_ * np.matmul(R_, newP[lol]) + T_
        e = Y_inp[lol] - newP[lol]
        error += np.matmul(np.transpose(e), e)
    error = error/len(static_numpy)
    print("Error in loop {} is {}.".format(loop, error))
    if error < thresh:
        break
