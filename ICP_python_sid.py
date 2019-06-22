# This file contains custom implementation for ICP algorithm
import helper_functions as helper
import data_loader as loader
import config as cfg

# Loading two point clouds

if cfg.USER is True:
    static_cloud = str(input('Enter the path to the static point cloud: '))
    moving_cloud = str(input('Enter the path to the moving point cloud: '))
else:
    static_cloud = '/Volumes/Siddhant/3D_Data/bunnyfiles/bun000.ply'
    moving_cloud = '/Volumes/Siddhant/3D_Data/bunnyfiles/bun090.ply'

# Visualizing the input clouds
if cfg.VISUAL is True:
    print("[INFO] Visualizing the static point cloud")
    loader.display_ugly(static_cloud)

    print("[INFO] Visualizing the moving point cloud")
    loader.display_ugly(moving_cloud)
else:
    print('[INFO] Data not visualized.')

static_numpy = loader.data_numpy(static_cloud)
moving_numpy = loader.data_numpy(moving_cloud)

static_centroid = helper.centroid(static_numpy)
moving_centroid = helper.centroid(moving_numpy)

# print("Difference: ", static_centroid - moving_centroid)

static_variance = helper.variance(static_numpy)
moving_variance = helper.variance(moving_numpy)

print('Static centroid', static_centroid, 'Moving centroid', moving_centroid)
print("Static Variance ", static_variance,  "Moving Variance ", moving_variance)
