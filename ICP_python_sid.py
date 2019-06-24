# This file contains custom implementation for ICP algorithm
import Helper.helper_functions as helper
import Helper.data_loader as loader
import Helper.config as cfg


if cfg.USER is True:
    static_cloud = str(input('Enter the path to the static point cloud: '))
    moving_cloud = str(input('Enter the path to the moving point cloud: '))
else:
    static_cloud = 'Data/bun000.ply'
    moving_cloud = 'Data/bun090.ply'

# Visualizing the input clouds
if cfg.VISUAL is True:
    print("[INFO] Visualizing the static point cloud")
    loader.display_ugly(static_cloud)

    print("[INFO] Visualizing the moving point cloud")
    loader.display_ugly(moving_cloud)
else:
    print('[INFO] Data not visualized.')

static_numpy = loader.down_sampled_numpy(static_cloud)
moving_numpy = loader.down_sampled_numpy(moving_cloud)

point, minimum = helper.find_min_distance_point(moving_numpy[0], static_numpy)
print("Minimum distance is {}, point is {}".format(minimum, point))
