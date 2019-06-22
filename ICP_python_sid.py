# This file contains custom implementation for ICP algorithm
import data_loader as loader

VISUAL = False
USER = False

# Loading two point clouds

if USER is True:
    static_cloud = str(input('Enter the path to the static point cloud: '))
    moving_cloud = str(input('Enter the path to the moving point cloud: '))
else:
    static_cloud = '/Volumes/Siddhant/3D_Data/bunnyfiles/bun000.ply'
    moving_cloud = '/Volumes/Siddhant/3D_Data/bunnyfiles/bun090.ply'

# Visualizing the input clouds
if VISUAL is True:
    print("[INFO] Visualizing the static point cloud")
    loader.display_ugly(static_cloud)

    print("[INFO] Visualizing the moving point cloud")
    loader.display_ugly(moving_cloud)
else:
    print('[INFO] Data not visualized.')

static_numpy = loader.data_numpy(static_cloud)
moving_numpy = loader.data_numpy(moving_cloud)


def centroid(point_cloud):
    """
    This method is used to calculate the centroid of a point cloud
    :param point_cloud: point cloud in form of numpy array
    :return: Centroid of the point cloud
    """
    summation = 0
    for i in range(len(point_cloud)):
        summation += point_cloud[i]
    return summation/len(point_cloud)


static_centroid = centroid(static_numpy)
moving_centroid = centroid(moving_numpy)

print("Difference: ", static_centroid - moving_centroid)

