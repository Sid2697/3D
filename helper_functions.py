# This file contains helper functions for supporting ICP
import numpy as np


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


def variance(point_cloud):
    """
    This method is used to calculate the variance of a point cloud
    :param point_cloud: point cloud in form of numpy array
    :return: Variance of the point cloud
    """
    centroid_point_cloud = centroid(point_cloud)
    diff = 0
    for i in range(len(point_cloud)):
        diff += np.square((point_cloud[i] - centroid_point_cloud))
    return diff / (len(point_cloud) - 1)
