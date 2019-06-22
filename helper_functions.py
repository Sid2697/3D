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


def covariance(point_cloud_1, point_cloud_2):
    """
    This method provides column wise covariance between 2 point clouds matrices.
    :param point_cloud_1: first point cloud in form of numpy array
    :param point_cloud_2: second point cloud in from of numpy array
    :return: covariance of each column with respect to the same column in other point cloud
    """
    centroid_point_cloud_1, centroid_point_cloud_2 = centroid(point_cloud_1), centroid(point_cloud_2)
    min_len = len(point_cloud_1) if len(point_cloud_1) < len(point_cloud_2) else len(point_cloud_2)
    prod = 0
    for i in range(min_len):
        diff_1 = point_cloud_1[i] - centroid_point_cloud_1
        diff_2 = point_cloud_2[i] - centroid_point_cloud_2
        prod += diff_1 * diff_2
    return prod/(min_len - 1)
