# This file contains functions for supporting ICP
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


def inner_product(vector_1, vector_2):
    """
    This method takes in 2 vectors and calculates the dot product
    :param vector_1: First vector whose dot product is to be calculated
    :param vector_2: Second vector whose dot product is to be calculated
    :return: dot product between vector_1 and vector_2
    """
    vector_1_transpose = np.transpose(vector_1)
    return np.dot(vector_1_transpose, vector_2)


def euclidean_distance(point_1, point_2):
    """
    This method is used to calculate Euclidean distance between 2 3D points
    :param point_1: First point from whom distance is to be calculated
    :param point_2: Second point till which the distance is to be calculated
    :return: Distance between point_1 and point_2
    """
    diff = point_1 - point_2
    squared = np.square(diff)
    summed = np.sum(squared)
    return np.sqrt(summed)


def find_min_distance_point(moving_point, static_numpy):
    """
    This method takes finds the corresponding point for a moving point cloud in static point cloud
    :param moving_point: point from moving cloud whose correspondence is to be found in static point cloud
    :param static_numpy: point cloud in which corresponding point is to be calculated
    :return: point with the min distance and distance between them
    """
    minimum, distance = np.inf, np.inf
    point = 0
    for i in range(len(static_numpy)):
        distance = euclidean_distance(static_numpy[i], moving_point)
        if distance < minimum:
            minimum = distance
            point = static_numpy[i]
    return point, distance


def dot_product(p_i_dash, y_i_dash, a, b):
    dot_prod = np.dot(p_i_dash[:, a], y_i_dash[:, b])
    return dot_prod


def find_alignment(Y, latest_moving_numpy):
    mu_y = centroid(Y)
    mu_p = centroid(latest_moving_numpy)

    y_i_dash = Y - mu_y
    p_i_dash = latest_moving_numpy - mu_p

    S_xx = dot_product(p_i_dash, y_i_dash, 0, 0)
    S_xy = dot_product(p_i_dash, y_i_dash, 0, 1)
    S_xz = dot_product(p_i_dash, y_i_dash, 0, 2)
    S_yx = dot_product(p_i_dash, y_i_dash, 1, 0)
    S_yy = dot_product(p_i_dash, y_i_dash, 1, 1)
    S_yz = dot_product(p_i_dash, y_i_dash, 1, 2)
    S_zx = dot_product(p_i_dash, y_i_dash, 2, 0)
    S_zy = dot_product(p_i_dash, y_i_dash, 2, 1)
    S_zz = dot_product(p_i_dash, y_i_dash, 2, 2)

    N = np.array([[S_xx + S_yy + S_zz, S_yz - S_zy, - S_xz + S_zx, S_xy - S_yx],
                  [S_yz - S_zy, S_xx - S_zz - S_yy, S_xy + S_yx, S_xz + S_zx],
                  [-S_xz + S_zx, S_xy + S_yx, S_yy - S_zz - S_xx, S_yz + S_zy],
                  [S_xy - S_yx, S_xz + S_zx, S_yz + S_zy, S_zz - S_yy - S_xx]])

    n = np.linalg.eigh(N)
    Eigen_values = n[0]
    Eigen_vector = n[1]

    m = []
    for k in range(4):
        m.append(Eigen_values[k])

    hj = m.index(max(m))
    q = Eigen_vector[:, hj]

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
    Sp, D = 0, 0

    for num in range(len(y_i_dash)):
        D += np.matmul(np.transpose(y_i_dash[num]), y_i_dash[num])
        Sp += np.matmul(np.transpose(p_i_dash[num]), p_i_dash[num])
    Dtry = np.matmul(np.transpose(q), N)
    Dnew = np.matmul(Dtry, q)
    s = np.sqrt(D / Sp)
    t = mu_y - s * np.matmul(R, mu_p)

    err = 0

    for haha in range(len(y_i_dash)):
        d = Y[haha] - (s * np.matmul(R, latest_moving_numpy[haha]) + t)
        err += np.matmul(np.transpose(d), d)
    return [s, R, t, err]
