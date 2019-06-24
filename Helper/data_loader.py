# This file is used to load and save the 3D data
from scipy.io import savemat
from tqdm import tqdm
import open3d as o3d
import numpy as np
import pptk


def display_ugly(file_path):
    """
    This method is used to visualize the data using open3d which results in an ugly display XD
    :param file_path: Path to the file to be visualized
    :return: None
    """
    print('[INFO] Loading the data for visualization.')
    point_cloud = o3d.io.read_point_cloud(file_path)
    o3d.visualization.draw_geometries([point_cloud])


def display_beauty(file_path):
    """
    This method is used to visualize the data using pptk which results in a beautiful display
    :param file_path: Path to the file to be visualized
    :return: None
    """
    point_cloud_numpy = data_numpy(file_path)
    pptk.viewer(point_cloud_numpy)


def data_numpy(file_path):
    """
    This method is used to convert the point cloud data into numpy array for further processing.
    :param file_path: Path to the file to be visualized
    :return: Numpy array of point cloud
    """
    print('[INFO] Loading the data for conversion to numpy.')
    point_cloud = o3d.io.read_point_cloud(file_path)
    point_cloud_numpy = np.asarray(point_cloud.points)
    return point_cloud_numpy


def down_sampled_numpy(cloud):
    """
    This method is used to get the down sampled point cloud
    :param cloud: Point cloud which is to be down sampled [type: point cloud]
    :return: down sampled point cloud [type: numpy asarry]
    """
    down = o3d.voxel_down_sample(o3d.io.read_point_cloud(cloud), voxel_size=0.009)
    return np.asarray(down.points)


def data_matlab(file_path, file_name):
    """
    This method is used to convert point cloud into the form which can be loaded into matlab
    :param file_path: Path to the file to be visualized
    :param file_name: Path to the file in which the data is to be saved
    :return: None
    """
    print('[INFO] Loading the data for conversion.')
    point_cloud_numpy = data_numpy(file_path)
    x, y, z = [], [], []
    for point in tqdm(range(len(point_cloud_numpy)), ascii=True, desc='Progress', ncols=100):
        x.append(point_cloud_numpy[point][0])
        y.append(point_cloud_numpy[point][1])
        z.append(point_cloud_numpy[point][2])
    savemat(file_name + '.mat', mdict={'x': x, 'y': y, 'z': z})
    print('[INFO] Matlab matrix saved with name {}.mat'.format(file_name))


if __name__ == '__main__':
    path = "C:\\Users\\Siddhant\\Desktop\\3D data\\Room_211.pts"
    data_matlab(path, 'sid_trial')
