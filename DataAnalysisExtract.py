"""
@file:DataAnalysisExtract.py
@author: Application Engineer
@Date:2023/5/10
@mail:Application@cn.inno.com
"""
import math
import os
from multiprocessing import *
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rosbag
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
import ExcelOperation
import pypcd
from pathlib import Path
from collections import namedtuple

_DATATYPES = {}
_DATATYPES[PointField.INT8] = ('b', 1)
_DATATYPES[PointField.UINT8] = ('B', 1)
_DATATYPES[PointField.INT16] = ('h', 2)
_DATATYPES[PointField.UINT16] = ('H', 2)
_DATATYPES[PointField.INT32] = ('i', 4)
_DATATYPES[PointField.UINT32] = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)
result = []

# Use a namedtuple to store the constants use in Extract class
Constants = namedtuple('Constants', ['x', 'y', 'z', 'f', 'I', 'intensity',
                                     'frame_count', 'fields_wanted',
                                     'topics', 'case'])

# Define the constants as a global variable
CONSTANTS = Constants(x=3, y=4, z=5, f=7, I=0, intensity=6,
                      frame_count=0,
                      fields_wanted=['flags', 'flag', 'scan_id',
                                     'scanline', 'scan_idx',
                                     'x', 'y', 'z',
                                     'intensity', 'reflectance',
                                     'frame_idx', 'frame_id',
                                     'elongation', 'is_2nd_return',
                                     'multi_return', 'timestamp',
                                     'channel', 'roi', 'facet',
                                     'confid_level'],
                      topics=['/iv_points', '/AT128/pandar_points',
                              '/rslidar_points', 'iv_points'],
                      case=["FOVROI", "POD", "PointsNum", "MeanItensity", "Precision"])


class Extract:

    def __init__(self):
        # Use the constants from the namedtuple instead of defining them again
        self.x = CONSTANTS.x
        self.y = CONSTANTS.y
        self.z = CONSTANTS.z
        self.f = CONSTANTS.f
        self.I = CONSTANTS.I
        self.intensity = CONSTANTS.intensity
        self.frame_count = CONSTANTS.frame_count
        self.fields_wanted = CONSTANTS.fields_wanted
        # Use np.zeros_like instead of np.zeros to avoid specifying the shape and dtype explicitly
        self.index_sort = np.zeros_like(self.fields_wanted)
        self.topics = CONSTANTS.topics
        self.case = CONSTANTS.case
        self.topic = ''

    def get_pointcloud2(self, msg):
        # Use keyword arguments only for clarity and readability
        ps = PointCloud2(header=msg.header,
                         height=1,
                         width=msg.row_step // msg.point_step,  # Use integer division to avoid floating point errors
                         is_dense=False,
                         is_bigendian=msg.is_bigendian,
                         fields=msg.fields,
                         point_step=msg.point_step,
                         row_step=msg.row_step,
                         data=msg.data)
        return ps

    def get_struct_fmt_map(self, is_bigendian, fields):
        """
        Get PointField from bag message using tools in ros
        """
        result = []
        fmt_pre = '>' if is_bigendian else '<'
        for field in sorted(fields, key=(lambda f: f.offset)):
            if field.datatype not in _DATATYPES:
                print('Skipping unknown PointField datatype [{}]'.format(field.datatype))
                exit(-1)
            else:
                datatype_fmt, datatype_length = _DATATYPES[field.datatype]
                result.append((fmt_pre + datatype_fmt, field.offset, field.name))

        result.sort(key=(lambda tup: tup[2]))
        return result

    def get_ndarray_from_msg(self, msg, topic, frame_idx):
        """
        Get point cloud data from bag message using tools in ros
        """
        fields = self.get_struct_fmt_map(msg.is_bigendian, msg.fields)
        arrays = []
        frame_col_name = 'frame_idx'
        if frame_idx != None:
            fields = [(None, None, frame_col_name)] + fields
        num_points = msg.row_step / msg.point_step
        if topic == '/rslidar_points':
            num_points = 78750
        for f in fields:
            if f[2] != frame_col_name:
                if num_points > 0:
                    arrays.append(np.ndarray(int(num_points), f[0], msg.data, f[1], msg.point_step))
                else:
                    arrays.append(np.ndarray([0]))
            else:
                arrays.append(np.full(int(num_points), frame_idx, dtype=np.int32))

        arrays2 = np.swapaxes(arrays, 0, 1)
        return num_points, [f[2] for f in fields], arrays2

    # def get_ndarray_from_msg(self, msg, topic, frame_idx):
    #     """
    #     Get point cloud data from bag message using tools in ros
    #     """
    #
    #     # Use a list comprehension to get the fields
    #     fields = [self.get_struct_fmt_map(msg.is_bigendian, f) for f in msg.fields]
    #
    #     # Use a ternary operator to assign the frame_col_name
    #     frame_col_name = 'frame_idx' if frame_idx is not None else None
    #
    #     # Use integer division to avoid floating point errors
    #     num_points = msg.row_step // msg.point_step
    #
    #     # Use a dictionary to store the topic-specific values
    #     topic_values = {'/rslidar_points': 78750}
    #
    #     # Use the get method to assign the num_points based on the topic
    #     num_points = topic_values.get(topic, num_points)
    #
    #     # Use a list comprehension to create the arrays
    #     arrays = [
    #         np.ndarray(int(num_points), f[0], msg.data, f[1], msg.point_step) if f[2] != frame_col_name and num_points > 0 else np.full(int(num_points), frame_idx, dtype=np.int32) for f in fields]
    #
    #     # Use np.transpose instead of np.swapaxes for readability and simplicity[^1^][1]
    #     arrays2 = np.transpose(arrays)
    #
    #     return num_points, [f[2] for f in fields], arrays2

    # def get_bag_data(self, file_path, topic, FrameLimit):
    #     """
    #     Get point cloud data from bag
    #     """
    #     self.frame_count = 0
    #     array0 = np.array(0)
    #     points = []
    #     info = rosbag.Bag(file_path).get_message_count(topic)
    #     try:
    #         if info == 0 and topic == '/iv_points':
    #             topic = 'iv_points'
    #         bag_data = rosbag.Bag(file_path).read_messages(topic)
    #         if topic == '/rslidar_points':
    #             for topic, msg, t in bag_data:
    #                 arrays = list(point_cloud2.read_points(msg))
    #                 for i in range(len(arrays)):
    #                     points.append(list(arrays[i]) + [self.frame_count])
    #
    #                 while self.frame_count == 0:
    #                     fields = msg.fields
    #                     array0 = np.zeros(len(fields) + 1)
    #                     break
    #
    #                 self.frame_count += 1
    #                 array0 = np.vstack((array0, np.array(points)))
    #                 points.clear()
    #                 if self.frame_count == FrameLimit[1] + 2:
    #                     array0 = array0[1:]
    #                     fields = ['x', 'y', 'z', 'intensity', 'frame_idx']
    #                     return array0, fields
    #
    #         else:
    #             for topic, msg, t in bag_data:
    #                 pt_num, fields1, arrays = self.get_ndarray_from_msg(msg, topic, self.frame_count)
    #                 while self.frame_count == 0:
    #                     fields = fields1
    #                     array0 = np.zeros(len(fields))
    #                     break
    #
    #                 self.frame_count += 1
    #                 array0 = np.vstack((array0, arrays))
    #                 if self.frame_count == FrameLimit[1] + 2:
    #                     array0 = array0[1:]
    #                     return array0, fields
    #
    #             array0 = array0[1:]
    #             return array0, fields
    #     except Exception as e:
    #         try:
    #             print(e)
    #         finally:
    #             e = None
    #             del e

    def get_bag_data(self, file_path, topic, FrameLimit):
        """
        Get point cloud data from bag
        """

        # Initialize the frame count and the output array
        self.frame_count = 0
        array0 = None

        # Use a context manager to open and close the bag file automatically
        with rosbag.Bag(file_path) as bag:

            # Check if the topic exists in the bag file
            info = bag.get_message_count(topic)
            if info == 0 and topic == '/iv_points':
                topic = 'iv_points'

            # Iterate over the messages in the topic
            for topic, msg, t in bag.read_messages(topics=[topic]):

                # Handle different topics separately
                if topic == '/rslidar_points':

                    # Convert the message to a list of arrays
                    arrays = list(point_cloud2.read_points(msg))

                    # Add the frame count to each array and stack them vertically
                    arrays = np.hstack((arrays, np.full((len(arrays), 1), self.frame_count)))

                    # Initialize the output array with the first message fields and data
                    if self.frame_count == 0:
                        fields = msg.fields + ['frame_idx']
                        array0 = arrays

                    # Append the subsequent messages data to the output array
                    else:
                        array0 = np.vstack((array0, arrays))

                else:

                    # Get the number of points, fields and data from the message
                    pt_num, fields1, arrays = self.get_ndarray_from_msg(msg, topic, self.frame_count)

                    # Initialize the output array with the first message fields and data
                    if self.frame_count == 0:
                        fields = fields1
                        array0 = arrays

                    # Append the subsequent messages data to the output array
                    else:
                        array0 = np.vstack((array0, arrays))

                # Increment the frame count by one
                self.frame_count += 1

                # Check if the frame limit is reached and return the output array and fields
                if self.frame_count == FrameLimit[1] + 2:
                    return array0, fields

        # Return the output array and fields after iterating over all messages
        return array0, fields

    # def get_pcd_data(self, file_path):
    #     """
    #     Get point cloud data from pcd
    #     """
    #     pts = []
    #     data = open(file_path, mode='r').readlines()
    #     pts_num = eval(data[8].split(' ')[-1])
    #     fields = data[1].strip('\nFIELDS ').split(' ')
    #     for line in data[10:]:
    #         p = line.strip('\n').split(' ')
    #         pts.append(p)
    #
    #     assert len(pts) == pts_num
    #     res = np.zeros((pts_num, len(pts[0])), dtype=float)
    #     for i in range(pts_num):
    #         res[i] = pts[i]
    #
    #     return res, fields

    def get_pcd_data(self, file_path):
        """
        Get point cloud data from pcd
        """

        # Use a context manager to open and close the file automatically
        with open(file_path, mode='r') as f:

            # Read the header lines and get the number of points and fields
            header = []
            while True:
                line = f.readline().strip()
                header.append(line)
                if line.startswith('DATA'):
                    break
            pts_num = int(header[8].split()[-1])
            fields = header[1].split()[1:]

            # Read the data lines and convert them to a numpy array
            data = f.readlines()
            data = [line.strip().split() for line in data]
            data = np.array(data, dtype=float)

            # Check if the number of points matches the header
            assert len(data) == pts_num

            # Return the data array and fields
            return data, fields

    # def get_pcap_data(self, file_path):
    #     """
    #     Get point cloud data from pcap
    #     """
    #     dir = os.path.dirname(file_path)
    #     for filename in os.listdir(dir):
    #         if '.pcd' in filename:
    #             print(dir + '/' + filename)
    #             arrays, fields = self.get_pcd_data(dir + '/' + filename)
    #             array = np.vstack((array, arrays))
    #
    #     return array, fields

    def get_pcap_data(self, file_path):
        """
        Get point cloud data from pcap
        """

        # Get the directory of the file path
        dir = Path(file_path).parent

        # Initialize the output array and fields
        array = None
        fields = None

        # Iterate over the files in the directory that have .pcd extension
        for filename in dir.glob('*.pcd'):

            # Print the file name
            print(filename)

            # Get the data and fields from the pcd file
            arrays, fields = self.get_pcd_data(filename)

            # Initialize the output array with the first file data
            if array is None:
                array = arrays

            # Append the subsequent files data to the output array
            else:
                array = np.vstack((array, arrays))

        # Return the output array and fields
        return array, fields

    # def get_file_data(self, file_path, topic, FrameLimit):
    #     """
    #     Get point cloud data from file; four file formats are currently supported
    #     """
    #     fields = ''
    #     if '.pcd' in file_path:
    #         res, fields = self.get_pcd_data(file_path)
    #     elif '.csv' in file_path:
    #         res = pd.read_csv(file_path)
    #         fields = list(res.columns.values)
    #         res = res.values
    #     elif '.bag' in file_path:
    #         res, fields = self.get_bag_data(file_path, topic, FrameLimit)
    #     elif '.pcap' in file_path:
    #         res, fields = self.get_pcap_data(file_path)
    #     else:
    #         print('Get data from file failed')
    #
    #     for i in range(len(self.fields_wanted)):  # Make the data order conform to 'fields_wanted'
    #         for j in range(len(fields)):
    #             if self.fields_wanted[i] == fields[j]:
    #                 self.index_sort[i] = j
    #                 break
    #             else:
    #                 self.index_sort[i] = -1
    #     j = 0
    #     new_sort = np.zeros(len(fields), dtype=int)
    #     for i in range(len(self.index_sort)):
    #         if self.index_sort[i] != -1:
    #             new_sort[j] = self.index_sort[i]
    #             j += 1
    #     sorted_fields = list(range(len(new_sort)))
    #     for i in range(len(new_sort)):
    #         sorted_fields[i] = fields[new_sort[i]]
    #     res = res[:, new_sort]
    #     return res, sorted_fields

    def get_file_data(self, file_path, topic, FrameLimit):
        """
        Get point cloud data from file; four file formats are currently supported
        """

        # Get the suffix of the file path
        suffix = Path(file_path).suffix

        # Use a match-case statement to handle different file formats[^1^][2]
        match suffix:
            case ".pcd":
                res, fields = self.get_pcd_data(file_path)
            case ".csv":
                res = pd.read_csv(file_path)
                fields = list(res.columns.values)
                res = res.values
            case ".bag":
                res, fields = self.get_bag_data(file_path, topic, FrameLimit)
            case ".pcap":
                res, fields = self.get_pcap_data(file_path)
            case _:
                print('Get data from file failed')
                return None, None

        # Use a list comprehension to get the index sort based on the fields wanted
        self.index_sort = [fields.index(f) if f in fields else -1 for f in self.fields_wanted]

        # Use a list comprehension to get the new sort by filtering out the -1 values
        new_sort = [i for i in self.index_sort if i != -1]

        # Use a list comprehension to get the sorted fields based on the new sort
        sorted_fields = [fields[i] for i in new_sort]

        # Use numpy indexing to get the sorted data based on the new sort
        res = res[:, new_sort]

        return res, sorted_fields

    # def get_fold_files(self, path):
    #     """
    #     Determine whether it is a calibration bag file.
    #     """
    #     count = 0
    #     result = []
    #     count1 = 0
    #     result1 = []
    #     for root, dirs, files in os.walk(path):
    #         for filename in sorted(files):
    #             if 'Cali' in filename:
    #                 if '.bag' in filename:
    #                     result.append(filename)
    #                     count += 1
    #             if '.bag' in filename:
    #                 result1.append(filename)
    #                 count1 += 1
    #
    #     if count:
    #         print('find %d Calibration bag files' % count)
    #         print('using', ''.join(result[-1]))
    #     if count1:
    #         print('find %d normal bag files' % count1)
    #         print('using', ''.join(result1[-1]))
    #     if count1:
    #         return result1
    #     return None, None

    def get_fold_files(self, path):
        """
        Determine whether it is a calibration bag file.
        """

        # Initialize the result lists and counts
        cali_files = []
        cali_count = 0
        normal_files = []
        normal_count = 0

        # Iterate over the files in the path that have .bag extension
        for filename in Path(path).glob('*.bag'):

            # Check if the file name contains 'Cali'
            if 'Cali' in filename.name:
                # Append the file name to the calibration list and increment the count
                cali_files.append(filename.name)
                cali_count += 1

            # Append the file name to the normal list and increment the count
            normal_files.append(filename.name)
            normal_count += 1

        # Print the results
        if cali_count:
            print(f'find {cali_count} Calibration bag files')
            print(f'using {cali_files[-1]}')

        if normal_count:
            print(f'find {normal_count} normal bag files')
            print(f'using {normal_files[-1]}')

        # Return the normal list or None if empty
        return normal_files or None


class Analysis:

    def __init__(self):
        self.extract = Extract()
        # self.q = Queue()

    # def extract_point_fitting_plane(self, arrays):
    #     if self.extract.topic != '/iv_points' and self.extract.topic != 'iv_points':
    #         z = arrays[:, 0]
    #         y = arrays[:, 1]
    #         x = arrays[:, 2]
    #     else:
    #         x = arrays[:, 3]
    #         y = arrays[:, 4]
    #         z = arrays[:, 5]
    #     A = np.zeros((3, 3))
    #     for i in range(0, len(arrays[:, 3])):
    #         A[(0, 0)] = A[(0, 0)] + x[i] ** 2
    #         A[(0, 1)] = A[(0, 1)] + x[i] * y[i]
    #         A[(0, 2)] = A[(0, 2)] + x[i]
    #         A[(1, 0)] = A[(0, 1)]
    #         A[(1, 1)] = A[(1, 1)] + y[i] ** 2
    #         A[(1, 2)] = A[(1, 2)] + y[i]
    #         A[(2, 0)] = A[(0, 2)]
    #         A[(2, 1)] = A[(1, 2)]
    #         A[(2, 2)] = len(arrays[:, 3])
    #
    #     B = np.zeros((3, 1))
    #     for i in range(0, len(arrays[:, 3])):
    #         B[(0, 0)] = B[(0, 0)] + x[i] * z[i]
    #         B[(1, 0)] = B[(1, 0)] + y[i] * z[i]
    #         B[(2, 0)] = B[(2, 0)] + z[i]
    #
    #     A_inv = np.linalg.inv(A)
    #     X = np.dot(A_inv, B)
    #     print('result：z = %.3f * x + %.3f * y + %.3f' % (X[(0, 0)], X[(1, 0)], X[(2, 0)]))
    #     variance = 0
    #     for i in range(len(arrays[:, 3])):
    #         variance = variance + (X[(0, 0)] * x[i] + X[(1, 0)] * y[i] + X[(2, 0)] - z[i]) ** 2
    #
    #     print('standard deviation:{}'.format(math.sqrt(variance / len(arrays[:, 3])) * 100))
    #     ax = plt.axes(projection='3d')
    #     ax.scatter3D(z, y, x, cmap='b', c='r')
    #     ax.set_xlabel('X Label')
    #     ax.set_ylabel('Y Label')
    #     ax.set_zlabel('Z Label')
    #     x1 = x
    #     y1 = y
    #     x1, y1 = np.meshgrid(x1, y1)
    #     z1 = X[(0, 0)] * x + X[(1, 0)] * y1 + X[(2, 0)]
    #     ax.plot_wireframe(z1, y1, x1, rstride=1, cstride=1)
    #     plt.xlim((min(z) - 0.3, max(z) + 0.3))
    #     plt.title('point extract')
    #     # plt.show()
    #     return ('{:.3f}'.format(X[(0, 0)]), '{:.3f}'.format(X[(1, 0)]), '{:.3f}'.format(X[(2, 0)]),
    #             '{:.3f}'.format(math.sqrt(variance / len(arrays[:, 3])) * 100))

    def extract_point_fitting_plane(self, arrays):
        # This function fits a plane from a set of 3D points and returns the equation and standard deviation
        # of the plane
        # Input: arrays - a numpy array of shape (n, 6) or (n, 3), where n is the number of points
        # Output: a tuple of four strings, representing the coefficients of x, y and z in the plane equation
        # and the standard deviation
        if self.extract.topic in ['/iv_points', 'iv_points']:
            x = arrays[:, 3]
            y = arrays[:, 4]
            z = arrays[:, 5]
        else:
            z = arrays[:, 0]
            y = arrays[:, 1]
            x = arrays[:, 2]
        A = np.array([[np.sum(x ** 2), np.sum(x * y), np.sum(x)], [np.sum(x * y), np.sum(y ** 2), np.sum(y)],
                      [np.sum(x), np.sum(y), len(x)]])
        B = np.array([[np.sum(x * z)], [np.sum(y * z)], [np.sum(z)]])
        X = np.linalg.solve(A, B)
        print('result：z = %.3f * x + %.3f * y + %.3f' % (X[0], X[1], X[2]))
        variance = np.mean((X[0] * x + X[1] * y + X[2] - z) ** 2)
        print('standard deviation:{}'.format(math.sqrt(variance) * 100))
        ax = plt.axes(projection='3d')
        ax.scatter3D(z, y, x, cmap='b', c='r')
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')
        x1 = x
        y1 = y
        x1, y1 = np.meshgrid(x1, y1)
        z1 = X[0] * x + X[1] * y1 + X[2]
        ax.plot_wireframe(z1, y1, x1, rstride=1, cstride=1)
        plt.xlim((min(z) - 0.3, max(z) + 0.3))
        plt.title('point extract')
        plt.show()
        return ('{:.3f}'.format(X[(0, 0)]), '{:.3f}'.format(X[(1, 0)]), '{:.3f}'.format(X[(2, 0)]),
                '{:.3f}'.format(math.sqrt(variance / len(arrays[:, 3])) * 100))

    # def point_number_of_FOV_per_frame(self, arrays):
    #     x = range(len(arrays))
    #     y = arrays
    #     plt.plot(x, y)
    #     plt.xlabel('X')
    #     plt.ylabel('Y')
    #     plt.title('point number of FOV')
    #     plt.show()
    #
    # def point_number_of_target_per_frame(self, arrays):
    #     x = range(len(arrays))
    #     y = arrays
    #     plt.plot(x, y)
    #     plt.xlabel('X')
    #     plt.ylabel('Y')
    #     plt.title('point number of target')
    #     plt.show()
    #
    # def mean_intensity_per_frame(self, arrays):
    #     x = range(len(arrays))
    #     y = arrays
    #     plt.plot(x, y)
    #     plt.xlabel('X')
    #     plt.ylabel('Y')
    #     plt.title('intensity of target')
    #     plt.show()

    def plot_per_frame(self, arrays, title):
        # This function plots a line chart of an array of values per frame
        # Input: arrays - a numpy array of shape (n,) or (n, 1), where n is the number of frames
        #        title - a string for the title of the chart
        # Output: None
        x = range(len(arrays))
        y = arrays
        plt.plot(x, y)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title(title)
        plt.show()

    # def point_of_target_show(self, arrays):
    #     z = arrays[:, self.extract.x]
    #     y = arrays[:, self.extract.y]
    #     x = arrays[:, self.extract.z]
    #     ax = plt.axes(projection='3d')
    #     ax.scatter3D(x, y, z, cmap='b', c='b')
    #     ax.set_xlabel('X Label')
    #     ax.set_ylabel('Y Label')
    #     ax.set_zlabel('Z Label')
    #     ax.set_zlim((min(z) - 0.2, max(z) + 0.2))
    #     plt.title('point extract')
    #     plt.xlim((min(x) - 0.2, max(x) + 0.2))
    #     plt.show()

    def plot_3d_points(self, arrays):
        # This function plots a 3D scatter plot of a set of points
        # Input: arrays - a numpy array of shape (n, 6) or (n, 3), where n is the number of points
        # Output: None
        x = arrays[:, self.extract.z]
        y = arrays[:, self.extract.y]
        z = arrays[:, self.extract.x]
        ax = plt.axes(projection='3d')
        ax.scatter3D(x, y, z, cmap='b', c='b')
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')
        ax.set_zlim((np.min(z) - 0.2, np.max(z) + 0.2))
        plt.title('point extract')
        plt.xlim((np.min(x) - 0.2, np.max(x) + 0.2))
        plt.show()

    # def Calculate_data(self, file_path, FrameLimit, BoundingBox, IntensityBox, case, topic):
    #     """
    #     Calculate the POD, average number of points per frame in FOV/Bounding, and reflectivity information(Main)
    #     """
    #     results = []
    #     FOVROI = ['NoResult']
    #     POD = ['NoResult']
    #     PointsNum = ['NoResult']
    #     MeanItensity = ['NoResult']
    #     Precision = ['NoResult']
    #     self.extract.topic = self.extract.topics[topic]
    #     if self.extract.topic != '/iv_points' and self.extract.topic != 'iv_points':
    #         self.extract.x = 0
    #         self.extract.y = 1
    #         self.extract.z = 2
    #         self.extract.intensity = 3
    #         self.extract.f = 4
    #     pts_arrays, fields = self.extract.get_file_data(file_path, self.extract.topic, FrameLimit)
    #     pts_arrays = pts_arrays[~np.isnan(pts_arrays).any(axis=1)]
    #     pts_sel = self.filter_points(pts_arrays, FrameLimit, BoundingBox, IntensityBox)
    #     pointnum_perframe = np.bincount(pts_sel[:, self.extract.f].astype(int).flatten(), minlength=FrameLimit[1] + 1)
    #     pointnum_perframe = pointnum_perframe[FrameLimit[0]:]
    #     frame_counts = len(pointnum_perframe)
    #     print('共分析： ', frame_counts, '帧')
    #     if case[0] == 1:
    #         FOVROI = self.Analyze_FOVROI_Angular_Resolution(pts_arrays, fields)
    #     if case[4] == 1:
    #         POD = self.POD(pts_sel, frame_counts, len(pts_sel[:, 4]) / frame_counts, BoundingBox)
    #     if case[2] == 1:
    #         PointsNum = self.Calculate_points_num(pts_sel, pts_arrays, pointnum_perframe, frame_counts, FrameLimit)
    #     if case[1] == 1:
    #         MeanItensity = self.calculate_mean_intensity(pts_sel, self.extract.f, self.extract.intensity, FrameLimit)
    #     if case[3] == 1:
    #         Precision = self.Extract_point_fitting_plane(pts_sel, FrameLimit)
    #     results.extend([FOVROI])
    #     results.extend([MeanItensity])
    #     results.extend(PointsNum)
    #     results.extend([Precision])
    #     results.extend(POD)
    #     ExcelOperation.WritetToExcel(results, file_path)
    #     return results

    def Calculate_data(self, file_path, FrameLimit, BoundingBox, IntensityBox, case, topic):
        # This function calculates the POD, average number of points per frame in FOV/Bounding, and reflectivity
        # information(Main)
        # Input: file_path - a string for the path of the file to read
        #        FrameLimit - a tuple of two integers for the start and end frame
        #        BoundingBox - a tuple of six floats for the x, y and z limits of the bounding box
        #        IntensityBox - a tuple of two floats for the intensity range
        #        case - a list of five integers (0 or 1) for indicating which functions to call
        #        topic - an integer for indicating which topic to extract
        # Output: results - a dictionary of strings for the results of each function
        results = {"FOVROI": "NoResult", "POD": "NoResult", "PointsNum": "NoResult", "MeanItensity": "NoResult",
                   "Precision": "NoResult"}
        names = ["FOVROI", "POD", "PointsNum", "MeanItensity", "Precision"]
        self.extract.topic = self.extract.topics[topic]
        if self.extract.topic in ['/iv_points', 'iv_points']:
            self.extract.x = 3
            self.extract.y = 4
            self.extract.z = 5
            self.extract.intensity = 6
            self.extract.f = 7
        else:
            self.extract.x = 0
            self.extract.y = 1
            self.extract.z = 2
            self.extract.intensity = 3
            self.extract.f = 4
        pts_arrays, fields = self.extract.get_file_data(file_path, self.extract.topic, FrameLimit)
        pts_arrays = pts_arrays[~np.isnan(pts_arrays).any(axis=1)]
        pts_sel = self.filter_points(pts_arrays, FrameLimit, BoundingBox, IntensityBox)
        pointnum_perframe = np.bincount(pts_sel[:, self.extract.f].astype(int).flatten(), minlength=FrameLimit[1] + 1)
        pointnum_perframe = pointnum_perframe[FrameLimit[0]:]
        frame_counts = len(pointnum_perframe)
        print('共分析： ', frame_counts, '帧')
        # 定义一个字典，键是case的索引，值是一个元组，包含函数名和参数列表
        functions = {
            0: (self.Analyze_FOVROI_Angular_Resolution, [pts_arrays, fields]),
            4: (self.POD, [pts_sel, frame_counts, len(pts_sel[:, 4]) / frame_counts, BoundingBox]),
            2: (self.Calculate_points_num, [pts_sel, pts_arrays, pointnum_perframe, frame_counts, FrameLimit]),
            1: (self.calculate_mean_intensity, [pts_sel, self.extract.f, self.extract.intensity, FrameLimit]),
            3: (self.Extract_point_fitting_plane, [pts_sel, FrameLimit])
        }

        for i in range(len(case)):
            if case[i] == 1:
                func = functions[i][0]  # 获取函数名
                args = functions[i][1]  # 获取参数列表
                result = func(*args)  # 调用函数并传入参数
                name = names[i]  # 获取函数的名称
                results[name] = result  # 把返回值更新到results字典中

        ExcelOperation.WritetToExcel(results, file_path)

        return results

    # def get_points_distance(self, pts_sel):
    #     """
    #     Calculate_points_average_distance
    #     """
    #     distance = 0
    #     x = pts_sel[:, self.extract.x]
    #     y = pts_sel[:, self.extract.y]
    #     z = pts_sel[:, self.extract.z]
    #     for i in range(len(pts_sel[:, 4])):
    #         distance = distance + math.sqrt(x[i] ** 2 + y[i] ** 2 + z[i] ** 2)
    #
    #     distance = distance / len(pts_sel[:, 4])
    #     print('distance=:', distance)
    #     return distance

    def get_points_distance(self, pts_sel):
        # This function calculates the average distance of a set of points to the origin
        # Input: pts_sel - a numpy array of shape (n, 6) or (n, 3), where n is the number of points
        # Output: distance - a float for the average distance
        distance = np.sqrt(np.sum(pts_sel[:, self.extract.x:self.extract.z + 1] ** 2, axis=1)).mean()
        print('distance = {:.3f}'.format(distance))
        return distance

    # def Calculate_points_num(self, pts_sel, pts_arrays, pointnum_perframe, frame_count, FrameLimit):
    #     """
    #     Calculate_data_points
    #     """
    #     pts_arrays = pts_arrays[np.where(
    #         (pts_arrays[:, self.extract.f] > FrameLimit[0] - 1) & (pts_arrays[:, self.extract.f] < FrameLimit[1] + 1))]
    #     fov_pointnum_perframe = np.bincount(pts_arrays[:, self.extract.f].astype(int).flatten())[FrameLimit[0]:]
    #     target_mean_points = len(pts_sel) / frame_count
    #     pts_sel_var = np.var(pointnum_perframe)
    #     pts_sel_std_1 = np.std(pointnum_perframe)
    #     pts_sel_std_2 = np.std(pointnum_perframe, ddof=1)
    #     max_width_height = self.Get_Max_Width_Height(pts_sel)
    #     print('target mean points:', target_mean_points)
    #     print('target points per frame:', pointnum_perframe)
    #     print('目标点方差为：%f' % pts_sel_var)
    #     print('目标点总体标准差为: %f' % pts_sel_std_1)
    #     print('目标点样本标准差为: %f' % pts_sel_std_2)
    #     results = [
    #         ['FOV mean points', pts_arrays.shape[0] / frame_count, 'FOV sum points', pts_arrays.shape[0],
    #          'FOV pointnum perframe', fov_pointnum_perframe.tolist(), 'target mean points', target_mean_points,
    #          'target points Variance', pts_sel_var, 'target points Standard Deviation', pts_sel_std_1,
    #          'target points per frame', pointnum_perframe.tolist(), 'Max Width', max_width_height[0],
    #          'Max Width', max_width_height[1]]]
    #     return results

    def Calculate_points_num(self, pts_sel, pts_arrays, pointnum_perframe, frame_count, FrameLimit):
        # This function calculates some data related to the number of points, such as mean, variance, standard deviation, max width and height
        # Input: pts_sel - a numpy array of shape (n, 6) or (n, 3), where n is the number of selected points
        #        pts_arrays - a numpy array of shape (m, 6) or (m, 3), where m is the number of all points
        #        pointnum_perframe - a numpy array of shape (k,), where k is the number of frames and each element is the number of points in that frame
        #        frame_count - an integer for the number of frames
        #        FrameLimit - a tuple of two integers for the start and end frame
        # Output: results - a dictionary of strings for the results of each calculation
        pts_arrays = pts_arrays[
            (FrameLimit[0] <= pts_arrays[:, self.extract.f]) & (pts_arrays[:, self.extract.f] <= FrameLimit[1])]
        target_mean_points = pts_sel.shape[0] / frame_count
        pts_sel_var = np.var(pointnum_perframe)
        pts_sel_std_1 = np.std(pointnum_perframe)
        pts_sel_std_2 = np.std(pointnum_perframe, ddof=1)
        max_width_height = self.Get_Max_Width_Height(pts_sel)
        fov_pointnum_perframe = np.bincount(pts_arrays[:, self.extract.f].astype(int).flatten())[FrameLimit[0]:]
        print('target mean points: {:.3f}'.format(target_mean_points))
        print('target points per frame: {}'.format(pointnum_perframe))
        print('目标点方差为：{:.3f}'.format(pts_sel_var))
        print('目标点总体标准差为: {:.3f}'.format(pts_sel_std_1))
        print('目标点样本标准差为: {:.3f}'.format(pts_sel_std_2))

        results = {"FOV mean points": pts_arrays.shape[0] / frame_count,
                   "FOV sum points": pts_arrays.shape[0],
                   "FOV pointnum perframe": fov_pointnum_perframe.tolist(),
                   "target mean points": target_mean_points,
                   "target points Variance": pts_sel_var,
                   "target points Standard Deviation": pts_sel_std_1,
                   "target points per frame": pointnum_perframe.tolist(),
                   "Max Width": max_width_height[0],
                   "Max Height": max_width_height[1]}

        return results

    # def Meanintensity_perframe(self, pts_sel, f, inten, FrameLimit):
    #     """
    #     Calculate_points_intensity
    #     """
    #     meanintensity_perframe = []
    #     frame_min = FrameLimit[0]
    #     frame_max = FrameLimit[1]
    #     pts_sel = self.filter_points(pts_sel, FrameLimit, [], [0, 105])
    #     for i in range(frame_min, frame_max):
    #         a = pts_sel[np.where(pts_sel[:, f] == i)]
    #         meanintensity_perframe.append(np.mean(a[:, inten]))
    #         intensity = np.mean(pts_sel[:, inten])
    #
    #     pts_sel_var = np.var(meanintensity_perframe)
    #     pts_sel_std_1 = np.std(meanintensity_perframe)
    #     pts_sel_std_2 = np.std(meanintensity_perframe, ddof=1)
    #     print('mean intensity per frame:', meanintensity_perframe)
    #     print('mean intensity:', intensity)
    #     print('intensity方差为：%f' % pts_sel_var)
    #     print('intensity总体标准差为: %f' % pts_sel_std_1)
    #     print('intensity样本标准差为: %f' % pts_sel_std_2)
    #     meanintensity_perframe.extend([intensity])
    #     results = ['Mean Intensity', intensity, 'Mean Intensity Per Frame', meanintensity_perframe,
    #      'Intensity Variance', pts_sel_var, 'Intensity Standard Deviation', pts_sel_std_1]
    #     return results

    def calculate_mean_intensity(self, points, frame_index, intensity_index, frame_range):
        """
        Calculate the mean intensity and other statistics for each frame and the whole points.

        Parameters:
        points: a numpy array of shape (n, 7) where n is the number of points and 7 is the number of attributes (x, y, z, f, r, g, b)
        frame_index: an integer indicating the index of the frame attribute in the points array
        intensity_index: an integer indicating the index of the intensity attribute in the points array
        frame_range: a tuple of two integers (min_frame, max_frame) indicating the range of frames to select from

        Returns:
        a list of results containing the mean intensity, the mean intensity per frame, the intensity variance and the intensity standard deviation
        """
        # filter by frame range and intensity range
        min_frame, max_frame = frame_range
        min_intensity, max_intensity = 0, 105  # hard-coded for simplicity
        points = self.filter_points(points, frame_range, [], [min_intensity, max_intensity])

        # calculate the mean intensity per frame using list comprehension
        mean_intensity_per_frame = [np.mean(points[points[:, frame_index] == i][:, intensity_index])
                                    for i in range(min_frame, max_frame)]

        # calculate the mean intensity and other statistics for the whole points
        mean_intensity = np.mean(points[:, intensity_index])
        intensity_variance = np.var(mean_intensity_per_frame)
        intensity_std = np.std(mean_intensity_per_frame)

        # print the results using formatted strings
        print(f"mean intensity per frame: {mean_intensity_per_frame}")
        print(f"mean intensity: {mean_intensity}")
        print(f"intensity variance: {intensity_variance}")
        print(f"intensity standard deviation: {intensity_std}")

        # return a list of results
        results = ["Mean Intensity", mean_intensity,
                   "Mean Intensity Per Frame", mean_intensity_per_frame,
                   "Intensity Variance", intensity_variance,
                   "Intensity Standard Deviation", intensity_std]

        return results

    # def Extract_point_fitting_plane(self, pts_sel, FrameLimit):
    #     """
    #     extract_point_fitting_plane
    #     """
    #     i = FrameLimit[0]
    #     distance = self.get_points_distance(pts_sel)
    #     tmp = pts_sel[np.where((pts_sel[:, self.extract.f] >= i) & (pts_sel[:, self.extract.f] < i + distance / 25))]
    #     a, b, y, sigma = self.extract_point_fitting_plane(tmp)
    #     Precision = ['a', a, 'b', b, 'y', y, 'sigma', sigma]
    #     return Precision

    def Extract_point_fitting_plane(self, pts_sel, FrameLimit):
        # This function fits a plane from a subset of points and returns the coefficients and standard deviation of the plane
        # Input: pts_sel - a numpy array of shape (n, 6) or (n, 3), where n is the number of selected points
        #        FrameLimit - a tuple of two integers for the start and end frame
        # Output: Precision - a dictionary of strings for the coefficients and standard deviation of the plane
        i = FrameLimit[0]
        distance = self.get_points_distance(pts_sel)
        tmp = pts_sel[(i <= pts_sel[:, self.extract.f]) & (pts_sel[:, self.extract.f] < i + distance / 25)]
        a, b, y, sigma = self.extract_point_fitting_plane(tmp)
        Precision = {"a": a, "b": b, "y": y, "sigma": sigma}
        return Precision

    # def Filter_xyz(self, input_array, framelimit, bounding_box, intensity_bounding):
    #     """
    #     Select points within the ‘bounding_box’
    #     """
    #     x = self.extract.x
    #     y = self.extract.y
    #     z = self.extract.z
    #     f = self.extract.f
    #     # if len(input_array) < 1:
    #     #     input_array
    #     if bool(framelimit):
    #         frame_max = framelimit[1]
    #         frame_min = framelimit[0]
    #         input_array = input_array[np.where((input_array[:, f] > frame_min - 1) & (input_array[:, f] < frame_max + 1))]
    #     if bool(bounding_box):
    #         xmin, xmax, ymin, ymax, zmin, zmax = (
    #          bounding_box[0], bounding_box[1], bounding_box[2], bounding_box[3],
    #          bounding_box[4], bounding_box[5])
    #         input_array = input_array[np.where((input_array[:, x] > xmin) & (input_array[:, x] < xmax))]
    #         input_array = input_array[np.where((input_array[:, y] > ymin) & (input_array[:, y] < ymax))]
    #         input_array = input_array[np.where((input_array[:, z] > zmin) & (input_array[:, z] < zmax))]
    #     if bool(intensity_bounding):
    #         input_array = input_array[np.where((input_array[:, 6] >= intensity_bounding[0]) & (input_array[:, 6] <= intensity_bounding[1]))]
    #     return input_array

    def filter_points(self, points, frame_range, box_range, intensity_range):
        """
        Select points within the given ranges.

        Parameters:
        points: a numpy array of shape (n, 7) where n is the number of points and 7 is the number of attributes (x, y, z, f, r, g, b)
        frame_range: a tuple of two integers (min_frame, max_frame) indicating the range of frames to select from
        box_range: a tuple of six floats (xmin, xmax, ymin, ymax, zmin, zmax) indicating the range of coordinates to select from
        intensity_range: a tuple of two floats (min_intensity, max_intensity) indicating the range of intensity values to select from

        Returns:
        a numpy array of shape (m, 7) where m is the number of selected points
        """
        x = self.extract.x
        y = self.extract.y
        z = self.extract.z
        f = self.extract.f
        i = 6  # intensity index

        # filter by frame range if given
        if frame_range:
            min_frame, max_frame = frame_range
            points = points[(points[:, f] > min_frame - 1) & (points[:, f] < max_frame + 1)]

        # filter by box range if given
        if box_range:
            xmin, xmax, ymin, ymax, zmin, zmax = box_range
            points = points[(points[:, x] > xmin) & (points[:, x] < xmax) &
                            (points[:, y] > ymin) & (points[:, y] < ymax) &
                            (points[:, z] > zmin) & (points[:, z] < zmax)]

        # filter by intensity range if given
        if intensity_range:
            min_intensity, max_intensity = intensity_range
            points = points[(points[:, i] >= min_intensity) & (points[:, i] <= max_intensity)]

        return points

    # def POD(self, pts, frame_count, real_points, bounding):
    #     """
    #     Calculate points POD within the ‘bounding_box’
    #     """
    #     distance = self.get_points_distance(pts)
    #     LaserSpot = round(0.00087 * distance * 2, 3)
    #     Height = bounding[1] - bounding[0] - LaserSpot * 1.2
    #     Width = bounding[3] - bounding[2] - LaserSpot * 1.35
    #     if distance >= 1000:
    #         number = 0
    #         ideal_points = system_pod_points[len(system_pod_points) - 1]
    #         for i in range(len(system_pod_points) - 1):
    #             row = system_pod_points[i][0]
    #             colmin = system_pod_points[i][1]
    #             colmax = system_pod_points[i][2]
    #             pts1 = pts[np.where((pts[:, 1] == row) & (pts[:, 2] >= colmin) & (pts[:, 2] <= colmax))]
    #             number += len(pts1[:, 1])
    #
    #         real_points = number / frame_count
    #     else:
    #         row = math.atan(Width / (2 * distance)) * 2 * 180 / math.pi / 0.09 + 1
    #         col = math.atan(Height / (2 * distance)) * 2 * 180 / math.pi / 0.08 + 1
    #         ideal_points = row * col
    #         pod = '{:.2%}'.format(real_points / ideal_points)
    #     if ideal_points < real_points:
    #         pod = '{:.2%}'.format(1 + (real_points - ideal_points) / ideal_points)
    #     print('idea_points', ideal_points)
    #     print('real_points', real_points)
    #     print('pod:', pod)
    #     results = [['distance', '{:.2f}'.format(distance), 'ideal_points', '{:.2f}'.format(ideal_points), 'real_points',
    #                 '{:.2f}'.format(real_points), 'POD', '{:.2%}'.format(real_points / ideal_points)]]
    #     return results

    def POD(self, pts, frame_count, real_points, bounding):
        """
        Calculate points POD within the ‘bounding_box’
        """
        distance = self.get_points_distance(pts)
        LaserSpot = round(0.00087 * distance * 2, 3)
        Height = bounding[1] - bounding[0] - LaserSpot * 1.2
        Width = bounding[3] - bounding[2] - LaserSpot * 1.35
        if distance >= 1000:
            number = 0
            ideal_points = system_pod_points[-1]
            for i in range(len(system_pod_points) - 1):
                row, colmin, colmax = system_pod_points[i]
                pts1 = pts[pts[:, 1] == row]
                number += len(pts1[:, 1])

            real_points = number / frame_count
        else:
            row = math.degrees(math.atan(Width / (2 * distance))) * 2 / 0.09 + 1
            col = math.degrees(math.atan(Height / (2 * distance))) * 2 / 0.08 + 1
            ideal_points = row * col
        pod = f'POD: {real_points / ideal_points:.2%}'
        if ideal_points < real_points:
            pod = f'POD: {1 + (real_points - ideal_points) / ideal_points:.2%}'
        print(f'idea_points: {ideal_points}')
        print(f'real_points: {real_points}')
        print(f'pod: {pod}')
        results = [['distance', f'{distance:.2f}', 'ideal_points', f'{ideal_points:.2f}', 'real_points',
                    f'{real_points:.2f}', 'POD', f'{real_points / ideal_points:.2%}']]
        return results

    # def Analyze_ROI_MAXClearance(self, pts, fields):
    #     """
    #     Analyze ROI MAXClearance
    #     """
    #     pts = pts[np.where((pts[:, 2] > 740) & (pts[:, 13] == 3))]
    #     Scanline_max = int(max(pts[:, 1]))
    #     Scanline_min = int(min(pts[:, 1]))
    #     Channel_max = int(max(pts[:, 12]))
    #     Channel_min = int(min(pts[:, 12]))
    #     x = np.zeros([Scanline_max - Scanline_min + 1, Channel_max - Channel_min + 1])
    #     maxClerance = x
    #     for i in range(Scanline_min, Scanline_max + 1):
    #         for j in range(Channel_min, Channel_max + 1):
    #             x[(i - Scanline_min, j)] = np.mean(pts[(np.where((pts[:, 1] == i) & (pts[:, 12] == j)), 3)])
    #
    #     for j in range(Channel_min, Channel_max):
    #         for i in range(len(x[:, 0])):
    #             maxClerance[(i, j)] = x[(i, j + 1)] - x[(i, j)]
    #
    #     print('ROIMaxClerance:', max(map(max, maxClerance)))

    def Analyze_ROI_MAXClearance(self, pts, fields):
        """
        Analyze ROI MAXClearance
        """
        pts = pts[(pts[:, 2] > 740) & (pts[:, 13] == 3)]
        Scanline_max = int(pts[:, 1].max())
        Scanline_min = int(pts[:, 1].min())
        Channel_max = int(pts[:, 12].max())
        Channel_min = int(pts[:, 12].min())
        x = np.mean(pts[:, 3].reshape(Scanline_max - Scanline_min + 1, Channel_max - Channel_min + 1), axis=0)
        maxClerance = x[:, 1:] - x[:, :-1]
        print(f'ROIMaxClerance: {maxClerance.max()}')

    # def Analyze_FOVROI_Angular_Resolution(self, pts, fields):
    #     """
    #     Analyze ROI&Non-ROI Angular Resolution
    #     """
    #     if 'flags' in fields:
    #         pts = pts[np.where((pts[:, 7] > 10) & (pts[:, 7] < 12))]
    #         temp = np.zeros(len(fields) + 2)
    #         for i in range(len(pts[:, 0])):
    #             channel = np.append((pts[i]), [int('{:08b}'.format(int(pts[(i, 0)]))[-2:], 2)], axis=0)
    #             roi = np.append(channel, [int('{:08b}'.format(int(pts[(i, 0)]))[-3:-2], 2)], axis=0)
    #             temp = np.vstack([temp, roi])
    #
    #         jobs = []
    #         for i in range(2):
    #             p = Process(target=(self.Calculate_Angle_Resolution), args=(temp, i, self.q))
    #             jobs.append(p)
    #             p.start()
    #
    #         for p in jobs:
    #             p.join()
    #
    #         result = [self.q.get() for j in jobs]
    #         print('Branch:\n', result)
    #     return result

    def Analyze_FOVROI_Angular_Resolution(self, pts, fields):
        """
        Analyze ROI&Non-ROI Angular Resolution
        """
        if 'flags' in fields:
            pts = pts[(pts[:, 7] > 10) & (pts[:, 7] < 12)]
            temp = np.zeros([1, len(fields) + 2])
            channel = (pts[:, 0].astype(int) & 3)
            roi = ((pts[:, 0].astype(int) >> 1) & 3)
            temp = np.vstack([temp, np.hstack([pts, channel[:, None], roi[:, None]])])
            with Pool(2) as p:
                result = p.starmap(self.Calculate_Angle_Resolution, [(temp, 0), (temp, 1)])

        return result

    # def Calculate_Angle_Resolution(self, temp, I, q):
    #     """
    #     Calculate ROI&Non-ROI Angular Resolution
    #     """
    #     temp = temp[np.where(temp[:, 12] == I)]
    #     if I == 1:
    #         roi_line = 0
    #     if I == 0:
    #         roi_line = 56
    #     temp1 = temp[np.where((temp[:, 4] > -0.04) & (temp[:, 4] < 0.04))]
    #     top_scanline_id = temp1[np.where(temp1[:, 3] == max(temp1[:, 3]))][:, 1]
    #     top_channel_id = temp1[np.where(temp1[:, 3] == max(temp1[:, 3]))][:, 11]
    #     bottom_scanline_id = temp1[np.where(temp1[:, 3] == min(temp1[:, 3]))][:, 1]
    #     bottom_channel_id = temp1[np.where(temp1[:, 3] == min(temp1[:, 3]))][:, 11]
    #     bottom_line = temp[np.where((temp[:, 1] == bottom_scanline_id) & (temp[:, 11] == bottom_channel_id))]
    #     top_line = temp[np.where((temp[:, 1] == top_scanline_id) & (temp[:, 11] == top_channel_id))]
    #     bottom_point = bottom_line[np.where((bottom_line[:, 4] > -0.04) & (bottom_line[:, 4] < 0.04))]
    #     bottom_point = bottom_point[np.where(bottom_point[:, 3] == min(bottom_point[:, 3]))]
    #     top_point = top_line[np.where((top_line[:, 4] > -0.04) & (top_line[:, 4] < 0.04))]
    #     top_point = top_point[np.where(top_point[:, 3] == max(top_point[:, 3]))]
    #     bottom_line_right = bottom_line[np.where(bottom_line[:, 2] == max(bottom_line[:, 2]))]
    #     bottom_line_left = bottom_line[np.where(bottom_line[:, 4] == min(bottom_line[:, 4]))]
    #     line_num = (max(temp[:, 1]) - min(temp[:, 1]) + 1) * 4 - roi_line
    #     points_num = max(bottom_line[:, 2]) - min(bottom_line[:, 2]) + 1
    #     Lx = bottom_line_left[(0, 3)]
    #     Rx = bottom_line_right[(0, 3)]
    #     Bx = bottom_point[(0, 3)]
    #     Tx = top_point[(0, 3)]
    #     Ly = bottom_line_left[(0, 4)]
    #     Ry = bottom_line_right[(0, 4)]
    #     By = bottom_point[(0, 4)]
    #     Ty = top_point[(0, 4)]
    #     Lz = bottom_line_left[(0, 5)]
    #     Rz = bottom_line_right[(0, 5)]
    #     Bz = bottom_point[(0, 5)]
    #     Tz = top_point[(0, 5)]
    #     Hangle = np.degrees(np.arccos((Lx * Rx + Ly * Ry + Lz * Rz) / np.sqrt(
    #         (pow(Lx, 2) + pow(Ly, 2) + pow(Lz, 2)) * (pow(Rx, 2) + pow(Ry, 2) + pow(Rz, 2)))))
    #     Vangle = np.degrees(np.arccos((Bx * Tx + By * Ty + Bz * Tz) / np.sqrt(
    #         (pow(Bx, 2) + pow(By, 2) + pow(Bz, 2)) * (pow(Tx, 2) + pow(Ty, 2) + pow(Tz, 2)))))
    #     H_Resolution = Hangle / points_num
    #     V_Resolution = Vangle / line_num
    #     print('Hangle:', Hangle, 'H_Resolution:', H_Resolution, 'Vangle:', Vangle, 'V_Resolution:', V_Resolution,
    #           'ROI:', I)
    #     q.put(['Hangle', 'H_Resolution', 'Vangle', 'V_Resolution', 'I'])

    def Calculate_Angle_Resolution(self, points, ROI_Index):
        """
        Calculate ROI&Non-ROI Angular Resolution
        """
        # 选择对应的ROI&Non-ROI数据
        points = points[np.where(points[:, 11] == ROI_Index)]
        # 根据I设置ROI行数
        if ROI_Index == 1:
            roi_line = 0
        if ROI_Index == 0:
            roi_line = 56
        # 筛选出ROI/Non_ROI区域的数据
        points1 = points[np.where((points[:, 4] > -0.04) & (points[:, 4] < 0.04))]
        # 找出区域最上方和最下方的扫描线ID和通道ID
        top_scanline_id = points1[np.where(points1[:, 3] == max(points1[:, 3]))][:, 1]
        top_channel_id = points1[np.where(points1[:, 3] == max(points1[:, 3]))][:, 10]
        bottom_scanline_id = points1[np.where(points1[:, 3] == min(points1[:, 3]))][:, 1]
        bottom_channel_id = points1[np.where(points1[:, 3] == min(points1[:, 3]))][:, 10]
        # 找出最上方和最下方的扫描线数据
        bottom_line = points[np.where((points[:, 1] == bottom_scanline_id) & (points[:, 11] == bottom_channel_id))]
        top_line = points[np.where((points[:, 1] == top_scanline_id) & (points[:, 11] == top_channel_id))]
        # 找出最上方和最下方的扫描线中心点数据
        bottom_point = bottom_line[np.where((bottom_line[:, 4] > -0.04) & (bottom_line[:, 4] < 0.04))]
        bottom_point = bottom_point[np.where(bottom_point[:, 3] == min(bottom_point[:, 3]))]
        top_point = top_line[np.where((top_line[:, 4] > -0.04) & (top_line[:, 4] < 0.04))]
        top_point = top_point[np.where(top_point[:, 3] == max(top_point[:, 3]))]
        # 找出最下方扫描线的左右端点数据
        bottom_line_right = points1[np.where(points1[:, 2] == max(points1[:, 2]))]
        bottom_line_left = points1[np.where(points1[:, 4] == min(points1[:, 4]))]
        # 计算扫描线数和点数
        line_num = (max(points[:, 1]) - min(points[:, 1]) + 1) * 4 - roi_line
        points_num = max(bottom_line[:, 2]) - min(bottom_line[:, 2]) + 1
        # 提取左右端点和上下端点的坐标
        Lx, Ly, Lz = bottom_line_left[(0, [3, 4, 5])]
        Rx, Ry, Rz = bottom_line_right[(0, [3, 4, 5])]
        Bx, By, Bz = bottom_point[(0, [3, 4, 5])]
        Tx, Ty, Tz = top_point[(0, [3, 4, 5])]
        # 计算水平角分辨率和垂直角分辨率
        Hangle = np.degrees(np.arccos((Lx * Rx + Ly * Ry + Lz * Rz) / np.sqrt(
            (pow(Lx, 2) + pow(Ly, 2) + pow(Lz, 2)) * (pow(Rx, 2) + pow(Ry, 2) + pow(Rz, 2)))))
        Vangle = np.degrees(np.arccos((Bx * Tx + By * Ty + Bz * Tz) / np.sqrt(
            (pow(Bx, 2) + pow(By, 2) + pow(Bz, 2)) * (pow(Tx, 2) + pow(Ty, 2) + pow(Tz, 2)))))
        H_Resolution = Hangle / points_num
        V_Resolution = Vangle / line_num
        print('Hangle:', Hangle, 'H_Resolution:', H_Resolution, 'Vangle:', Vangle, 'V_Resolution:', V_Resolution,
              'ROI_Index:', ROI_Index)
        return Hangle, H_Resolution, Vangle, V_Resolution, ROI_Index

    def Calculate_Center_Of_Mess(self, pts_sel):
        """
        Calculate points center of mess
        """
        print('Calculating the center of mass x y z:', np.mean((pts_sel[:, 3:6]), axis=0))

    def Get_Max_Width_Height(self, pts_sel):
        """
        Calculate target points Max Width&Height
        """
        max_width = max(pts_sel[:, self.extract.y]) - min(pts_sel[:, self.extract.y])
        max_height = max(pts_sel[:, self.extract.x]) - min(pts_sel[:, self.extract.x])
        results = [max_width, max_height]
        return results


if __name__ == '__main__':
    system_pod_points = [[22, 634, 635], [23, 635, 636], 4]
    intensity_bounding = []
    file_path = [
        '/home/demo/Desktop/04TestData/FK24_20418/ref/20418_40ref2m_2023-02-27-13-49-14.bag',
        '/home/demo/Desktop/04TestData/FK24_20418/ref/20418_40ref20m_2023-02-27-13-53-33.bag',
        '/home/demo/Desktop/04TestData/FK24_20418/ref/20418_40ref40m_2023-02-27-13-55-38.bag',
        '/home/demo/Desktop/04TestData/FK24_20418/ref/20418_40ref60m_2023-02-27-13-58-39.bag',
        '/home/demo/Desktop/04TestData/FK24_20418/ref/20418_40ref80m_2023-02-27-14-02-30.bag',
        '/home/demo/Desktop/04TestData/FK24_20418/ref/20418_40ref100m_2023-02-27-14-05-12.bag',
        '/home/demo/Desktop/04TestData/FK24_20418/ref/20418_90ref2m_2023-02-27-14-07-49.bag',
        '/home/demo/Desktop/04TestData/FK24_20418/ref/20418_90ref20m_2023-02-27-14-11-16.bag',
        '/home/demo/Desktop/04TestData/FK24_20418/ref/20418_90ref40m_2023-02-27-14-15-07.bag',
        '/home/demo/Desktop/04TestData/FK24_20418/ref/20418_90ref60m_2023-02-27-14-17-05.bag',
        '/home/demo/Desktop/04TestData/FK24_20418/ref/20418_90ref80m_2023-02-27-14-19-35.bag',
        '/home/demo/Desktop/04TestData/FK24_20418/ref/20418_90ref100m_2023-02-27-14-21-33.bag']

    bounding00 = [-0.4, 0.45, -0.45, 1.15, 1.95, 2.1]  # 10%ref
    bounding0 = [-0.45, 1.05, -0.35, 1.25, 19.95, 20.1]
    bounding1 = [-0.5, 0.95, -0.35, 1.15, 39.95, 40.1]
    bounding2 = [-0.6, 0.95, -0.75, 0.85, 60, 60.15]
    bounding3 = [-0.7, 0.8, -0.85, 0.7, 79.9, 80.1]
    bounding4 = [-0.85, 0.65, -1.2, 0.3, 99.9, 100.2]
    bounding5 = [-0.9, 0.55, -0.85, 0.65, 119.9, 120.15]
    bounding6 = [-0.85, 0.6, -1.2, 0.35, 140, 140.7]
    bounding7 = [-1.0, 0.45, -2.1, -0.5, 159.9, 160.15]
    bounding8 = [-1.15, 0.25, -1.85, -0.35, 180, 180.4]
    bounding9 = [-1.25, 0.25, -2.35, -0.7, 199.9, 200.2]
    bounding10 = [-1.35, 0.25, -2.55, -0.7, 219.9, 220.25]
    bounding11 = [-1.5, 0.05, -2.3, -0.75, 239.9, 240.3]
    bounding12 = [-1.55, -0.35, -2.45, -0.8, 249.9, 250.3]
    bounding13 = [-0.35, 0.40, -0.15, 0.75, 2, 2.2]  # 40%ref
    bounding14 = [-0.35, 0.55, -0.35, 0.55, 20, 20.1]
    bounding15 = [-0.45, 0.45, -0.4, 0.55, 39.9, 40.1]
    bounding01 = [-0.55, 0.35, -0.45, 0.4, 59.9, 60.1]
    bounding16 = [-0.75, 0.15, -0.6, 0.35, 79.9, 80.15]
    bounding17 = [-0.9, 0.05, -0.65, 0.25, 100, 100.15]
    bounding18 = [-0.35, 0.45, -0.2, 0.75, 2, 2.15]  # 90%ref
    bounding19 = [-0.35, 0.5, -0.35, 0.55, 20, 20.1]
    bounding20 = [-0.4, 0.35, -0.35, 0.45, 39.9, 40.1]
    bounding21 = [-0.55, 0.2, -0.45, 0.35, 59.9, 60.1]
    bounding22 = [-0.55, 0.15, -0.55, 0.20, 80, 80.1]
    bounding23 = [-0.7, -0.15, -0.56, 0.15, 99.9, 100.1]
    # bounding24 = [-0.75, 0.85, -1.55, 0, 119.8, 120.8]
    # bounding25 = [0.3, 1.85, -1.55, 0, 119.8, 120.8]
    # bounding26 = [0.45, 1.95, -1.55, 0, 119.8, 120.8]
    # bounding27 = [0.45, 1.95, -1.6, -0.1, 119.8, 120.8]
    # bounding28 = [-0.15, 1.35, -2.3, -0.8, 119.8, 120.8]
    # bounding29 = [0.1, 1.6, -2.2, -0.7, 119.8, 120.8]
    # bounding30 = [0.4, 1.8, -2.2, -0.7, 119.8, 120.8]
    # bounding31 = [0.25, 1.85, -2.25, -0.75, 119.8, 120.8]
    bounding = [bounding00, bounding0, bounding1, bounding2, bounding3, bounding4,
                bounding5, bounding6, bounding7, bounding8, bounding9,
                bounding10, bounding11, bounding12, bounding13, bounding14,
                bounding15, bounding01, bounding16, bounding17,
                bounding18, bounding19, bounding20, bounding21, bounding22,
                bounding23]

    FrameLimit0 = [100, 200]
    FrameLimit1 = [0, 100]
    FrameLimit2 = [70, 170]
    FrameLimit3 = [50, 150]
    FrameLimit4 = [0, 100]
    FrameLimit5 = [0, 100]
    FrameLimit6 = [0, 100]
    FrameLimit7 = [0, 100]
    FrameLimit8 = [0, 100]
    FrameLimit9 = [0, 100]
    FrameLimit10 = [0, 100]
    FrameLimit11 = [0, 100]
    FrameLimit = [FrameLimit0, FrameLimit1, FrameLimit2, FrameLimit3, FrameLimit4, FrameLimit5, FrameLimit6,
                  FrameLimit7, FrameLimit8, FrameLimit9, FrameLimit10, FrameLimit11]
    # for i in range(len(file_path)):
    #     Analysis().Calculate_data(file_path[i], [0, 100], bounding[i+14], [], [0, 1, 0, 0, 0], 0)
    Analysis().Calculate_data("C:/Users/Wayne/Desktop/分析脚本/Frame_TS_Analysis/win2bag_UDP/2022_08_26_08_42_43.bag",
                              [0, 100], [], [], [1, 0, 0, 0, 0], 0)
    print('What have done is done')
