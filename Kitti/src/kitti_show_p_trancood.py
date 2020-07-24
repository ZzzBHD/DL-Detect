#!/usr/bin/env python

import os
import sys
import rospy
import copy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from std_msgs.msg import Float32MultiArray
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
sys.path.append("..")
import numpy as np
import cv2


class Object3d(object):
    """ 3d object label """

    def __init__(self, label_file_line):
        data = label_file_line.split(" ")
        data[1:] = [float(x) for x in data[1:]]
        # print(data[0])
        # print(data[1])
        # extract label, truncation, occlusion
        self.type = data[0]  # 'Car', 'Pedestrian', ...
        self.truncation = data[1]  # truncated pixel ratio [0..1]
        self.occlusion = int(
            data[2]
        )  # 0=visible, 1=partly occluded, 2=fully occluded, 3=unknown
        self.alpha = data[3]  # object observation angle [-pi..pi]

        # extract 2d bounding box in 0-based coordinates
        self.xmin = data[4]  # left
        self.ymin = data[5]  # top
        self.xmax = data[6]  # right
        self.ymax = data[7]  # bottom
        self.box2d = np.array([self.xmin, self.ymin, self.xmax, self.ymax])

        # extract 3d bounding box information
        # self.h = data[8]  # box height
        # self.w = data[9]  # box width
        # self.l = data[10]  # box length (in meters)
        self.h = data[8]  # box height
        self.w = data[9]  # box width
        self.l = data[10]  # box length (in meters)
        self.t = (data[11], data[12], data[13])  # location (x,y,z) in camera coord.
        self.ry = data[14]  # yaw angle (around Y-axis in camera coordinates) [-pi..pi]
   
def inverse_rigid_trans(Tr):
    """ Inverse a rigid body transform matrix (3x4 as [R|t])
        [R'|-R't; 0|1]
    """
    inv_Tr = np.zeros_like(Tr)  # 3x4
    inv_Tr[0:3,0:3] = np.transpose(Tr[0:3, 0:3])
    inv_Tr[0:3,3] = np.dot(-np.transpose(Tr[0:3, 0:3]), Tr[0:3, 3])
    return inv_Tr

def roty(t):
    """ Rotation about the y-axis. """
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def rotz(t):
    """Rotation about the z-axis."""
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c, -s,  0],
                     [s,  c,  0],
                     [0,  0,  1]])

def rotx(t):
    """Rotation about the x-axis."""
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[1,  0,  0],
                     [0,  c, -s],
                     [0,  s,  c]])
def compute_box_3d(obj, P=None):
    """ Takes an object and a projection matrix (P) and projects the 3d
        bounding box into the image plane.
        Returns:
            corners_2d: (8,2) array in left image coord.
            corners_3d: (8,3) array in in rect camera coord.
    """
    # compute rotational matrix around yaw axis
    R = rotz(-obj.ry)
    # 3d bounding box dimensions
    l = obj.l
    w = obj.w
    h = obj.h

    # 3d bounding box corners
    y_corners = [l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2]
    z_corners = [0, 0, 0, 0, h, h, h, h]
    x_corners = [w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2]

    # rotate and translate 3d bounding box
    corners_3d = np.dot(R, np.vstack([x_corners, y_corners, z_corners]))
    # print corners_3d.shape
    corners_3d[0, :] = corners_3d[0, :] + obj.t[0]
    corners_3d[1, :] = corners_3d[1, :] + obj.t[1]
    corners_3d[2, :] = corners_3d[2, :] + obj.t[2]
    # only draw 3d bounding box for objs in front of the camera
    # cache = copy.deepcopy(corners_3d[0, :])
    # corners_3d[0, :] = corners_3d[2, :]
    # corners_3d[2, :] = - corners_3d[1, :]
    # corners_3d[1, :] = - cache
    # del cache
    # print corners_3d
    return np.transpose(corners_3d)

def read_label(label_filename):
    lines = [line.rstrip() for line in open(label_filename)]
    objects = [Object3d(line) for line in lines]
    return objects

def get_label_objects(label_dir,idx):
    label_filename = os.path.join(label_dir, "%06d.txt" % (idx))
    return read_label(label_filename)

def type2num(obj):
    if obj.type == 'Car':
        return 1
    elif obj.type == 'Van':
        return 2
    elif obj.type == 'Truck':
        return 3
    elif obj.type == 'Pedestrian':
        return 4   
    elif obj.type == 'Person_sitting':
        return 5
    elif obj.type == 'Cyclist':
        return 6
    elif obj.type == 'Tram':
        return 7
    elif obj.type == 'Misc':
        return 8
    else :
        return 9

if __name__ == '__main__':
    rospy.init_node('kitti_show_cood', anonymous=True)
    pub_p = rospy.Publisher('kitti_box', Float32MultiArray, queue_size=1)

    label_dir = rospy.get_param('~label_dir')
    show_count = rospy.get_param('~show_count')
    show_num = rospy.get_param('~show_num')
    
    all_box = []
    all_type = []

    for count in range(show_count):

        objects = get_label_objects(label_dir,show_num)
        print("object number:", len(objects))
        for obj in objects:
            if obj.type == "DontCare":
                continue
            # Draw 3d bounding box
            box3d_pts_3d = compute_box_3d(obj)
            box3d_pts_3d = np.reshape(box3d_pts_3d,(-1,3))
            box3d_pts_3d_velo = np.reshape(box3d_pts_3d,(1,-1))
            print("-----------------")
            print(box3d_pts_3d_velo)
            type_num = type2num(obj)
            all_box = np.append(all_box,type_num)
            all_box = np.append(all_box,box3d_pts_3d_velo)
            all_type = np.append(all_type,obj.type)
        show_num = show_num + 1
    object_boxs = Float32MultiArray(data=all_box)
    print "Publish ",len(all_box), "Data..."
    print "Type:", all_type
    while not rospy.is_shutdown():
        pub_p.publish(object_boxs) 
