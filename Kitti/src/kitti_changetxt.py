#!/usr/bin/env python
#coding=utf-8

import os
import sys
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from std_msgs.msg import Float32MultiArray 
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
sys.path.append("..")
import numpy as np
import cv2

#把kitti label中原先在camera坐标系下的3Dbox坐标转换成Lidar坐标系下

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
        self.h = data[8]  # box height
        self.w = data[9]  # box width
        self.l = data[10]  # box length (in meters)
        self.t = (data[11], data[12], data[13])  # location (x,y,z) in camera coord.
        self.ry = data[14]  # yaw angle (around Y-axis in camera coordinates) [-pi..pi]

    # def print_object(self):
    #     print(
    #         "Type, truncation, occlusion, alpha: %s, %d, %d, %f"
    #         % (self.type, self.truncation, self.occlusion, self.alpha)
    #     )
    #     print(
    #         "2d bbox (x0,y0,x1,y1): %f, %f, %f, %f"
    #         % (self.xmin, self.ymin, self.xmax, self.ymax)
    #     )
    #     print("3d bbox h,w,l: %f, %f, %f" % (self.h, self.w, self.l))
    #     print(
    #         "3d bbox location, ry: (%f, %f, %f), %f"
    #         % (self.t[0], self.t[1], self.t[2], self.ry)
    #     )

class Calibration(object):
    """ Calibration matrices and utils
        3d XYZ in <label>.txt are in rect camera coord.
        2d box xy are in image2 coord
        Points in <lidar>.bin are in Velodyne coord.

        y_image2 = P^2_rect * x_rect
        y_image2 = P^2_rect * R0_rect * Tr_velo_to_cam * x_velo
        x_ref = Tr_velo_to_cam * x_velo
        x_rect = R0_rect * x_ref

        P^2_rect = [f^2_u,  0,      c^2_u,  -f^2_u b^2_x;
                    0,      f^2_v,  c^2_v,  -f^2_v b^2_y;
                    0,      0,      1,      0]
                 = K * [1|t]

        image2 coord:
         ----> x-axis (u)
        |
        |
        v y-axis (v)

        velodyne coord:
        front x, left y, up z

        rect/ref camera coord:
        right x, down y, front z

        Ref (KITTI paper): http://www.cvlibs.net/publications/Geiger2013IJRR.pdf

        TODO(rqi): do matrix multiplication only once for each projection.
    """

    def __init__(self, calib_filepath, from_video=False):
        if from_video:
            calibs = self.read_calib_from_video(calib_filepath)
        else:
            calibs = self.read_calib_file(calib_filepath)
        # Projection matrix from rect camera coord to image2 coord
        self.P = calibs["P2"]
        self.P = np.reshape(self.P, [3, 4])
        # Rigid transform from Velodyne coord to reference camera coord
        self.V2C = calibs["Tr_velo_to_cam"]
        self.V2C = np.reshape(self.V2C, [3, 4])
        self.C2V = inverse_rigid_trans(self.V2C)
        # Rotation from reference camera coord to rect camera coord
        self.R0 = calibs["R0_rect"]
        self.R0 = np.reshape(self.R0, [3, 3])

        # Camera intrinsics and extrinsics
        self.c_u = self.P[0, 2]
        self.c_v = self.P[1, 2]
        self.f_u = self.P[0, 0]
        self.f_v = self.P[1, 1]
        self.b_x = self.P[0, 3] / (-self.f_u)  # relative
        self.b_y = self.P[1, 3] / (-self.f_v)

    def read_calib_file(self, filepath):
        """ Read in a calibration file and parse into a dictionary.
        Ref: https://github.com/utiasSTARS/pykitti/blob/master/pykitti/utils.py
        """
        data = {}
        with open(filepath, "r") as f:
            for line in f.readlines():
                line = line.rstrip()
                if len(line) == 0:
                    continue
                key, value = line.split(":", 1)
                # The only non-float values in these files are dates, which
                # we don't care about anyway
                try:
                    data[key] = np.array([float(x) for x in value.split()])
                except ValueError:
                    pass

        return data

    def read_calib_from_video(self, calib_root_dir):
        """ Read calibration for camera 2 from video calib files.
            there are calib_cam_to_cam and calib_velo_to_cam under the calib_root_dir
        """
        data = {}
        cam2cam = self.read_calib_file(
            os.path.join(calib_root_dir, "calib_cam_to_cam.txt")
        )
        velo2cam = self.read_calib_file(
            os.path.join(calib_root_dir, "calib_velo_to_cam.txt")
        )
        Tr_velo_to_cam = np.zeros((3, 4))
        Tr_velo_to_cam[0:3, 0:3] = np.reshape(velo2cam["R"], [3, 3])
        Tr_velo_to_cam[:, 3] = velo2cam["T"]
        data["Tr_velo_to_cam"] = np.reshape(Tr_velo_to_cam, [12])
        data["R0_rect"] = cam2cam["R_rect_00"]
        data["P2"] = cam2cam["P_rect_02"]
        return data

    def cart2hom(self, pts_3d):
        """ Input: nx3 points in Cartesian
            Oupput: nx4 points in Homogeneous by pending 1
        """
        n = pts_3d.shape[0]
        pts_3d_hom = np.hstack((pts_3d, np.ones((n, 1))))
        return pts_3d_hom

        # ===========================
        # ------- 3d to 3d ----------
        # ===========================

    def project_velo_to_ref(self, pts_3d_velo):
        pts_3d_velo = self.cart2hom(pts_3d_velo)  # nx4
        return np.dot(pts_3d_velo, np.transpose(self.V2C))

    def project_ref_to_velo(self, pts_3d_ref):
        pts_3d_ref = self.cart2hom(pts_3d_ref)  # nx4
        return np.dot(pts_3d_ref, np.transpose(self.C2V))

    def project_rect_to_ref(self, pts_3d_rect):
        """ Input and Output are nx3 points """
        return np.transpose(np.dot(np.linalg.inv(self.R0), np.transpose(pts_3d_rect)))

    def project_ref_to_rect(self, pts_3d_ref):
        """ Input and Output are nx3 points """
        return np.transpose(np.dot(self.R0, np.transpose(pts_3d_ref)))

    def project_rect_to_velo(self, pts_3d_rect):
        """ Input: nx3 points in rect camera coord.
            Output: nx3 points in velodyne coord.
        """
        pts_3d_ref = self.project_rect_to_ref(pts_3d_rect)
        return self.project_ref_to_velo(pts_3d_ref)

    def project_velo_to_rect(self, pts_3d_velo):
        pts_3d_ref = self.project_velo_to_ref(pts_3d_velo)
        return self.project_ref_to_rect(pts_3d_ref)

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
    R = roty(obj.ry)

    # 3d bounding box dimensions
    l = obj.l
    w = obj.w
    h = obj.h

    # 3d bounding box corners
    x_corners = [l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2]
    y_corners = [0, 0, 0, 0, -h, -h, -h, -h]
    z_corners = [w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2]

    # rotate and translate 3d bounding box
    corners_3d = np.dot(R, np.vstack([x_corners, y_corners, z_corners]))
    # print corners_3d.shape
    corners_3d[0, :] = corners_3d[0, :] + obj.t[0]
    corners_3d[1, :] = corners_3d[1, :] + obj.t[1]
    corners_3d[2, :] = corners_3d[2, :] + obj.t[2]
    corners_3d[0,-1] = obj.t[0]
    corners_3d[1,-1] = obj.t[1]
    corners_3d[2,-1] = obj.t[2]
    # print 'cornsers_3d: ', corners_3d
    # only draw 3d bounding box for objs in front of the camera

    # print  corners_2d
    return np.transpose(corners_3d)

def read_label(label_filename):
    lines = [line.rstrip() for line in open(label_filename)]
    objects = [Object3d(line) for line in lines]
    return objects

def get_calibration(calib_dir,idx):
    calib_filename = os.path.join(calib_dir, "%06d.txt" % (idx))
    return Calibration(calib_filename)

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
    rospy.init_node('kitti_show', anonymous=True)
    pub_p = rospy.Publisher('kitti_box', Float32MultiArray, queue_size=1)
    
    label_dir = "/media/cyber-z/F/数据集/KITTI/data_object_label_2/training/label_2/"
    calib_dir = "/media/cyber-z/F/数据集/KITTI/data_object_calib/training/calib/"
    out_dir = "/media/cyber-z/E/test/label_tran/"
    fileList = os.listdir(label_dir)

    for file in fileList:
        print file
        i = 0
        with open(out_dir + file, 'a+') as f:
            calib_name = calib_dir + file
            label_name = label_dir + file
            objects = read_label(label_name)
            # objects = get_label_objects(label_name,show_num)
            calib = Calibration(calib_name)
            # calib = get_calibration(calib_dir,show_num)
            for obj in objects:
                i = i + 1
                # if obj.type == "DontCare":
                #     continue
                # Draw 3d bounding box
                box3d_pts_3d = compute_box_3d(obj, calib.P)
                # box3d_pts_3d = compute_box_3d(obj)
                box3d_pts_3d_velo = calib.project_rect_to_velo(box3d_pts_3d)
                box3d_pts_3d_velo = np.reshape(box3d_pts_3d_velo,(-1,3))
                x_mean = box3d_pts_3d_velo[-1, 0]
                y_mean = box3d_pts_3d_velo[-1, 1]
                z_mean = box3d_pts_3d_velo[-1, 2]
                f.write(obj.type)
                f.write(" ")
                if obj.type != "DontCare":
                    f.write('%.02f'%obj.truncation)
                    f.write(" ")
                    f.write(str(obj.occlusion))
                    f.write(" ")
                    f.write(str('%.02f'%obj.alpha))
                    f.write(" ")
                    f.write(str('%.02f'%obj.xmin))
                    f.write(" ")
                    f.write(str('%.02f'%obj.ymin))
                    f.write(" ")
                    f.write(str('%.02f'%obj.xmax))
                    f.write(" ")
                    f.write(str('%.02f'%obj.ymax))
                    f.write(" ")
                    f.write(str('%.02f'%obj.h))
                    f.write(" ")
                    f.write(str('%.02f'%obj.w))
                    f.write(" ")
                    f.write(str('%.02f'%obj.l))
                    f.write(" ")
                    f.write(str('%.02f'%x_mean))
                    f.write(" ")
                    f.write(str('%.02f'%y_mean))
                    f.write(" ")
                    f.write(str('%.02f'%z_mean))
                    f.write(" ")
                    f.write(str('%.02f'%obj.ry))
                else:
                    f.write(str(-1))
                    f.write(" ")
                    f.write(str(-1))
                    f.write(" ")
                    f.write(str(-10))
                    f.write(" ")
                    f.write(str('%.02f'%obj.xmin))
                    f.write(" ")
                    f.write(str('%.02f'%obj.ymin))
                    f.write(" ")
                    f.write(str('%.02f'%obj.xmax))
                    f.write(" ")
                    f.write(str('%.02f'%obj.ymax))
                    f.write(" ")
                    f.write(str(-1))
                    f.write(" ")
                    f.write(str(-1))
                    f.write(" ")
                    f.write(str(-1))
                    f.write(" ")
                    f.write(str(-1000))
                    f.write(" ")
                    f.write(str(-1000))
                    f.write(" ")
                    f.write(str(-1000))
                    f.write(" ")
                    f.write(str(-10))
                if i != len(objects):
                    f.write("\n")
            