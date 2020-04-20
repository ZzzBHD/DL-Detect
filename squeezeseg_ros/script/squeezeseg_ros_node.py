#!/usr/bin/python2
# -*- coding: utf-8 -*-

"""
    online segmentation using .npy & SqueezeSeg model

    this script can
                    1. read all .npy file from lidar_2d folder
                    2. predict label from SqueezeSeg model using tensorflow
                    3. publish to 'sqeeuze_seg/points' topic

"""
import sys
import os.path
import numpy as np
from PIL import Image

import tensorflow as tf

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Header

from squeezeseg.config import *
from squeezeseg.nets import SqueezeSeg
from squeezeseg.utils.util import *
from squeezeseg.utils.clock import Clock
from squeezeseg.imdb import kitti  # ed: header added


class NPY_TENSORFLOW_TO_ROS():
    def __init__(self, FLAGS, fov, pub_topic, sub_topic="/sub_topic",  
    input_type="", cloud_type="XYZI",
    x_channel="X", y_channel="Y", z_channel="Z", i_channel="intensity",
    npy_path="", npy_file_list="", read_list=False,read_number=1):

        os.environ['CUDA_VISIBLE_DEVICES'] = FLAGS.gpu
        self._mc = kitti_squeezeSeg_config()
        self._mc.LOAD_PRETRAINED_MODEL = False

        self._mc.BATCH_SIZE = 1
        self._model = SqueezeSeg(self._mc)
        self._saver = tf.train.Saver(self._model.model_params)

        self._session = tf.Session(
            config=tf.ConfigProto(allow_soft_placement=True))
        self._saver.restore(self._session, FLAGS.checkpoint)

        # ed: Publisher
        self._pub = rospy.Publisher(pub_topic, PointCloud2, queue_size=1)
        self._sub = rospy.Subscriber(sub_topic, PointCloud2, self.velo_callback, queue_size=10)

        self._total_num = read_number
        self._input_type = input_type

        self._fov = (int(fov.split(',')[0]),int(fov.split(',')[1]))
        print self._fov
        if read_list:
            self.get_npy_from_lidar_2d(npy_path, npy_file_list)
            print "Total npy file:",self.len_files
            self._total_num = self.len_files

        self._cloud_type = cloud_type
        self._channel = []
        self._channel.extend([x_channel, y_channel, z_channel, i_channel]) 
        self.idx = 0
        if self._input_type == "npy":
            while not rospy.is_shutdown():
                record = np.load(os.path.join(self.npy_path, self.npy_files[self.idx]))
                print "record:",record.shape  #(64, 512, 6)
                feature_cloud = record[:, :, :5]
                self.prediction_publish(feature_cloud)
                self.idx += 1
                if self.idx == self._total_num:
                    self.idx = 0
        elif self._input_type == "bag":
            rospy.spin()

    # Read all .npy data from lidar_2d folder
    def get_npy_from_lidar_2d(self, npy_path, npy_file_list):
        self.npy_path = npy_path
        self.npy_file_list = open(npy_file_list, 'r').read().split('\n')
        self.npy_files = []

        for i in range(len(self.npy_file_list)):
            self.npy_files.append(
                self.npy_path + self.npy_file_list[i] + '.npy')

        self.len_files = len(self.npy_files)

    def prediction_publish(self, cloud):
        clock = Clock()
        feature_cloud = cloud
        # to perform prediction
        lidar_mask = np.reshape(
            (feature_cloud[:, :, 4] > 0),
            [self._mc.ZENITH_LEVEL, self._mc.AZIMUTH_LEVEL, 1]
        )
        norm_lidar = (feature_cloud - self._mc.INPUT_MEAN) / self._mc.INPUT_STD
        print "norm_lidar:",norm_lidar.shape # (64, 512, 5)

        pred_cls = self._session.run(
            self._model.pred_cls,
            feed_dict={
                self._model.lidar_input: [norm_lidar],
                self._model.keep_prob: 1.0,
                self._model.lidar_mask: [lidar_mask]
            }
        )
        label = pred_cls[0]
        print "pred_cls:",pred_cls.shape # (1, 64, 512)
        # point cloud for SqueezeSeg segments
        x = feature_cloud[:, :, 0].reshape(-1)
        y = feature_cloud[:, :, 1].reshape(-1)
        z = feature_cloud[:, :, 2].reshape(-1)
        i = feature_cloud[:, :, 3].reshape(-1)
        label = label.reshape(-1)
        cloud = np.stack((x, y, z, i, label))

        header = Header()
        header.stamp = rospy.Time().now()
        header.frame_id = "pandar"

        # point cloud segments
        msg_segment = self.create_cloud_xyzil32(header, cloud.T)

        # publish
        self._pub.publish(msg_segment)
        rospy.loginfo("Point cloud processed. Took %.6f ms.",
                      clock.takeRealTime())

    # create pc2_msg with 5 fields
    def create_cloud_xyzil32(self, header, points):
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('intensity', 12, PointField.FLOAT32, 1),
                  PointField('label', 16, PointField.FLOAT32, 1)]
        return pc2.create_cloud(header, fields, points)

    def velo_callback(self, msg):
        if self._cloud_type == "XYZI":
            pcl_msg = pc2.read_points(msg, skip_nans=False, field_names=(
            self._channel[0], self._channel[1], self._channel[2], self._channel[3]))
            cloud_input = np.array(list(pcl_msg), dtype=np.float32)
            cloud_input[:, 3] = self._normalize(cloud_input[:, 3])
        # elif self._cloud_type == "XYZ":
        #     pcl_msg = pc2.read_points(msg, skip_nans=False, field_names=(
        #     self._channel[0], self._channel[1], self._channel[2]))
        #     cloud_input = np.array(list(pcl_msg), dtype=np.float32)
        #     cloud_input[:, 2] = self._normalize(cloud_input[:, 2])
        print("\ncloud_input.shape = ", cloud_input.shape)

        crop_points = self.crop_cloud(cloud_input)
        print("crop_points.shape = ", crop_points.shape)
        feature_image = self.projectCloud2Image(crop_points)#(64, 512, 5), 5->(x,y,z,i,range)
        print("feature_image.shape = ", feature_image.shape)
        self.prediction_publish(cloud=feature_image)

    def crop_cloud(self, cloud):
        '''
        function: crop cloud by fov of (-45, 45) at azimuth 
        input: cloud: (n, 4)-->(x, y, z, i)
        ouput: cloud: (m, 4)-->(x, y, z, i) (m ~= n/4)
        ''' 
        x = cloud[:, 0]
        y = cloud[:, 1]
        azi = np.arctan2(y, x).reshape(len(x), 1) * 180.0 / np.pi
        cloud_xyzia = np.concatenate((cloud, azi), axis=1)#(x, y, z, i, azimuth)
        if self._cloud_type == "XYZI":
            crop_index = (cloud_xyzia[:, 4] > self._fov[0]) &  (cloud_xyzia[:, 4] < self._fov[1])#fov
            crop_points = cloud_xyzia[crop_index]
            return crop_points[:, :4] # (x, y, z, i)
        # elif self._cloud_type == "XYZ":
        #     crop_index = (cloud_xyzia[:, 3] > self._fov[0]) &  (cloud_xyzia[:, 3] < self._fov[1])#fov
        #     crop_points = cloud_xyzia[crop_index]
        #     return crop_points[:, :3] # (x, y, z)


    def projectCloud2Image(self, cloud, width=512, height=64):
        '''
        function: project cloud to spherical image
        input:  cloud:           (n, 4)      --> (x, y, z, i)
        output: spherical image: (64, 512, 5)--> (x, y, z, i, r)
        '''
        n = cloud.shape[0]
        x = cloud[:, 0].reshape(n, 1)
        y = cloud[:, 1].reshape(n, 1)
        z = cloud[:, 2].reshape(n, 1)
        r = np.sqrt(x**2 + y**2 + z**2).reshape(n, 1)

        yaw = np.arctan2(y, x).reshape(n, 1)
        pitch = np.arcsin(z/r).reshape(n, 1)

        #compute resolution of the cloud at each direction
        resolution_w = (yaw.max() - yaw.min()) / (width - 1)
        resolution_h = (pitch.max() - pitch.min()) / (height - 1)

        #compute each point's grid index in the image
        index_w = np.floor(((yaw - yaw.min()) / resolution_w)).astype(np.int)
        index_h = np.floor((pitch - pitch.min()) / resolution_h).astype(np.int)

        cloud_xyzir = np.concatenate((cloud, r), axis=1) #(x,y,z,i,r)
        
        spherical_image = np.zeros((height, width, 5))
        spherical_image[index_h, index_w, :] = cloud_xyzir[:, np.newaxis, :] #broadcast
        spherical_image = spherical_image[::-1, ::-1, :] #reverse image
        return spherical_image

    def _normalize(self,x):
            return (x - x.min())/(x.max() - x.min())


if __name__ == '__main__':
    rospy.init_node('squeezeseg_ros_node')

    pub_topic = rospy.get_param('pub_topic')
    input_type = rospy.get_param('input_type')
    fov = rospy.get_param('fov')
    if input_type == "bag":
        x_channel = rospy.get_param('x_channel')
        y_channel = rospy.get_param('y_channel')
        z_channel = rospy.get_param('z_channel')
        i_channel = rospy.get_param('i_channel')
        sub_topic = rospy.get_param('sub_topic')
    elif input_type == "npy":
        read_list = rospy.get_param('read_list')
        print read_list
        npy_path = rospy.get_param('npy_path')
        npy_file_list = rospy.get_param('npy_file_list')
        if not read_list:
            read_number = rospy.get_param('read_number')
        else:
            read_number = 1
    checkpoint = rospy.get_param('checkpoint')
    gpu = rospy.get_param('gpu')

    FLAGS = tf.app.flags.FLAGS
    tf.app.flags.DEFINE_string(
        'checkpoint', checkpoint,
        """Path to the model paramter file.""")
    tf.app.flags.DEFINE_string('gpu', gpu, """gpu id.""")
    if input_type == "npy":
        print "-----npy model-----"
        npy_tensorflow_to_ros = NPY_TENSORFLOW_TO_ROS(FLAGS=FLAGS,
                                                      pub_topic=pub_topic,
                                                      fov=fov,
                                                      input_type=input_type,
                                                      npy_path=npy_path,
                                                      npy_file_list=npy_file_list,
                                                      read_list=read_list,
                                                      read_number=read_number)
    if input_type == "bag":
        print "-----bag model-----"
        npy_tensorflow_to_ros = NPY_TENSORFLOW_TO_ROS(FLAGS=FLAGS,
                                                      pub_topic=pub_topic,
                                                      sub_topic=sub_topic,
                                                      fov=fov,
                                                      input_type=input_type,
                                                      x_channel=x_channel,
                                                      y_channel=y_channel,
                                                      z_channel=z_channel,
                                                      i_channel=i_channel)
            
