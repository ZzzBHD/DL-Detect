# DL-Detect
Tof Detection on E100 through DL （Stage-3）
## 上传并备份一些通过Deep Learning进行点云目标识别的项目代码
### Stage-3部分简介
这部分主要是Tof（Depth）和深度学习结合的内容，包括:<br>
1：对tof相机的点云处理，与Stage-1/2不同的是，在进行voxel采样前进行了一次形态学处理，对depth img进行了先开再闭的运算，滤波大小为3x3<br>
2：上传了一些在kitti 3D object榜上开源的深度学习模型代码，其中某些完成了ROS框架内的调用实现，参考了以下仓库：<br>
（1）Second：https://github.com/cedricxie/second_ros.git<br>
（2）Voxelnet：https://github.com/qianguih/voxelnet.git （save model需要从原始仓库中clone）<br>
（3）Squeezeseg：https://github.com/AbangLZU/SqueezeSeg_Ros.git<br>
（4）SqueezesegV2：https://github.com/chaomath/squeezesegv2_ros.git<br>
3：对kitti数据进行了bounding box可视化，代码放在了Kitti show文件夹中<br>
4：实现了将tof数据进行SVM预分类后单帧写入label的代码，存放在Tof_writekitti文件夹中<br>
