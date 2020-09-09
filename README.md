# Visual_Inertial_SLAM_Course

**Introduction**：
This project is used for an online course about visual inertial odometry on [shenlanxueyuan](http://www.shenlanxueyuan.com). She is based on the VINS-Mono framework but does not rely on ROS, Ceres, G2o. This code is very basic and aims to demonstrate Eigen-based back-end LM algorithms, sliding window algorithms, robust kernel functions, etc. This code supports for Ubuntu or Mac OS.

+ [**exercise1**](./exercise1_Rotation_update): Rotation update of Lie algebra and quaternion
+ [**exercise2**](./exercise2_IMU_Calibration): IMU trajectory integration and IMU noise calibration
+ [**exercise3**](./exercise3_Curve_Fitting_LM): Use LM algorithm to fit a curve
+ [**exercise4**](./exercise4_Hessian_Nullspace): Nullspace of Hessian Matrix in Bundle Adjustment
+ [**exercise5**](./exercise5_Schur_Complement): Use Schur Complement to accelerate computation of Hessian matrix
+ [**exercise6**](./exercise6_Triangulation): Triangulation of landmark in Mono-SLAM
+ [**exercise7**](./exercise7_VinsMon_on_euroc): Vins Mono on euroc dataset

## Basic framework of Vins-Mono
<div algin=center><img src=./doc/framework.png></dev>
