# 1. IMU trajectory integration

### 1. Generate simulated IMU data and use euler integration/ middle point integration to compute the trajectory
`cd vio_data_simulation`  
`mkdir build`  
`cd build`  
`cmake ..`  
`make`  
run **data_gen** in folder **bin**.
  
### 2. Plot the true IMU trajectory and integrated IMU trajectory
`cd vio_data_simulation`  
`cd python_tool`  
`python draw_trajectory`

**euler integration:**
<div align=center><img width=360 height=300 src=https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise2_IMU_Calibration/doc/euler.png></div>

**integration result:**
<div align=center><img src =https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise2_IMU_Calibration/doc/euler_int.png></div>

**middle-point integration:**
<div align=center><img width=380 height=200 src=https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise2_IMU_Calibration/doc/midpoint1.png></div>
<div align=center><img width=620 height=150 src=https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise2_IMU_Calibration/doc/midpoint2.png></div>


**integration result:**
<div align=center><img src=https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise2_IMU_Calibration/doc/mid_point_int.png></div>

It can be shown that the result of middle-point integration is more accurate than euler integration.


# 2. Use Allan Calibration to compute the noise parameter of IMU data

### 1. Generate simulated stationary IMU data with noise
`cd allan_calibration`  
`catkin_make`  
`source devel/setup.bash`  
`rosrun vio_data_simulation vio_data_simulation_node`  
The covariance of IMU noise can be modified in file **param.h**.  
The generated IMU data(imu.bag) is stored in ./allan_calibration/src/kalibr_allan/data.

### 2. Use kalibr_allan tool to compute the noise parameter
**conver the .bag file to .mat file**  
`rosrun bagconvert bagconvert ./src/kalibr_allan/data/imu.bag /imu`  
The generated **imu.mat** is stored in the same folder with **imu.bag**.  

**calculate the noise parameter**  
`cd allan_calibration/src/kalibr_allan/matlab`  
run **SCRIPT_allan_matparallel.m**, this process last about 20min.  
run **SCRIPT_allan_matparallel.m**, the result is shown as below:  

**covariance data 1:**  
gyro_noise_sigma = 0.015  
gyro_bias_sigma = 0.00005  
gyro_noise_sigma = 0.019  
acc_bias_sigma = 0.0005  

<div align=center><img src=https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise2_IMU_Calibration/doc/imu_gyro1.png></div>
<div align=center><img src=https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise2_IMU_Calibration/doc/imu_acc1.png></div>


**covariance data 2:**  
gyro_noise_sigma = 0.06  
gyro_bias_sigma = 0.0005  
gyro_noise_sigma = 0.085  
acc_bias_sigma = 0.005  

<div align=center><img src=https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise2_IMU_Calibration/doc/imu_gyro2.png></div>
<div align=center><img src=https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise2_IMU_Calibration/doc/imu_acc2.png></div>



