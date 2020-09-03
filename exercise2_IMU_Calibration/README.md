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

### euler integration
<div align=center><img width=360 height=300 src=https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise2_IMU_Calibration/doc/euler.png></div>

integration result:
<div align=center><img src =https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise2_IMU_Calibration/doc/euler_int.png></div>





The covariance of IMU noise can be modified in file **param.h**.
