# 1. IMU trajectory integration

### 1. generate simulated IMU data and use euler integration/ middle point integration to compute the trajectory
`cd vio_data_simulation`  
`mkdir build`  
`cd build`  
`cmake ..`  
`make`  
run **data_gen** in folder **bin**.
  
### 2. plot the true IMU trajectory and integrated IMU trajectory
`cd vio_data_simulation`  
`cd python_tool`  
`python draw_trajectory`



The covariance of IMU noise can be modified in file **param.h**.
