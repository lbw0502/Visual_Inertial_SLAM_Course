# Use Schur Complement to accelerate computation of Hessian matrix

### Schur Complement

Suppose A, B, C, D are four blocks of matrix M, and D is invertible. Let
<div align=center><img src=https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise5_Schur_Complement/doc/schur1.png></div>
Then the Schur complement of the block D of matrix M is defined by
<div align=center><img src=https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise5_Schur_Complement/doc/schur2.png></div>
and if A is invertible, the Schur complement of the block A of the matrix M is defined by
<div align=center><img src=https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise5_Schur_Complement/doc/schur3.png></div>

### Schur Complement in Hessian Matrix
<div align=center><img src=https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise5_Schur_Complement/doc/hessian1.png></div>

### build the project
The code generates 3 camera poses and several 3D landmarks under world frame. The project uses reprojection error and graph optimization to optimize the camera poses.

`mkdir build`  
`cd build`  
`cmake ..`  
`make`  
`./app/testMonoBA`

### optimization result
<div align=center><img src=https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise5_Schur_Complement/doc/result.png></div>
