# Triangulation of landmark in Mono-SLAM

+ consider that a landmark y can be observed by several key frames ![](https://latex.codecogs.com/gif.latex?k%20%3D%201%2C...%2Cn)
+ we use homogenous coordinate, ![](https://latex.codecogs.com/gif.latex?%5Cbold%7By%7D%20%5Cin%20%5Cmathbb%7BR%7D%5E4). The observation in each key frame is ![](https://latex.codecogs.com/gif.latex?%5Cbold%7Bx%7D_k%20%3D%20%5Bu_k%2C%20v_k%2C%201%5D%5ET)
+ the projection matrix (from world frame to camera frame) ![](https://latex.codecogs.com/gif.latex?%5Cbold%7BP%7D_k%20%3D%20%5B%5Cbold%7BR%7D_k%2C%20%5Cbold%7Bt%7D_k%5D%20%5Cin%20%5Cmathbb%7BR%7D%5E%7B3%20%5Ctimes%204%7D)
+ we have
<div align=center><img src=https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise6_Triangulation/doc/tri1.png></div> 

where ![img](https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise6_Triangulation/doc/lambda_k.png) is the depth of observed landmark under camera frame  

+ we also know
<div align=center><img src=https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise6_Triangulation/doc/tri2.png></div>

![](https://latex.codecogs.com/gif.latex?%5Cbold%7BP%7D%5ET_%7Bk%2C3%7D) is the 3rd row of ![](https://latex.codecogs.com/gif.latex?%5Cbold%7BP%7D_%7Bk%7D)  



+ plug this equation into the eqution above,
<div align=center><img src=https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise6_Triangulation/doc/tri3.png></div>

+ stack all these observations together
<div align=center><img src=https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise6_Triangulation/doc/tri4.png></div>

So we can compute the SVD of D and take the last column of V matrix as the result.


### build the project
### run the project
`cd build`  
`./estimate_depth`
### result
<div align=center><img src=https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise6_Triangulation/doc/result.png></div>
