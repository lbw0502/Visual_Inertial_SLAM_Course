
# Rotation update of Lie algebra and quaternion

We can use rotation matrix or quaternion to represent a rotation operation. When we use ![](https://latex.codecogs.com/gif.latex?%5Cbold%7Bw%7D) to upate a rotation, there are two ways:
<div align=center><img src =https://latex.codecogs.com/gif.latex?%5Cbold%7BR%7D%20%5Cleftarrow%20%5Cbold%7BR%7D%20%5C%20exp%28%5Cbold%7B%5Chat%7Bw%7D%7D%29></div>
<div align=center><img src =https://latex.codecogs.com/gif.latex?%5Cbold%7Bq%7D%20%5Cleftarrow%20%5Cbold%7Bq%7D%20%5Cotimes%20%5B1%2C%5Cfrac%7B1%7D%7B2%7D%5Cbold%7Bw%7D%5D%5ET></div>

When we assign ![](https://latex.codecogs.com/gif.latex?%5Cbold%7Bw%7D) a very small value, eg. ![](https://latex.codecogs.com/gif.latex?%5Cbold%7Bw%7D%20%3D%20%5B0.01%2C0.02%2C0.03%5D%5ET), the two update results should be very close.

### 1. Build the project

### 2. Result
<div align=center><img src =https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise1_Rotation_update/doc/result.png></div>

