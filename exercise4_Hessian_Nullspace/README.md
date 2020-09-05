# Nullspace of Hessian Matrix in Bundle Adjustment

In monocular visual slam system, the freedom of unobservability is 7: 6 for camera pose(se3) and 1 for scale.

### Bundle Adjustment
**Loss function:**
<div align=center><img src=https://latex.codecogs.com/gif.latex?%5Cfrac%7B1%7D%7B2%7D%20%5Csum_%7Bi%3D1%7D%5Em%20%5Csum_%7Bj%3D1%7D%5En%7C%7C%5Cbold%7Be%7D_%7Bi%2Cj%7D%7C%7C%5E2%20%3D%20%5Cfrac%7B1%7D%7B2%7D%5Csum_%7Bi%3D1%7D%5Em%20%5Csum_%7Bj%3D1%7D%5En%7C%7C%5Cbold%7Bu%7D_j%20-%20%5Cfrac%7B1%7D%7Bs_j%7D%5Cbold%7BK%7D%5Cbold%7BT_i%7D%5Cbold%7BP_j%7D%20%7C%7C%5E2></div>

use first order of Taylor expansion:
<div align=center><img src=https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise4_Hessian_Nullspace/doc/BA.png></div>
The whole Hessian matrix is sparse, for each measurement, the coresponding Jacobian matrix is only related to one camera pose and one landmark.
<div align=center><img src=https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise4_Hessian_Nullspace/doc/BA1.png></div>
<div align=center><img src=https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise4_Hessian_Nullspace/doc/BA2.png></div>
The Jacobian matrix respect to camera pose and landmark can be computed seperately.

**Jacobian for camera pose**
