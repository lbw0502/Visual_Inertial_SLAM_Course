# Use LM algorithm to fit a curve
Given 100 x and y, compute the parameters of function ![](https://latex.codecogs.com/gif.latex?y%20%3D%20exp%28ax%5E2&plus;bx&plus;c%29)
### 1. build the project
### 2. run 
`cd build/app`  
`./testCurveFitting`

## Levenberg-Marquardt algorithm
Residual function f(x) is a non-linear function, use Taylor expansion (first order) to approximate:  
<div align=center><img src=https://latex.codecogs.com/gif.latex?%5Cbold%7Bf%7D%20%28%5Cbold%7Bx%7D&plus;%5CDelta%20%5Cbold%7Bx%7D%29%20%5Capprox%20%5Cbold%7Bf%28x%29%7D%20&plus;%20%5Cbold%7BJ%7D%20%5CDelta%20%5Cbold%7Bx%7D></div>
Lost function F(x) becomes:
<div align=center><img src=https://latex.codecogs.com/gif.latex?F%28%5Cbold%7Bx%7D&plus;%20%5CDelta%20%5Cbold%7Bx%7D%29%20%3D%20%5Cfrac%7B1%7D%7B2%7D%20%7C%7C%5Cbold%7Bf%28x&plus;%5CDelta%20x%29%7D%7C%7C%5E2%20%3D%20F%28%5Cbold%7Bx%7D%29%20&plus;%20%5CDelta%20%5Cbold%7Bx%7D%20%5ET%20%5Cbold%7BJ%7D%5ET%5Cbold%7Bf%7D%20&plus;%20%5Cfrac%7B1%7D%7B2%7D%20%5CDelta%20%5Cbold%7Bx%7D%5ET%5Cbold%7BJ%7D%5ET%5Cbold%7BJ%7D%20%5CDelta%20%5Cbold%7Bx%7D></div>

Compute first order derivative of lost function and set it to zero:
<div align=center><img src=https://latex.codecogs.com/gif.latex?%5Cbold%7BJ%7D%5ET%20%5Cbold%7BJ%7D%20%5CDelta%20%5Cbold%7Bx%7D%20%3D%20-%5Cbold%7BJ%7D%5ET%5Cbold%7Bf%7D></div>
Levenberg and Marquardt improve this Gauss-Newton method, and use damping factor:  
<div align=center><img src=https://latex.codecogs.com/gif.latex?%28%5Cbold%7BJ%7D%5ET%20%5Cbold%7BJ%7D%20%5CDelta%20&plus;%20%5Cmu%20%5Cbold%7BI%7D%29%20%5Cbold%7Bx%7D%20%3D%20-%5Cbold%7BJ%7D%5ET%5Cbold%7Bf%7D></div>  with 
<div align=center><img src=https://latex.codecogs.com/gif.latex?%5Cmu%20%5Cge%200></div>

The ![](https://latex.codecogs.com/gif.latex?%5Cmu) is decided by update strategy:
<div align=center><img src=https://latex.codecogs.com/gif.latex?%5Crho%20%3D%20%5Cfrac%7BF%28%5Cbold%7Bx%7D%29%20-%20F%28%5Cbold%7Bx%7D%20&plus;%20%5CDelta%20%5Cbold%7Bx%7D%29%7D%7B%5Cfrac%7B1%7D%7B2%7D%20%5CDelta%20%5Cbold%7Bx%7D%5ET%28%5Cmu%20%5CDelta%20%5Cbold%7Bx%7D%20-%20%5Cbold%7BJ%7D%5ET%5Cbold%7Bf%7D%29%7D></div>

+ if ![](https://latex.codecogs.com/gif.latex?%5Crho%20%3E%200) and is a big number, reduce ![](https://latex.codecogs.com/gif.latex?%5Cmu), let LM be more similiar to Gauss-Newton to accelerate the convergence.
+ if ![](https://latex.codecogs.com/gif.latex?%5Crho%20%3C%200) and is a small nunber, enlarge ![](https://latex.codecogs.com/gif.latex?%5Cmu) and reduce the iteration step.  

**The value of ![](https://latex.codecogs.com/gif.latex?%5Cmu) during the iteration:**
<div align=center><img src=https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise3_Curve_Fitting_LM/doc/mu.png></div>

**The optimization result:**
<div align=center><img src=https://github.com/lbw0502/Visual_Inertial_SLAM_Course/blob/master/exercise3_Curve_Fitting_LM/doc/opt_result.png></div>


