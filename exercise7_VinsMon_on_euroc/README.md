# Vins Mono on euroc dataset

### Licence

The source code is released under GPLv3 license.

We are still working on improving the code reliability. For any technical issues, please contact Yijia He <heyijia_2013@163.com> , Xiang Gao <https://github.com/gaoxiang12> or Huakun Cui <https://github.com/StevenCui>.

For commercial inquiries, please contact Song Zhao <zhaosong@shenlanxueyuan.com>


### dependencies

1. pangolin: <https://github.com/stevenlovegrove/Pangolin>

2. opencv

3. Eigen

4. Ceres

### build the project

```c++
cd exercise7_VinsMono_on_euroc
mkdir build 
cd build
cmake ..
make -j4
```

### run
#### 1. CurveFitting Example to Verify the Solver.
```c++
cd build
../bin/testCurveFitting 
```

#### 2. VINs-Mono on Euroc Dataset
```c++
cd build
../bin/run_euroc /home/bowen/dataset/MH-05/mav0/ ../config/
```

#### 3. Use EVO to evaluate the trajectory
The output trajectory is stored in file **pose_output.txt** in tum format  
`cd bin`  
1. align the space in the file  
`cat pose_output.txt | tr -s [:space:] > pose_output_new.txt`  
2. draw the trajectory  
`evo_traj tum pose_output_new.txt -p`  
3. paste the ground truth trajectory of MH-05 here as **groundtruth.csv**  
4. convert the ground truth data to tum format  
`evo_traj euroc groundtruth.csv --save_as_tum`  
5. compare the result of the ground truth and our trajectory  
`evo_ape tum groundtruth.tum pose_output_new.txt -va -p`  
<div align=center><img src=./doc/compare_result.png></div>
<div align=center><img src=./doc/compare_result2.png></div>



#### Note: evo installation
```
pip install evo --upgrade --no-binary evo
sudo apt-get install python3.5-tk
```
for python3.5

