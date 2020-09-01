#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/so3.hpp"
#include "sophus/se3.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char** argv){

    // rotate 90 degree around z-axis
    Matrix3d R = AngleAxisd(M_PI/2, Vector3d(0,0,1)).toRotationMatrix();
    Sophus::SO3d SO3_R(R);
    Quaterniond q(R);

    cout << "Rotation Matrix representation: " << endl;
    cout << SO3_R.matrix() << endl;
    cout << "Quaternion representation: " << endl;
    cout << q.toRotationMatrix() << endl;

    Vector3d w(0.01, 0.02, 0.03);


    Sophus::SO3d SO3_R_update = SO3_R * Sophus::SO3d::exp(w);

    Quaterniond q_small(1, w[0]/2, w[1]/2, w[2]/2);
    Quaterniond q_updated = (q * q_small).normalized();



    cout << "updat by Rotation Matrix: " << endl;
    cout << SO3_R_update.matrix() << endl;
    cout << "updat by Quaternion: " << endl;
    cout << q_updated.toRotationMatrix() << endl;

}