//
// Created by hyj on 18-11-11.
//
#include <iostream>
#include <vector>
#include <random>  
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

struct Pose
{
    Pose(Eigen::Matrix3d R, Eigen::Vector3d t):Rwc(R),qwc(R),twc(t) {};
    Eigen::Matrix3d Rwc;
    Eigen::Quaterniond qwc;
    Eigen::Vector3d twc;

    Eigen::Vector2d uv;
};
int main()
{

    int poseNums = 10;
    double radius = 8;
    double fx = 1.;
    double fy = 1.;
    std::vector<Pose> camera_pose;
    for(int n = 0; n < poseNums; ++n ) {
        double theta = n * 2 * M_PI / ( poseNums * 4);
        // rotate around z-axis
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d t = Eigen::Vector3d(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
        camera_pose.push_back(Pose(R,t));
    }

    // generate a random landmark
    std::default_random_engine generator;
    std::uniform_real_distribution<double> xy_rand(-4, 4.0);
    std::uniform_real_distribution<double> z_rand(8., 10.);
    double tx = xy_rand(generator);
    double ty = xy_rand(generator);
    double tz = z_rand(generator);

    Eigen::Vector3d Pw(tx, ty, tz);
    // this landmark can be observed since frame 3
    int start_frame_id = 3;
    int end_frame_id = poseNums;
    for (int i = start_frame_id; i < end_frame_id; ++i) {
        Eigen::Matrix3d Rcw = camera_pose[i].Rwc.transpose();
        Eigen::Vector3d Pc = Rcw * (Pw - camera_pose[i].twc);

        double x = Pc.x();
        double y = Pc.y();
        double z = Pc.z();

        camera_pose[i].uv = Eigen::Vector2d(x/z,y/z);
    }
    
    // use triangulation to estimate depth information
    Eigen::Vector3d P_est;
    P_est.setZero();

    int frame_num = end_frame_id - start_frame_id;
    Eigen::MatrixXd D(frame_num*2,4);

    for(int i=start_frame_id, j=0; i<end_frame_id; i++, j++){
        double u  = camera_pose[i].uv[0];
        double v  = camera_pose[i].uv[1];

        Eigen::MatrixXd P(3,4);
        P.block<3,3>(0,0) = camera_pose[i].Rwc.transpose();
        P.block<3,1>(0,3) = -camera_pose[i].Rwc.transpose() * camera_pose[i].twc;

        D.row(j*2) = u*P.row(2) - P.row(0);
        D.row(j*2+1) = v*P.row(2) - P.row(1);
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(D, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Vector4d v4 = svd.matrixV().col(3);
    P_est << v4(0)/v4(3), v4(1)/v4(3), v4(2)/v4(3);
    
    std::cout <<"ground truth: \n"<< Pw.transpose() <<std::endl;
    std::cout <<"your result: \n"<< P_est.transpose() <<std::endl;
    return 0;
}
