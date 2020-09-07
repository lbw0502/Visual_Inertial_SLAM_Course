#include <iostream>
#include <random>
#include "backend/vertex_inverse_depth.h"
#include "backend/vertex_pose.h"
#include "backend/edge_reprojection.h"
#include "backend/problem.h"

using namespace myslam::backend;
using namespace std;

/*
 * Frame : stores pose and observation of each camera frame
 */
struct Frame {
    Frame(Eigen::Matrix3d R, Eigen::Vector3d t) : Rwc(R), qwc(R), twc(t) {};
    Eigen::Matrix3d Rwc;
    Eigen::Quaterniond qwc;
    Eigen::Vector3d twc;

    // observed features and their id
    unordered_map<int, Eigen::Vector3d> featurePerId; 
};

/*
 * generate simulated data under world frame: camera pose, feature points, and oberservation for each frame
 */
void GetSimDataInWordFrame(vector<Frame> &cameraPoses, vector<Eigen::Vector3d> &points) {

    // number of feature, suppose each camera frame can observe all features
    int featureNums = 20;

    // number of camera frames
    int poseNums = 3;

    double radius = 8;
    for (int n = 0; n < poseNums; ++n) {
        double theta = n * 2 * M_PI / (poseNums * 4);

        // rotate around z-axis
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d t = Eigen::Vector3d(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
        cameraPoses.push_back(Frame(R, t));
    }

    // generate 3D landmark
    std::default_random_engine generator;
    std::normal_distribution<double> noise_pdf(0., 1. / 1000.);  // 2pixel / focal
    for (int j = 0; j < featureNums; ++j) {
        std::uniform_real_distribution<double> xy_rand(-4, 4.0);
        std::uniform_real_distribution<double> z_rand(4., 8.);

        Eigen::Vector3d Pw(xy_rand(generator), xy_rand(generator), z_rand(generator));
        points.push_back(Pw);

        // observation of each frame
        for (int i = 0; i < poseNums; ++i) {
            Eigen::Vector3d Pc = cameraPoses[i].Rwc.transpose() * (Pw - cameraPoses[i].twc);
            Pc = Pc / Pc.z();
            Pc[0] += noise_pdf(generator);
            Pc[1] += noise_pdf(generator);
            cameraPoses[i].featurePerId.insert(make_pair(j, Pc));
        }
    }
}

int main() {
    // prepare the simulated data
    vector<Frame> cameras;
    vector<Eigen::Vector3d> points;
    GetSimDataInWordFrame(cameras, points);
    Eigen::Quaterniond qic(1, 0, 0, 0);
    Eigen::Vector3d tic(0, 0, 0);

    // construct problem
    Problem problem(Problem::ProblemType::SLAM_PROBLEM);

    // add camera pose verticies
    vector<shared_ptr<VertexPose> > vertexCams_vec;
    for (size_t i = 0; i < cameras.size(); ++i) {
        shared_ptr<VertexPose> vertexCam(new VertexPose());
        Eigen::VectorXd pose(7);
        pose << cameras[i].twc, cameras[i].qwc.x(), cameras[i].qwc.y(), cameras[i].qwc.z(), cameras[i].qwc.w();
        vertexCam->SetParameters(pose);

       if(i < 2)
           vertexCam->SetFixed();

        problem.AddVertex(vertexCam);
        vertexCams_vec.push_back(vertexCam);
    }

    // add landmark verticies and edges
    std::default_random_engine generator;
    std::normal_distribution<double> noise_pdf(0, 1.);
    double noise = 0;
    vector<double> noise_invd;
    vector<shared_ptr<VertexInverseDepth> > allPoints;
    for (size_t i = 0; i < points.size(); ++i) {

        // use inverse_depth to represent landmark
        Eigen::Vector3d Pw = points[i];
        Eigen::Vector3d Pc = cameras[0].Rwc.transpose() * (Pw - cameras[0].twc);
        noise = noise_pdf(generator);
        double inverse_depth = 1. / (Pc.z() + noise);
        noise_invd.push_back(inverse_depth);

        // initialize landmark vertex (in form of inverse depth)
        shared_ptr<VertexInverseDepth> verterxPoint(new VertexInverseDepth());
        VecX inv_d(1);
        inv_d << inverse_depth;
        verterxPoint->SetParameters(inv_d);
        problem.AddVertex(verterxPoint);
        allPoints.push_back(verterxPoint);

        // add edge between camera pose vertex and landmark vertex
        for (size_t j = 1; j < cameras.size(); ++j) {
            Eigen::Vector3d pt_i = cameras[0].featurePerId.find(i)->second;
            Eigen::Vector3d pt_j = cameras[j].featurePerId.find(i)->second;
            shared_ptr<EdgeReprojection> edge(new EdgeReprojection(pt_i, pt_j));
            edge->SetTranslationImuFromCamera(qic, tic);

            // connected verticies of current edge
            std::vector<std::shared_ptr<Vertex> > edge_vertex;
            edge_vertex.push_back(verterxPoint);
            edge_vertex.push_back(vertexCams_vec[0]);
            edge_vertex.push_back(vertexCams_vec[j]);
            edge->SetVertex(edge_vertex);

            problem.AddEdge(edge);
        }
    }

    problem.Solve(5);

    std::cout << "\nCompare MonoBA results after opt..." << std::endl;
    for (size_t k = 0; k < allPoints.size(); k+=1) {
        std::cout << "after opt, point " << k << " : gt " << 1. / points[k].z() << " ,noise "
                  << noise_invd[k] << " ,opt " << allPoints[k]->Parameters() << std::endl;
    }
    std::cout<<"------------ pose translation ----------------"<<std::endl;
    for (int i = 0; i < vertexCams_vec.size(); ++i) {
        std::cout<<"translation after opt: "<< i <<" :"<< vertexCams_vec[i]->Parameters().head(3).transpose() << " || gt: "<<cameras[i].twc.transpose()<<std::endl;
    }

    problem.TestMarginalize();

    return 0;
}

