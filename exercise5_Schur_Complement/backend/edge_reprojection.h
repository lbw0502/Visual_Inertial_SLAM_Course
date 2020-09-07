#ifndef MYSLAM_BACKEND_VISUALEDGE_H
#define MYSLAM_BACKEND_VISUALEDGE_H

#include <memory>
#include <string>

#include <Eigen/Dense>

#include "backend/eigen_types.h"
#include "backend/edge.h"

namespace myslam {
namespace backend {

/**
 * visual reprojection error
 * connected edges: Inverse depth vertex of landmark (InveseDepth), the first camera pose vertex which observes this landmark (T_World_From_Body1)
 *                  the camera pose vertex which oberserves this landmark (T_World_From_Body2)
 * The ordering of verticies_ has to be: InveseDepth, T_World_From_Body1, T_World_From_Body2
 */
class EdgeReprojection : public Edge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeReprojection(const Vec3 &pts_i, const Vec3 &pts_j)
        : Edge(2, 3, std::vector<std::string>{"VertexInverseDepth", "VertexPose", "VertexPose"}) {
        pts_i_ = pts_i;
        pts_j_ = pts_j;
    }

    /// return edge type
    virtual std::string TypeInfo() const override { return "EdgeReprojection"; }

    /// compute residual
    virtual void ComputeResidual() override;

    /// compute Jacobian
    virtual void ComputeJacobians() override;

    void SetTranslationImuFromCamera(Eigen::Quaterniond &qic_, Vec3 &tic_);

private:
    //Translation imu from camera
    Qd qic;
    Vec3 tic;

    //measurements
    Vec3 pts_i_, pts_j_;
};

/**
*
* vsiual reprojection error
* 
* connected verticies: landmark's coordinate under world frame XYZ,
*                      the camera pose vertex which observes the landmark (T_World_From_Body1)
* the ordering of verticies_ has to be: XYZ, T_World_From_Body1
*
*/
class EdgeReprojectionXYZ : public Edge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeReprojectionXYZ(const Vec3 &pts_i)
        : Edge(2, 2, std::vector<std::string>{"VertexXYZ", "VertexPose"}) {
        obs_ = pts_i;
    }

    /// return edge type
    virtual std::string TypeInfo() const override { return "EdgeReprojectionXYZ"; }

    /// compute residual
    virtual void ComputeResidual() override;

    /// compute Jacobian
    virtual void ComputeJacobians() override;

    void SetTranslationImuFromCamera(Eigen::Quaterniond &qic_, Vec3 &tic_);

private:
    //Translation imu from camera
    Qd qic;
    Vec3 tic;

    //measurements
    Vec3 obs_;
};

/**
 * only compoute the camera pose in reprojection
 */
class EdgeReprojectionPoseOnly : public Edge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeReprojectionPoseOnly(const Vec3 &landmark_world, const Mat33 &K) :
        Edge(2, 1, std::vector<std::string>{"VertexPose"}),
        landmark_world_(landmark_world), K_(K) {}

    /// return edge type
    virtual std::string TypeInfo() const override { return "EdgeReprojectionPoseOnly"; }

    /// compute residual
    virtual void ComputeResidual() override;

    /// compute Jacobian
    virtual void ComputeJacobians() override;

private:
    Vec3 landmark_world_;
    Mat33 K_;
};

}
}

#endif
