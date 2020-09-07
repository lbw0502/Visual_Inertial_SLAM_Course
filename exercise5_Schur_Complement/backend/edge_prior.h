//
// Created by heyijia on 19-1-30.
//

#ifndef SLAM_COURSE_EDGE_PRIOR_H
#define SLAM_COURSE_EDGE_PRIOR_H

#include <memory>
#include <string>

#include <Eigen/Dense>

#include "backend/eigen_types.h"
#include "backend/edge.h"


namespace myslam {
namespace backend {

/**
* EdgeSE3Prior，此边为 1 元边，与之相连的顶点有：Ti
* connected edge: Ti
*/
class EdgeSE3Prior : public Edge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeSE3Prior(const Vec3 &p, const Qd &q) :
            Edge(6, 1, std::vector<std::string>{"VertexPose"}),
            Pp_(p), Qp_(q) {}

    virtual std::string TypeInfo() const override { return "EdgeSE3Prior"; }

    virtual void ComputeResidual() override;

    virtual void ComputeJacobians() override;


private:
    Vec3 Pp_;   // pose prior
    Qd   Qp_;   // Rotation prior
};

}
}


#endif //SLAM_COURSE_EDGE_PRIOR_H
