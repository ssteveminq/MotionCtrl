#ifndef ROBOTMODEL_H
#define ROBOTMODEL_H

#include <dart/dart.hpp>
#include <dart/io/io.hpp>
#include <dart/io/urdf/urdf.hpp>
#include <Eigen/Dense>

class RobotModel
{
private:
    RobotModel();

    dart::dynamics::SkeletonPtr m_skel;

public:
    static RobotModel* getRobotModel();
    virtual ~RobotModel();
    dart::dynamics::SkeletonPtr getSkeleton();
    int getNumDofs();

    void updateModel(const Eigen::VectorXd & q_, const Eigen::VectorXd & qdot_);
};

#endif /* ROBOTMODEL_H */
