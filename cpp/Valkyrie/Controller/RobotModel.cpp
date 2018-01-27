#include "RobotModel.hpp"

RobotModel* RobotModel::getRobotModel() {
    static RobotModel robot;
    return & robot;
}

RobotModel::RobotModel() {
    dart::io::DartLoader urdfLoader;
    m_skel = urdfLoader.parseSkeleton("/Users/junhyeok/Repository/MotionCtrl/cpp/Valkyrie/Simulator/RobotModel/valkyrie.urdf");

    printf("[Robot Model] Constructed\n");
}

RobotModel::~RobotModel() {}

void RobotModel::updateModel(const Eigen::VectorXd & q_,
                             const Eigen::VectorXd & qdot_) {
    m_skel->setPositions(q_);
    m_skel->setVelocities(qdot_);
}

dart::dynamics::SkeletonPtr RobotModel::getSkeleton() {
    return m_skel;
}

int RobotModel::getNumDofs() {
    return m_skel->getNumDofs();
}
