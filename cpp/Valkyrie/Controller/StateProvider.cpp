#include "StateProvider.hpp"
#include "RobotModel.hpp"

StateProvider* StateProvider::getStateProvider() {
    static StateProvider sp;
    return &sp;
}

StateProvider::StateProvider() {
    int dof(RobotModel::getRobotModel()->getNumDofs());
    q = Eigen::VectorXd::Zero(dof);
    qdot = Eigen::VectorXd::Zero(dof);

    printf("[State Provider] Constructed\n");
}

StateProvider::~StateProvider() {}
