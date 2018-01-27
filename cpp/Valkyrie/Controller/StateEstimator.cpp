#include "StateEstimator.hpp"
#include "StateProvider.hpp"
#include "RobotModel.hpp"

StateEstimator::StateEstimator() {
    m_robot = RobotModel::getRobotModel();
    m_sp = StateProvider::getStateProvider();

    printf("[State Estimator] constructed\n");
}

StateEstimator::~StateEstimator() {}

void StateEstimator::initialize(const Eigen::VectorXd & q_,
                                const Eigen::VectorXd & qdot_) {
    m_sp->q = q_;
    m_sp->qdot = qdot_;
    m_robot->updateModel(m_sp->q, m_sp->qdot);
}

void StateEstimator::update(const Eigen::VectorXd & q_,
                            const Eigen::VectorXd & qdot_) {
    StateProvider::getStateProvider()->q = q_;
    StateProvider::getStateProvider()->qdot = qdot_;
    m_robot->updateModel(m_sp->q, m_sp->qdot);
}
