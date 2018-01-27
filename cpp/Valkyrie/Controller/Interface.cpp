#include "Interface.hpp"
#include "StateEstimator.hpp"
//#include "BalanceTest.hpp"

Interface::Interface(): m_time(0.), m_initTime(0.01) {
    m_stateEstimator = new StateEstimator();
    //m_test = new BalanceTest();

    printf("[Interface] Constructed\n");
}

Interface::~Interface() {
    delete m_stateEstimator;
    //delete m_test;
}

void Interface::getCommand(const Eigen::VectorXd & q_,
                           const Eigen::VectorXd & qdot_,
                           Eigen::VectorXd & u_) {

    if (m_time < m_initTime) {
        m_stateEstimator->initialize(q_, qdot_);
        //m_test = Initialize();
    } else {
        m_stateEstimator->update(q_, qdot_);
        //m_test->GetTorqueInput(u_);
    }
    m_time += 0.001;
}
