#ifndef INTERFACE_H
#define INTERFACE_H

#include <Eigen/Dense>

//class Test;
class StateEstimator;

class Interface
{
private:
    //Test* m_test;
    double m_time;
    double m_initTime;
    StateEstimator* m_stateEstimator;

public:
    Interface();
    virtual ~Interface();

    void getCommand(const Eigen::VectorXd & q_,
                    const Eigen::VectorXd & qdot_,
                    Eigen::VectorXd & u_);
};

#endif /* INTERFACE_H */
