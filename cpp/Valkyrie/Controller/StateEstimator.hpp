#ifndef STATEESTIMATOR_H
#define STATEESTIMATOR_H

#include <Eigen/Dense>

class StateProvider;
class RobotModel;

class StateEstimator
{
private:
    StateProvider* m_sp;
    RobotModel* m_robot;

public:
    StateEstimator();
    virtual ~StateEstimator();

    void initialize(const Eigen::VectorXd & q_,
                    const Eigen::VectorXd & qdot_);
    void update(const Eigen::VectorXd & q_,
                const Eigen::VectorXd & qdot_);
};

#endif /* STATEESTIMATOR_H */
