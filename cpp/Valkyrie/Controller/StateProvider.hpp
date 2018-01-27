#ifndef STATEPROVIDER_H
#define STATEPROVIDER_H

#include <Eigen/Dense>

class StateProvider
{
private:
    StateProvider();

public:
    static StateProvider* getStateProvider();
    virtual ~StateProvider();

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
};

#endif /* STATEPROVIDER_H */
