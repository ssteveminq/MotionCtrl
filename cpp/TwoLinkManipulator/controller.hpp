#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <dart/dart.hpp>
#include "state_estimator.hpp"

class Task;

class Controller
{
protected:
    dart::dynamics::SkeletonPtr mSkeleton;
    double mInitTime;
    double mTime;
    double mStepTime;
    Eigen::VectorXd mTorqueCommand;

    virtual void _initialize() = 0;
    virtual void _computeCommand() = 0;
    void _printProperties();
    std::vector<Task*> mTaskList;
    StateEstimator* mStateEstimator;

public:
    Controller(dart::dynamics::SkeletonPtr _skeleton);
    virtual ~Controller();

    void update();
    dart::dynamics::SkeletonPtr getSkeleton();

};

#endif /* CONTROLLER_H */
