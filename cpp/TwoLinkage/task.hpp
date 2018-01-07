#ifndef TASK_H
#define TASK_H

#include <Eigen/Dense>
#include <dart/dart.hpp>

class Task
{
protected:
    virtual bool _updateCommand(const Eigen::VectorXd & _pos_des,
                                const Eigen::VectorXd & _vel_des,
                                const Eigen::VectorXd & _acc_des) = 0;
    virtual bool _updateTaskJacobian() = 0;
    virtual bool _updateTaskJdotQdot() = 0;

    dart::dynamics::SkeletonPtr mSkeleton;
    Eigen::VectorXd mOPCmd;
    Eigen::VectorXd mJdotQdot;
    Eigen::MatrixXd mJ;
    int mTaskDim;
    bool mIsTaskSet;

public:
    Task(dart::dynamics::SkeletonPtr _skel, int dim) : mTaskDim(dim),
                                                       mOPCmd(dim),
                                                       mIsTaskSet(false) {
        mSkeleton = _skel;
    }

    virtual ~Task() {};

    void getCommand(Eigen::VectorXd & _opCmd) { _opCmd = mOPCmd; }
    void getTaskJacobian(Eigen::MatrixXd & _j) { _j = mJ; }
    void getTaskJdotQdot(Eigen::MatrixXd & _jdotQdot) { _jdotQdot = mJdotQdot; }
    bool updateTask(const Eigen::VectorXd& _pos_des,
                    const Eigen::VectorXd & _vel_des,
                    const Eigen::VectorXd & _acc_des) {
        _updateCommand(_pos_des, _vel_des, _acc_des);
        _updateTaskJacobian();
        _updateTaskJdotQdot();
        mIsTaskSet = true;
        return true;
    }
    bool IsTaskSet() { return mIsTaskSet; }
    int getDim() { return mTaskDim; }
};

#endif /* TASK_H */
