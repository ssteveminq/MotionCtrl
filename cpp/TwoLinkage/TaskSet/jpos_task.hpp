#ifndef JPOS_TASK_H
#define JPOS_TASK_H

#include "task.hpp"
#include "state_provider.hpp"

class JPosTask: public Task
{
protected:
    virtual bool _updateCommand(const Eigen::VectorXd & _pos_des,
                                const Eigen::VectorXd & _vel_des,
                                const Eigen::VectorXd & _acc_des);
    virtual bool _updateTaskJacobian();
    virtual bool _updateTaskJdotQdot();

    StateProvider* mSP;

public:
    JPosTask(dart::dynamics::SkeletonPtr _skel, int _dim);
    virtual ~JPosTask();

    Eigen::VectorXd kp;
    Eigen::VectorXd kd;
};

#endif /* JPOS_TASK_H */
