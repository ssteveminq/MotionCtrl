#include "jpos_task.hpp"

JPosTask::JPosTask(dart::dynamics::SkeletonPtr _skel, int _dim) : Task(_skel, _dim),
                                                                  kp(_dim),
                                                                  kd(_dim) {
    mSP = StateProvider::getStateProvider();
    mJ = Eigen::MatrixXd(mTaskDim, mSkeleton->getNumDofs());
    mJdotQdot = Eigen::VectorXd(mTaskDim);
    for (int i = 0; i < mTaskDim; ++i) {
        kp[i] = 20.;
        kd[i] = 3.;
    }
}

JPosTask::~JPosTask() {}

bool JPosTask::_updateCommand(const Eigen::VectorXd & _pos_des,
                              const Eigen::VectorXd & _vel_des,
                              const Eigen::VectorXd & _acc_des) {
    for (int i = 0; i < mTaskDim; ++i) {
        mOPCmd[i] = _acc_des[i] + kp[i] * (_pos_des[i] - mSP->Q[i])
            + kd[i] * (_vel_des[i] - mSP->Qdot[i]);
    }
    return true;
}

bool JPosTask::_updateTaskJacobian() {
    mJ.setZero();
    mJ = Eigen::MatrixXd::Identity(mTaskDim, mTaskDim);
}

bool JPosTask::_updateTaskJdotQdot() {
    mJdotQdot.setZero();
    return true;
}
