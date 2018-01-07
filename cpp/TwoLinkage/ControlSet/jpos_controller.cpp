#include "jpos_controller.hpp"

JPosController::JPosController(dart::dynamics::SkeletonPtr _skeleton):
                                                        Controller(_skeleton) {
    mJPosTask = new JPosTask(mSkeleton, mSkeleton->getNumDofs());
    //mWBDC = new WBDC();
}

JPosController::~JPosController() {}

void JPosController::_initialize() {
    mTorqueCommand.setZero();
}

void JPosController::_computeCommand() {
    mTaskList.clear();
    _jPosTaskSetup();
    _jPosCtrl();
}


void JPosController::_jPosTaskSetup() {
    Eigen::VectorXd qdes(mSkeleton->getNumDofs());
    Eigen::VectorXd qdotdes(mSkeleton->getNumDofs());
    Eigen::VectorXd qddotdes(mSkeleton->getNumDofs());
    qdes << 0., 0.;
    qdotdes << 0., 0.;
    qddotdes << 0., 0.;
    mJPosTask->updateTask(qdes, qdotdes, qddotdes);
}

void JPosController::_jPosCtrl() {
    Eigen::VectorXd cmd(mSkeleton->getNumDofs());
    Eigen::VectorXd Cg   = mSkeleton->getCoriolisAndGravityForces();
    mJPosTask->getCommand(cmd);
    mTorqueCommand = cmd + Cg;
}
