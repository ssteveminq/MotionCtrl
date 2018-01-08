#include "controller.hpp"

Controller::Controller(dart::dynamics::SkeletonPtr _skeleton) {
    mTime = 0.0;
    mStepTime = 0.001;
    mSkeleton = _skeleton;
    mInitTime = 0.01;
    mTorqueCommand.resize(mSkeleton->getNumDofs());
    mStateEstimator = new StateEstimator(mSkeleton);
}

Controller::~Controller() {}

void Controller::update() {
    // [Practice 3] : print out pos, vel in configuration space & operational space
    //_printProperties();

    if (mTime < mInitTime) {
        mStateEstimator->initialization();
        _initialize();
    } else {
        // [Practice 4] Implement Controllers
        mStateEstimator->update();
        _computeCommand();
    }
    mSkeleton->setForces(mTorqueCommand);
    mTime += mStepTime;
}

dart::dynamics::SkeletonPtr Controller::getSkeleton() {
    return mSkeleton;
}

void Controller::_printProperties() {
    std::cout << " ========= BodyNode Properties ========= "<< std::endl;
    for (int i = 0; i < mSkeleton->getNumBodyNodes(); ++i) {
        dart::dynamics::BodyNodePtr bn = mSkeleton->getBodyNode(i);
        std::cout << "[Cartesian Pos] \n" <<
            bn->getTransform().translation() << std::endl;
        std::cout << "====================================" << std::endl;
    }
    for (int i = 0; i < mSkeleton->getNumBodyNodes(); ++i) {
        std::cout << " ========= Joint Properties ========= "<< std::endl;
        for (int i = 0; i < mSkeleton->getNumDofs(); ++i) {
            dart::dynamics::DegreeOfFreedom* dof = mSkeleton->getDof(i);
            std::cout << "[Pos] \n" << dof->getPosition() << std::endl;
            std::cout << "[Vel] \n" << dof->getVelocity() << std::endl;
            std::cout << "====================================" << std::endl;
        }
    }
}
