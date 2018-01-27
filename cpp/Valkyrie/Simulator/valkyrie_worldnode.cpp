#include "valkyrie_worldnode.hpp"
#include "Interface.hpp"

ValkyrieWorldNode::ValkyrieWorldNode(const dart::simulation::WorldPtr & world_) :
    dart::gui::osg::WorldNode(world_) {

    mSkel = world_->getSkeleton("valkyrie");
    mDof = mSkel->getNumDofs();
    mQ = Eigen::VectorXd::Zero(mDof);
    mQdot = Eigen::VectorXd::Zero(mDof);
    mActuatedTorque = Eigen::VectorXd::Zero(mDof - 6);
    mTorqueCommand = Eigen::VectorXd::Zero(mDof);
    mInterface = new Interface();
}

ValkyrieWorldNode::~ValkyrieWorldNode() {}

void ValkyrieWorldNode::customPreStep() {
    mQ = mSkel->getPositions();
    mQdot = mSkel->getVelocities();
    mInterface->getCommand(mQ, mQdot, mActuatedTorque);
    mTorqueCommand.tail(mDof - 6) = mActuatedTorque;

    mSkel->setForces(mTorqueCommand);
}
