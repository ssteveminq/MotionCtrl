#ifndef VALKYRIE_WORLD_NODE
#define VALKYRIE_WORLD_NODE

#include <dart/dart.hpp>
#include <dart/io/io.hpp>
#include <dart/gui/osg/osg.hpp>
#include <Eigen/Dense>

class Interface;

class ValkyrieWorldNode : public dart::gui::osg::WorldNode
{
private:
    Interface* mInterface;
    dart::dynamics::SkeletonPtr mSkel;

    Eigen::VectorXd mQ;
    Eigen::VectorXd mQdot;
    Eigen::VectorXd mActuatedTorque;
    Eigen::VectorXd mTorqueCommand;
    int mDof;

public:
    ValkyrieWorldNode(const dart::simulation::WorldPtr & world);
    virtual ~ValkyrieWorldNode();

    void customPreStep() override;
};

#endif /* VALKYRIE_WORLD_NODE */
