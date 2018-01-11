#include "kr_worldnode.hpp"

KRWorldNode::KRWorldNode(const dart::simulation::WorldPtr & world_) :
    dart::gui::osg::WorldNode(world_) {

    //mCtrl = new JPosController();
}

//KRWorldNode::~KRWorldNode() {}

void KRWorldNode::customPreStep() {
    //mCtrl->update();
}
