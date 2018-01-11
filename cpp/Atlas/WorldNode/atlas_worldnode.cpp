#include "atlas_worldnode.hpp"

AtlasWorldNode::AtlasWorldNode(const dart::simulation::WorldPtr & world_) :
    dart::gui::osg::WorldNode(world_) {

    //mCtrl = new JPosController();
}

//AtlasWorldNode::~AtlasWorldNode() {}

void AtlasWorldNode::customPreStep() {
    //mCtrl->update();
}
