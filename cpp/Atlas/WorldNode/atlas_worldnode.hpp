#ifndef KRWORLDNODE_H
#define KRWORLDNODE_H

#include <dart/dart.hpp>
#include <dart/io/io.hpp>
#include <dart/gui/osg/osg.hpp>

#include "controller.hpp"

class AtlasWorldNode : public dart::gui::osg::WorldNode
{
private:
    Controller* mCtrl;

public:
    AtlasWorldNode(const dart::simulation::WorldPtr & world);
    //virtual ~AtlasWorldNode();

    void customPreStep() override;
};

#endif /* KRWORLDNODE_H */
