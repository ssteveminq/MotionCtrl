#ifndef KRWORLDNODE_H
#define KRWORLDNODE_H

#include <dart/dart.hpp>
#include <dart/io/io.hpp>
#include <dart/gui/osg/osg.hpp>

#include "controller.hpp"

class ColWorldNode : public dart::gui::osg::WorldNode
{
private:
    Controller* mCtrl;
    dart::simulation::WorldPtr mWorld;

public:
    ColWorldNode(const dart::simulation::WorldPtr & world);
    virtual ~ColWorldNode();

    void customPreStep() override;
};

#endif /* KRWORLDNODE_H */
