#ifndef JPOS_CONTROLLER_H
#define JPOS_CONTROLLER_H

#include <dart/dart.hpp>
#include "controller.hpp"
#include "jpos_task.hpp"

class JPosController : public Controller
{
private:
    virtual void _initialize();
    virtual void _computeCommand();
    void _jPosTaskSetup();
    void _jPosCtrl();

    Task* mJPosTask;
    //WBDC* mWBDC;

public:
    JPosController(dart::dynamics::SkeletonPtr _skeleton);
    virtual ~JPosController();
};

#endif /* JPOS_CONTROLLER_H */
