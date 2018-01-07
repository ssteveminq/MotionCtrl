#ifndef MYWINDOW_H
#define MYWINDOW_H

#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include "controller.hpp"

class MyWindow : public dart::gui::SimWindow
{
public:
    MyWindow(Controller* _controller);
    virtual ~MyWindow();

    void timeStepping() override;
    void drawWorld() const override;

private:
    Controller* mController;
};

#endif /* MYWINDOW_H */
