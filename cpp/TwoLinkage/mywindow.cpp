#include "mywindow.hpp"
#include <iostream>

MyWindow::MyWindow(Controller* _controller) : SimWindow(),
                                              mController(_controller) {
}

MyWindow::~MyWindow() {}

void MyWindow::timeStepping() {
    mController->update();
    mWorld->step();
}

void MyWindow::drawWorld() const
{
    SimWindow::drawWorld();
}
