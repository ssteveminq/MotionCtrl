#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
//#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <stdio.h>
#include <iostream>
#include <string>
#include "draco_worldnode.hpp"

std::string GetCurrentWorkingDir(void)
{
    char buff[50];
    getcwd(buff,50);
    std::string current_working_dir(buff);
    return current_working_dir;
}

void setMeshColor(dart::dynamics::SkeletonPtr robot) {
    for(std::size_t i=0; i<robot->getNumBodyNodes(); ++i) {
        dart::dynamics::BodyNode* bn = robot->getBodyNode(i);
        auto shapeNodes = bn->getShapeNodesWith<dart::dynamics::VisualAspect>();
        for(auto shapeNode : shapeNodes) {
            std::shared_ptr<dart::dynamics::MeshShape> mesh =
                std::dynamic_pointer_cast<dart::dynamics::MeshShape>(
                        shapeNode->getShape());
            if(mesh)
                mesh->setColorMode(dart::dynamics::MeshShape::SHAPE_COLOR);
        }
    }
}

void _printModel(dart::dynamics::SkeletonPtr robot) {

    //for (int i = 0; i < robot->getNumBodyNodes(); ++i) {
        //dart::dynamics::BodyNodePtr bn = robot->getBodyNode(i);
        //std::cout << i << "th" << std::endl;
        //std::cout << bn->getName() << std::endl;
    //}

    //for (int i = 0; i < robot->getNumJoints(); ++i) {
        //dart::dynamics::Joint* joint = robot->getJoint(i);
        //std::cout << i << "th" << std::endl;
        //std::cout << joint->getNumDofs() << std::endl;
    //}

    //for (int i = 0; i < robot->getNumDofs(); ++i) {
        //dart::dynamics::DegreeOfFreedom* dof = robot->getDof(i);
        //std::cout << i << "th" << std::endl;
        //std::cout << dof->getName() << std::endl;
    //}

    std::cout << robot->getNumDofs() << std::endl;
    std::cout << robot->getNumJoints() << std::endl;
    std::cout << robot->getMassMatrix().rows() << std::endl;
    std::cout << robot->getMassMatrix().cols() << std::endl;
    //exit(0);
}

void _setInitialConfiguration(dart::dynamics::SkeletonPtr robot) {
    Eigen::VectorXd q(robot->getNumDofs());
    q.setZero();

    int num_joints=3;
    q[0] =0.000;

    Eigen::VectorXd idx_joint(num_joints);
     //lower body
    idx_joint[0] = robot->getDof("bodyPitch")->getIndexInSkeleton();
    idx_joint[1] = robot->getDof("kneePitch")->getIndexInSkeleton();
    idx_joint[2] = robot->getDof("ankle")->getIndexInSkeleton();

     //upper body
    Eigen::VectorXd config_joint(num_joints);
    config_joint << 0.2, -0.1, 1.5;

    for(int i(0); i<num_joints; i++)
        q[idx_joint[i]] = config_joint[i];    

    robot->setPositions(q);
}


int main() {

    /// getDirectory location of model file
    std::string current_path = GetCurrentWorkingDir(); 
    // remove "build" string from current path
    current_path = current_path.substr(0,current_path.find_last_of("\\/"));
    //std::string model_path=current_path +"/RobotModel/ground.urdf";
    //std::string robotmodel_path=current_path+"/RobotModel/draco.urdf";

    //// Generate world and add skeletons
    dart::simulation::WorldPtr world(new dart::simulation::World);
    //dart::io::DartLoader urdfLoader;
    dart::utils::DartLoader urdfLoader;

    //dart::dynamics::SkeletonPtr ground = urdfLoader.parseSkeleton(
    auto ground = urdfLoader.parseSkeleton("/home/mk/MotionCtrl/cpp/Draco/RobotModel/ground.urdf");
    auto robot= urdfLoader.parseSkeleton("/home/mk/MotionCtrl/cpp/Draco/RobotModel/mk_draco.urdf");
    world->addSkeleton(ground);
    world->addSkeleton(robot);
    Eigen::Vector3d gravity(0.0, 0.0, -9.81);

    world->setGravity(gravity);
    world->setTimeStep(1.0/1000);

    // Initial configuration
    _setInitialConfiguration(robot);
    // Print Information
    _printModel(robot);

    // Set mesh color
    setMeshColor(robot);

    //// Wrap a worldnode
    osg::ref_ptr<DracoWorldNode> node
        = new DracoWorldNode(world);
    node->setNumStepsPerCycle(20);

    //// Create viewer
    dart::gui::osg::Viewer viewer;
    viewer.addWorldNode(node);
    viewer.simulate(false);
    std::cout << viewer.getInstructions() << std::endl;
    viewer.setUpViewInWindow(0, 0, 640, 480);
    //viewer.getCameraManipulator()->setHomePosition(::osg::Vec3( 2.57,  3.14, 1.64),
    //viewer.getCameraManipulator()->setHomePosition(::osg::Vec3( 4.57,  5.14, 2.04),
    viewer.getCameraManipulator()->setHomePosition(::osg::Vec3( 0.57,  5.14, 2.04),
            ::osg::Vec3( 0.00,  0.00, 0.00),
            ::osg::Vec3(-0.55, -0.55, 0.95));
    viewer.setCameraManipulator(viewer.getCameraManipulator());
    viewer.run();
}
