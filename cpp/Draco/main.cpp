#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/io/io.hpp>
#include <dart/io/urdf/urdf.hpp>

#include "draco_worldnode.hpp"

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

    std::cout<<"MK code added test"<<std::endl;

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

int main() {
    //// Generate world and add skeletons
    dart::simulation::WorldPtr world(new dart::simulation::World);
    dart::io::DartLoader urdfLoader;
    dart::dynamics::SkeletonPtr ground = urdfLoader.parseSkeleton(
            //"/Users/junhyeok/Repository/MotionCtrl/cpp/Draco/RobotModel/ground.urdf");
            "/home/mk/loco/MotionCtrl/cpp/Draco/RobotModel/ground.urdf");
    dart::dynamics::SkeletonPtr robot = urdfLoader.parseSkeleton(
            //"/Users/junhyeok/Repository/MotionCtrl/cpp/Draco/RobotModel/draco.urdf");
            "/home/mk/loco/MotionCtrl/cpp/Draco/RobotModel/draco.urdf");
    //world->addSkeleton(ground);
    world->addSkeleton(robot);
    //Eigen::Vector3d gravity(0.0, 0.0, -9.81);
    Eigen::Vector3d gravity(0.0, 0.0, 0.0);
    world->setGravity(gravity);
    world->setTimeStep(1.0/1000);

    // Initial configuration

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
    viewer.simulate(true);
    std::cout << viewer.getInstructions() << std::endl;
    viewer.setUpViewInWindow(0, 0, 640, 480);
    viewer.getCameraManipulator()->setHomePosition(::osg::Vec3( 2.57,  3.14, 1.64),
            ::osg::Vec3( 0.00,  0.00, 0.00),
            ::osg::Vec3(-0.24, -0.25, 0.94));
    viewer.setCameraManipulator(viewer.getCameraManipulator());
    viewer.run();
}
