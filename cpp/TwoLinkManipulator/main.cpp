#include <dart/dart.hpp>
#include "mywindow.hpp"
#include "ControlSet/jpos_controller.hpp"

using namespace dart::dynamics;
using namespace dart::simulation;

void printModel(const SkeletonPtr& twoLinkage) {

    std::cout << " ========= BodyNode Properties ========= "<< std::endl;
    for (int i = 0; i < twoLinkage->getNumBodyNodes(); ++i) {
        BodyNodePtr bn = twoLinkage->getBodyNode(i);
        std::cout << "[Name] \n " << bn->getName() << std::endl;
        std::cout << "[Mass] \n " << bn->getMass() << std::endl;
        std::cout << "[CoM] \n " << bn->getCOM()  << std::endl;
        std::cout << "[Linear Jacobian at BodyNode] \n " <<
            bn->getLinearJacobian() << std::endl;
        std::cout << "[Angular Jacobian at BodyNode] \n " <<
            bn->getAngularJacobian() << std::endl;
        std::cout << "[Linear Jacobian at CoM] \n " <<
            bn->getLinearJacobian(Eigen::Vector3d(0., 0., 0.5)) << std::endl;
        std::cout << "[Generalized Jacobian] \n" <<
            bn->getJacobian(Frame::World()) << std::endl;
        std::cout << "====================================" << std::endl;
    }
    std::cout << " ========= Joint Properties ========= "<< std::endl;
    for (int i = 0; i < twoLinkage->getNumDofs(); ++i) {
        DegreeOfFreedom* dof = twoLinkage->getDof(i);
        std::cout << "[Name] \n" << dof->getName() << std::endl;
        std::cout << "[Stiffness] \n " <<  dof->getSpringStiffness() << std::endl;
        std::cout << "[Damping] \n " <<  dof->getDampingCoefficient() << std::endl;
        std::cout << "====================================" << std::endl;
    }

}

void buildRobot(const SkeletonPtr& twoLinkage) {
    //// Add root BodyNode
    // Joint properties
    RevoluteJoint::Properties properties;
    properties.mName = "0";
    properties.mAxis = Eigen::Vector3d::UnitY();
    properties.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0, 0, 0);
    properties.mRestPositions[0] = 0.0;
    properties.mSpringStiffnesses[0] = 0.0;
    properties.mDampingCoefficients[0] = 0.0;
    //properties.mDampingCoefficients[0] = 5.0;
    // Create BodyNode pointer
    BodyNodePtr root_bn = twoLinkage->createJointAndBodyNodePair<RevoluteJoint>(
            nullptr, properties, BodyNode::AspectProperties("1")).second;
    // Joint shape
    std::shared_ptr<CylinderShape> cyl(new CylinderShape(0.1, 0.3));
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.linear() = dart::math::eulerXYZToMatrix(Eigen::Vector3d
            (0.5 * M_PI, 0.0, 0.0));
    auto shapeNode = root_bn->createShapeNodeWith<VisualAspect>(cyl);
    shapeNode->getVisualAspect()->setColor(dart::Color::Blue());
    shapeNode->setRelativeTransform(tf);
    // Link shape
    std::shared_ptr<BoxShape> box(new BoxShape(Eigen::Vector3d(0.1, 0.3, 1.0)));
    shapeNode = root_bn->createShapeNodeWith<VisualAspect, CollisionAspect,
              DynamicsAspect>(box);
    tf = Eigen::Isometry3d::Identity();
    tf.translation() = Eigen::Vector3d(0., 0., 0.5);
    shapeNode->getVisualAspect()->setColor(dart::Color::Red());
    shapeNode->setRelativeTransform(tf);
    root_bn->setLocalCOM(Eigen::Vector3d(0., 0., 0.5));

    /// Add first BodyNode
    // Joint properties
    properties.mName = "1";
    properties.mAxis = Eigen::Vector3d::UnitY();
    properties.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0., 0., 1.);
    properties.mRestPositions[0] = 0.0;
    properties.mSpringStiffnesses[0] = 0.0;
    //properties.mDampingCoefficients[0] = 3.0;
    properties.mDampingCoefficients[0] = 0.0;
    // Create BodyNode pointer
    BodyNodePtr bn = twoLinkage->createJointAndBodyNodePair<RevoluteJoint>(
            root_bn, properties, BodyNode::AspectProperties("2")).second;
    // Joint shape
    tf = Eigen::Isometry3d::Identity();
    tf.linear() = dart::math::eulerXYZToMatrix(Eigen::Vector3d
            (0.5 * M_PI, 0.0, 0.0));
    shapeNode = bn->createShapeNodeWith<VisualAspect>(cyl);
    shapeNode->getVisualAspect()->setColor(dart::Color::Blue());
    shapeNode->setRelativeTransform(tf);
    // Link shape
    shapeNode = bn->createShapeNodeWith<VisualAspect, CollisionAspect,
              DynamicsAspect>(box);
    tf = Eigen::Isometry3d::Identity();
    tf.translation() = Eigen::Vector3d(0., 0., 0.5);
    shapeNode->getVisualAspect()->setColor(dart::Color::Red());
    shapeNode->setRelativeTransform(tf);
    bn->setLocalCOM(Eigen::Vector3d(0., 0., 0.5));
}

int main(int argc, char *argv[])
{
    //// Create an empty Skeleton with the name
    SkeletonPtr twoLinkage = Skeleton::create("twoLinkage");

    //// Build Robot in skeleton
    buildRobot(twoLinkage);

    // [Practice 1] : change initial pos
    twoLinkage->getDof(0)->setPosition(-30. * M_PI / 180.);
    twoLinkage->getDof(1)->setPosition(30. * M_PI / 180.);

    // [Practice 2] : print out model (mass, joint stiffness, damping, jacobian, com)
    //printModel(twoLinkage);

    //// Create a world and add the skeleton to the world
    WorldPtr world(new World);
    world->addSkeleton(twoLinkage);

    //// Create a window for rendering the world
    // [Practice 4] : Controller
    MyWindow window(new JPosController(twoLinkage));
    //Eigen::Vector3d gravity(0.0, 0.0, -9.81);
    Eigen::Vector3d gravity(0.0, 0.0, 0.0);
    world->setGravity(gravity);
    world->setTimeStep(1.0/1000);
    window.setWorld(world);

    // Initialize glut, initialize the window, and begin the glut event loop
    // TODO : Camera adjustment
    glutInit(&argc, argv);
    window.initWindow( 1280, 960, "Two Linkage Tutorial" );
    glutMainLoop();

    return 0;
}
