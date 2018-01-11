#include "col_worldnode.hpp"

ColWorldNode::ColWorldNode(const dart::simulation::WorldPtr & _world) :
    dart::gui::osg::WorldNode(_world) {
        mWorld = _world;
}

ColWorldNode::~ColWorldNode() {}

void ColWorldNode::customPreStep() {
    // Collision Example

    // Look through the collisions to see if the new object would start in
    // collision with something
    auto collisionEngine = mWorld->getConstraintSolver()->getCollisionDetector();
    dart::dynamics::SkeletonPtr ground = mWorld->getSkeleton(0);
    dart::dynamics::SkeletonPtr box = mWorld->getSkeleton(1);

    auto groundCol = collisionEngine->createCollisionGroup(ground.get());
    auto ballCol = collisionEngine->createCollisionGroup(box.get());

    dart::collision::CollisionOption option;
    dart::collision::CollisionResult result;
    bool collision = groundCol->collide(ballCol.get(), option, &result);

    //std::cout << box->getConstraintForces() << std::endl;
    std::cout << "======================" << std::endl;
    if (!collision) {
        std::cout << "No collision" << std::endl;
    } else {
        std::cout << "Num Contact : " << result.getNumContacts() << std::endl;
        for (int i = 0; i < result.getNumContacts(); ++i) {
            std::cout << i << "th Contact ++++++++" << std::endl;
            std::cout << "Contact Point : \n" << result.getContact(i).point << std::endl;
            std::cout << "Contact Force : \n" << result.getContact(i).force << std::endl;
        }
    }

}
