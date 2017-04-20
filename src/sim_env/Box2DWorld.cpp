//
// Created by joshua on 4/20/17.
//
#include "sim_env/Box2DWorld.h"
#include "Box2D/Box2D.h"

using namespace sim_env;

////////////////////////////////////////////////////////////////////////////////
/////////////////////* Definition of Box2DObject members *//////////////////////
////////////////////////////////////////////////////////////////////////////////

Box2DObject::Box2DObject(const std::string &object_description, Box2DWorldPtr world) {

}

Box2DObject::~Box2DObject() {

}

std::string Box2DObject::getName() const {
    return _name;
}

SimEnvEntityType Box2DObject::getType() const {
    return SimEnvEntityType::Object;
}

Eigen::Affine3d Box2DObject::getTransform() const {
    // TODO read position from Box2d -> x,y and rotation around z.
    // TODO other components should be 0.0
    return Eigen::Affine3d();
}

void Box2DObject::setTransform(const Eigen::Affine3d &tf) {

}

WorldPtr Box2DObject::getWorld() const {
    return _world;
}

WorldConstPtr Box2DObject::getConstWorld() const {
    return sim_env::WorldConstPtr(_world);
}

bool Box2DObject::checkCollision(ObjectConstPtr other_object) const {
    return false;
}

bool Box2DObject::checkCollision(const std::vector<ObjectConstPtr> &object_list) const {
    return false;
}

void Box2DObject::setActiveDOFs(const Eigen::VectorXi &indices) {

}

Eigen::VectorXi Box2DObject::getActiveDOFs() {
    return Eigen::VectorXi();
}

Eigen::VectorXi Box2DObject::getDOFIndices() {
    return Eigen::VectorXi();
}

Eigen::VectorXd Box2DObject::getDOFPositions(const Eigen::VectorXi &indices) const {
    return Eigen::VectorXd();
}

void Box2DObject::setDOFPositions(const Eigen::VectorXd &values, const Eigen::VectorXi &indices) {

}

Eigen::VectorXd Box2DObject::getDOFVelocities(const Eigen::VectorXi &indices) const {
    return Eigen::VectorXd();
}

void Box2DObject::setDOFVelocities(const Eigen::VectorXd &values, const Eigen::VectorXi &indices) {

}

////////////////////////////////////////////////////////////////////////////////
/////////////////////* Definition of Box2DRobot members *///////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/////////////////////* Definition of Box2DWorld members *///////////////////////
////////////////////////////////////////////////////////////////////////////////
Box2DWorld::Box2DWorld() {

}

Box2DWorld::~Box2DWorld() {

}

void Box2DWorld::loadWorld(const std::string &path) {

}

RobotPtr Box2DWorld::getRobot(const std::string &name) const {
    return nullptr;
}

ObjectPtr Box2DWorld::getObject(const std::string &name) const {
    return nullptr;
}

void Box2DWorld::getObjects(std::vector<ObjectPtr> &objects, bool exclude_robots) const {

}

void Box2DWorld::getRobots(std::vector<RobotPtr> &robots) const {

}

void Box2DWorld::stepPhysics(int steps) const {

}

bool Box2DWorld::supportsPhysics() const {
    return false;
}

void Box2DWorld::setPhysicsTimeStep(double physics_step) {

}

void Box2DWorld::getPhysicsTimeStep() const {

}

WorldViewerPtr Box2DWorld::getViewer() {
    return nullptr;
}
