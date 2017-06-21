//
// Created by joshua on 4/20/17.
//
#include <vector>
#include <map>
#include "sim_env/Box2DWorld.h"
#include "sim_env/Box2DIOUtils.h"
#include "Box2D/Box2D.h"
#include "sim_env/YamlUtils.h"
#include <boost/filesystem.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/format.hpp>

using namespace sim_env;
namespace bg = boost::geometry;

////////////////////////////////////////////////////////////////////////////////
/////////////////////* Definition of Box2DLink members *//////////////////////
////////////////////////////////////////////////////////////////////////////////
Box2DLink::Box2DLink(const Box2DLinkDescription &link_desc, Box2DWorldPtr world, bool is_static):
    _destroyed(false) {
    _name = link_desc.name;
    _world = world;
    // Create Box2D body
    std::shared_ptr<b2World> box2d_world = world->getRawBox2DWorld();
    b2BodyDef body_def;
    body_def.allowSleep = true;
    body_def.awake = true;
    body_def.type = is_static ? b2_staticBody : b2_dynamicBody;
    _body = box2d_world->CreateBody(&body_def);
    // run over polygons and create Box2D shape definitions + compute area
    float area = 0.0f;
    std::vector<b2PolygonShape> shape_defs;
    for (auto &polygon : link_desc.polygons) {
        b2PolygonShape b2_shape; // shape type for box2d
        std::vector<b2Vec2> b2_polygon; // temporal buffer for vertices
        // boost polygon for area, 'false' template parameter stands for counter clockwise (box2d standard)
        bg::model::polygon <bg::model::d2::point_xy<float>, false> boost_polygon;
        assert(polygon.size() % 2 == 0);
        // run over this polygon
        for (unsigned int i = 0; i < polygon.size() / 2; ++i) {
            // scale it
            float x = world->getScale() * polygon.at(2 * i);
            float y = world->getScale() * polygon.at(2 * i + 1);
            // create put vertex in buffer and in boost polygon
            b2_polygon.push_back(b2Vec2(x, y));
            bg::append(boost_polygon, bg::model::d2::point_xy<float>(x, y));
        }
        // compute the area
        area += bg::area(boost_polygon);
        // add the shape
        b2_shape.Set(b2_polygon.data(), (int32) b2_polygon.size());
        shape_defs.push_back(b2_shape);
    }
    assert(area > 0.0);
    // Now that we have the area and all shapes, we can create fixtures
    for (b2PolygonShape &shape_def : shape_defs) {
        b2FixtureDef fixture_def;
        fixture_def.isSensor = false;
        fixture_def.density = link_desc.mass / area;
        fixture_def.friction = link_desc.contact_friction;
        fixture_def.restitution = link_desc.restitution;
        fixture_def.shape = &shape_def;
        _body->CreateFixture(&fixture_def);
    }
    // Finally create a friction joint between this link and the ground plane
    float gravity = world->getGravity();
    b2FrictionJointDef friction_joint_def;
    friction_joint_def.localAnchorA.SetZero();
    friction_joint_def.localAnchorB.SetZero();
    friction_joint_def.bodyA = world->getGroundBody();
    friction_joint_def.bodyB = _body;
    friction_joint_def.collideConnected = false;
    friction_joint_def.maxForce = link_desc.trans_friction * _body->GetMass() * gravity;
    friction_joint_def.maxTorque = link_desc.rot_friction * _body->GetMass() * gravity;
    _friction_joint = box2d_world->CreateJoint(&friction_joint_def);
}

Box2DLink::~Box2DLink() {
    if (!_destroyed) {
        throw std::logic_error("[Box2DLink::~Box2DLink] Destructor called without prior object destruction.");
    }
}

void Box2DLink::destroy(const std::shared_ptr<b2World>& b2world) {
    b2world->DestroyJoint(_friction_joint);
    b2world->DestroyBody(_body);
    _destroyed = true;
}

bool Box2DLink::checkCollision(CollidableConstPtr other) const {
    // TODO implement me
    return false;
}

bool Box2DLink::checkCollision(const std::vector<CollidableConstPtr> &others) const {
    // TODO  implement me
    return false;
}

std::string Box2DLink::getName() const {
    return _name;
}

void Box2DLink::setName(const std::string &name) {
    _name = name;
}

EntityType Box2DLink::getType() const {
    return EntityType::Link;
}

Eigen::Affine3f Box2DLink::getTransform() const {
    b2Vec2 pos = _body->GetPosition();
    float orientation = _body->GetAngle();
    Eigen::Affine3f transform;
    Box2DWorldPtr world = getBox2DWorld();
    transform = Eigen::Translation3f(world->getInverseScale() * pos.x,
                                   world->getInverseScale() * pos.y,
                                   0.0);
    transform.rotate(Eigen::AngleAxisf(orientation, Eigen::Vector3f::UnitZ()));
    return transform;
}

Box2DWorldPtr Box2DLink::getBox2DWorld() const {
    if (_world.expired()) {
        throw std::logic_error("[Box2DLink::getBox2DWorld] Can not access Box2DWorld. A link should not exist without a world.");
    }
    return _world.lock();
}

WorldPtr Box2DLink::getWorld() const {
    return getBox2DWorld();
}

WorldConstPtr Box2DLink::getConstWorld() const {
    return getWorld();
}

b2Body *Box2DLink::getBody() {
    return _body;
}

void Box2DLink::registerChildJoint(Box2DJointPtr joint) {
//    auto position_in_list = std::find(_child_joints.begin(), _child_joints.end(), joint);
//    if (position_in_list == _child_joints.end()) {
    _child_joints.push_back(Box2DJointWeakPtr(joint));
//    }
}

void Box2DLink::registerParentJoint(Box2DJointPtr joint) {
//    auto position_in_list = std::find(_parent_joints.begin(), _parent_joints.end(), joint);
//    if (position_in_list == _parent_joints.end()) {
    _parent_joints.push_back(Box2DJointWeakPtr(joint));
//    }
}


////////////////////////////////////////////////////////////////////////////////
/////////////////////* Definition of Box2DJoint members *///////////////////////
////////////////////////////////////////////////////////////////////////////////

Box2DJoint::Box2DJoint(const Box2DJointDescription &joint_desc, Box2DLinkPtr link_a, Box2DLinkPtr link_b,
                       Box2DWorldPtr world):_destroyed(false) {
    _world = world;
    _name = joint_desc.name;
    _link_a = link_a;
    _link_b = link_b;
    assert(link_a->getName() == joint_desc.link_a);
    assert(link_b->getName() == joint_desc.link_b);
    // Now create a box2d joint
    std::shared_ptr<b2World> box2d_world = world->getRawBox2DWorld();
    if (joint_desc.joint_type == "revolute") {
        _joint_type = JointType::Revolute;
    } else if (joint_desc.joint_type == "prismatic") {
        _joint_type = JointType::Prismatic;
    } else {
        std::stringstream ss;
        ss << "Unknown joint type " << joint_desc.joint_type << ". Using revolute instead.";
        world->getLogger()->logErr(ss.str(), "[Box2DJoint::Box2DJoint]");
        _joint_type = JointType::Revolute;
    }
    switch (_joint_type) {
        // TODO this only works if the links are placed correctly
        // TODO it would be easiest if this was some kind of invariant, i.e. during construction all
        // TODO links are placed at their desired position. Maybe by making the constructor protected?
        // TODO and the object class a friend?
        case JointType::Revolute: {
            b2RevoluteJointDef joint_def;
            b2Vec2 box2d_axis(world->getScale() * joint_desc.axis[0],
                              world->getScale() * joint_desc.axis[1]);
            joint_def.Initialize(link_a->getBody(), link_b->getBody(), box2d_axis);
            joint_def.lowerAngle = joint_desc.limits[0];
            joint_def.upperAngle = joint_desc.limits[1];
            joint_def.enableLimit = joint_desc.limits.norm() > 0.0;
            joint_def.maxMotorTorque = joint_desc.max_torque;
            joint_def.enableMotor = joint_desc.actuated;
            _joint = box2d_world->CreateJoint(&joint_def);
            break;
        }
        case JointType::Prismatic: {
            b2PrismaticJointDef joint_def;
            b2Vec2 box2d_axis(joint_desc.axis[0], joint_desc.axis[1]);
            b2Vec2 box2d_direction(std::cos(joint_desc.axis_orientation), std::sin(joint_desc.axis_orientation));
            joint_def.Initialize(link_a->getBody(), link_b->getBody(), box2d_axis, box2d_direction);
            joint_def.lowerTranslation = world->getScale() * joint_desc.limits[0];
            joint_def.upperTranslation = world->getScale() * joint_desc.limits[1];
            joint_def.enableLimit = joint_desc.limits.norm() > 0.0;
            joint_def.maxMotorForce = joint_desc.max_torque;
            joint_def.enableMotor = joint_desc.actuated;
            _joint = box2d_world->CreateJoint(&joint_def);
        }
    }
}

Box2DJoint::~Box2DJoint() {
    if (!_destroyed) {
        throw std::logic_error("[Box2DJoint::~Box2DJoint] Destructor called without prior object destruction.");
    }
}

void Box2DJoint::destroy(const std::shared_ptr<b2World>& b2world) {
    b2world->DestroyJoint(_joint);
    _destroyed = true;
}

float Box2DJoint::getPosition() const {
    switch (_joint_type) {
        case JointType::Revolute: {
            b2RevoluteJoint* revolute_joint = static_cast<b2RevoluteJoint*>(_joint);
            revolute_joint->GetJointAngle();
        }
        case JointType::Prismatic: {
            b2PrismaticJoint* prismatic_joint = static_cast<b2PrismaticJoint*>(_joint);
            prismatic_joint->GetJointTranslation();
        }
    }
    return 0;
}

void Box2DJoint::setPosition(float v) {
    switch (_joint_type) {
        case JointType::Revolute: {
            b2RevoluteJoint* revolute_joint = static_cast<b2RevoluteJoint*>(_joint);
//            revolute_joint->S();
        }
        case JointType::Prismatic: {
            b2PrismaticJoint* prismatic_joint = static_cast<b2PrismaticJoint*>(_joint);
//            prismatic_joint->GetJointTranslation();
        }
    }
}

float Box2DJoint::getVelocity() const {
    return 0;
}

void Box2DJoint::setVelocity(float v) {

}

unsigned int Box2DJoint::getIndex() const {
    return 0;
}

Joint::JointType Box2DJoint::getJointType() const {
    return _joint_type;
}

std::string Box2DJoint::getName() const {
    return _name;
}

void Box2DJoint::setName(const std::string &name) {
    _name = name;
}

EntityType Box2DJoint::getType() const {
    return EntityType::Joint;
}

Eigen::Affine3f Box2DJoint::getTransform() const {
    // TODO do we need to keep the kinematic tree updated here? or can we get this from box2d
    return Eigen::Affine3f();
}

WorldPtr Box2DJoint::getWorld() const {
    return getBox2DWorld();
}

WorldConstPtr Box2DJoint::getConstWorld() const {
    return getWorld();
}

Box2DWorldPtr Box2DJoint::getBox2DWorld() const {
    if (_world.expired()) {
        throw std::logic_error("[Box2DJoint::getBox2DWorld] Can not access Box2DWorld. A joint should not exist without a world.");
    }
    return _world.lock();
}
////////////////////////////////////////////////////////////////////////////////
/////////////////////* Definition of Box2DObject members *//////////////////////
////////////////////////////////////////////////////////////////////////////////

Box2DObject::Box2DObject(const Box2DObjectDescription &obj_desc, Box2DWorldPtr world):_destroyed(false) {
    _name = obj_desc.name;
    _is_static = obj_desc.is_static;
    for (auto &link_desc : obj_desc.links) {
        Box2DLinkPtr link(new Box2DLink(link_desc, world, _is_static));
        _links[link->getName()] = link;
    }

    for (auto &joint_desc : obj_desc.joints) {
        Box2DLinkPtr link_a = _links[joint_desc.link_a];
        Box2DLinkPtr link_b = _links[joint_desc.link_b];
        Box2DJointPtr joint(new Box2DJoint(joint_desc, link_a, link_b, world));
        link_a->registerChildJoint(joint);
        link_b->registerParentJoint(joint);
        _joints[joint->getName()] = joint;
    }
}

Box2DObject::~Box2DObject() {
    if (!_destroyed) {
        throw std::logic_error("[Box2DObject::~Box2DObject] Object destructor called before destroy()");
    }
}

void Box2DObject::destroy(const std::shared_ptr<b2World>& b2world) {
    for (std::pair<const std::string, Box2DJointPtr>& joint_iter : _joints) {
        joint_iter.second->destroy(b2world);
    }
    _joints.clear(); // first delete all joints
    for (std::pair<const std::string, Box2DLinkPtr>& link_iter : _links) {
        link_iter.second->destroy(b2world);
    }
    _links.clear(); // then delete all links
    _destroyed = true;
}

std::string Box2DObject::getName() const {
    return _name;
}

EntityType Box2DObject::getType() const {
    return EntityType::Object;
}

Eigen::Affine3f Box2DObject::getTransform() const {
    // TODO read position from Box2d -> x,y and rotation around z.
    // TODO other components should be 0.0
    return Eigen::Affine3f();
}

void Box2DObject::setTransform(const Eigen::Affine3f &tf) {

}

WorldPtr Box2DObject::getWorld() const {
    return getBox2DWorld();
}

WorldConstPtr Box2DObject::getConstWorld() const {
    return getWorld();
}

Box2DWorldPtr Box2DObject::getBox2DWorld() const {
    Box2DWorldPtr world = _world.lock();
    if (!world) {
        throw std::logic_error("[Box2DObject::getBox2DWorld] Could not access Box2DWorld. This object should not exist anymore.");
    }
    return world;
}

bool Box2DObject::checkCollision(CollidableConstPtr other_object) const {
    return false;
}

bool Box2DObject::checkCollision(const std::vector<CollidableConstPtr> &object_list) const {
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

Eigen::VectorXf Box2DObject::getDOFPositions(const Eigen::VectorXi &indices) const {
    return Eigen::VectorXf();
}

void Box2DObject::setDOFPositions(const Eigen::VectorXf &values, const Eigen::VectorXi &indices) {

}

Eigen::VectorXf Box2DObject::getDOFVelocities(const Eigen::VectorXi &indices) const {
    return Eigen::VectorXf();
}

void Box2DObject::setDOFVelocities(const Eigen::VectorXf &values, const Eigen::VectorXi &indices) {

}

void Box2DObject::setName(const std::string &name) {
    _name = name;
}

bool Box2DObject::isStatic() const {
    return _is_static;
}

void Box2DObject::getLinks(std::vector<LinkPtr> links) {
    for (auto &iter : _links) {
        links.push_back(iter.second);
    }
}

void Box2DObject::getLinks(std::vector<LinkConstPtr> links) const {
    for (auto &iter : _links) {
        links.push_back(iter.second);
    }
}

void Box2DObject::getJoints(std::vector<JointPtr> joints) {
    for (auto &iter : _joints) {
        joints.push_back(iter.second);
    }
}

void Box2DObject::getJoints(std::vector<JointConstPtr> joints) const {
    for (auto &iter : _joints) {
        joints.push_back(iter.second);
    }
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////* Definition of Box2DRobot members *///////////////////////
////////////////////////////////////////////////////////////////////////////////
Box2DRobot::Box2DRobot(const Box2DObjectDescription &robot_desc, Box2DWorldPtr world):_destroyed(false) {
    // Up till now a robot is just a semantically special object. Rather than implementing
    // the same functionalities twice (for object and robot), we use composition here.
    _robot_object = Box2DObjectPtr(new Box2DObject(robot_desc, world));
}

Box2DRobot::~Box2DRobot() {
    if (not _destroyed) {
        throw std::logic_error("[Box2DRobot::~Box2DRobot] Object destructor called before destroy()");
    }
}

void Box2DRobot::destroy(const std::shared_ptr<b2World>& b2world) {
    _robot_object->destroy(b2world);
    _destroyed = true;
}

std::string Box2DRobot::getName() const {
    return _robot_object->getName();
}

void Box2DRobot::setName(const std::string &name) {
    _robot_object->setName(name);
}

void Box2DRobot::setTransform(const Eigen::Affine3f &tf) {
    _robot_object->setTransform(tf);
}

void Box2DRobot::setActiveDOFs(const Eigen::VectorXi &indices) {
    _robot_object->setActiveDOFs(indices);
}

Eigen::VectorXi Box2DRobot::getActiveDOFs() {
    return _robot_object->getActiveDOFs();
}

Eigen::VectorXi Box2DRobot::getDOFIndices() {
    return _robot_object->getDOFIndices();
}

Eigen::VectorXf Box2DRobot::getDOFPositions(const Eigen::VectorXi &indices) const {
    return _robot_object->getDOFPositions(indices);
}

void Box2DRobot::setDOFPositions(const Eigen::VectorXf &values, const Eigen::VectorXi &indices) {
    _robot_object->setDOFPositions(values, indices);
}

Eigen::VectorXf Box2DRobot::getDOFVelocities(const Eigen::VectorXi &indices) const {
    _robot_object->getDOFVelocities(indices);
}

void Box2DRobot::setDOFVelocities(const Eigen::VectorXf &values, const Eigen::VectorXi &indices) {
    _robot_object->setDOFVelocities(values, indices);
}

bool Box2DRobot::isStatic() const {
    return _robot_object->isStatic();
}

bool Box2DRobot::checkCollision(CollidableConstPtr other) const {
    return _robot_object->checkCollision(other);
}

bool Box2DRobot::checkCollision(const std::vector<CollidableConstPtr> &others) const {
    return _robot_object->checkCollision(others);
}

EntityType Box2DRobot::getType() const {
    return EntityType::Robot;
}

Eigen::Affine3f Box2DRobot::getTransform() const {
    return _robot_object->getTransform();
}

WorldPtr Box2DRobot::getWorld() const {
    return _robot_object->getWorld();
}

WorldConstPtr Box2DRobot::getConstWorld() const {
    return _robot_object->getConstWorld();
}

void Box2DRobot::getLinks(std::vector<LinkPtr> links) {
    _robot_object->getLinks(links);
}

void Box2DRobot::getLinks(std::vector<LinkConstPtr> links) const {
    _robot_object->getLinks(links);
}

void Box2DRobot::getJoints(std::vector<JointPtr> joints) {
    _robot_object->getJoints(joints);
}

void Box2DRobot::getJoints(std::vector<JointConstPtr> joints) const {
    _robot_object->getJoints(joints);
}


////////////////////////////////////////////////////////////////////////////////
/////////////////////* Definition of Box2DWorld members *///////////////////////
////////////////////////////////////////////////////////////////////////////////

Box2DWorld::Box2DWorld() : _world(nullptr), _b2_ground_body(nullptr) {
}

Box2DWorld::~Box2DWorld() {
    eraseWorld();
}

void Box2DWorld::loadWorld(const std::string &path) {
    Box2DEnvironmentDescription env_desc = Box2DEnvironmentDescription();
    parseYAML(path, env_desc);
    eraseWorld();
    createWorld(env_desc);
}

RobotPtr Box2DWorld::getRobot(const std::string &name) const {
    if (_robots.count(name)) {
        return _robots.at(name);
    }
    return nullptr;
}

ObjectPtr Box2DWorld::getObject(const std::string &name) const {
    if (_objects.count(name)) {
        return _objects.at(name);
    }
    return nullptr;
}

void Box2DWorld::getObjects(std::vector<ObjectPtr> &objects, bool exclude_robots) const {
    if (!exclude_robots) {
        std::vector<RobotPtr> robots;
        getRobots(robots);
        objects.insert(objects.end(), robots.begin(), robots.end());
    }
    for (auto &obj_map_iter : _objects) {
        objects.push_back(obj_map_iter.second);
    }
}

void Box2DWorld::getRobots(std::vector<RobotPtr> &robots) const {
    for (auto &robot_map_iter : _robots) {
        robots.push_back(robot_map_iter.second);
    }
}

void Box2DWorld::stepPhysics(int steps) const {
    // TODO
}

bool Box2DWorld::supportsPhysics() const {
    return true;
}

void Box2DWorld::setPhysicsTimeStep(float physics_step) {

}

void Box2DWorld::getPhysicsTimeStep() const {

}

WorldViewerPtr Box2DWorld::getViewer() {
    return nullptr;
}

LoggerPtr Box2DWorld::getLogger() {
    return DefaultLogger::getInstance();
}

float Box2DWorld::getScale() const {
    return _scale;
}

float Box2DWorld::getInverseScale() const {
    if (_scale == 0.0f) {
        return 1.0f;
    }
    return 1.0f / _scale;
}

float Box2DWorld::getGravity() const {
    return GRAVITY * getScale();
}

////////////////////////////// PROTECTED FUNCTIONS //////////////////////////////
std::shared_ptr<b2World> Box2DWorld::getRawBox2DWorld() {
    return _world;
}

b2Body* Box2DWorld::getGroundBody() {
    return _b2_ground_body;
}

////////////////////////////// PRIVATE FUNCTIONS //////////////////////////////
void Box2DWorld::eraseWorld() {
    for (auto& robot_iter : _robots) {
        robot_iter.second->destroy(_world);
    }
    _robots.clear();
    for (auto& object_iter : _objects) {
        object_iter.second->destroy(_world);
    }
    _objects.clear();
    if (_b2_ground_body) {
        _world->DestroyBody(_b2_ground_body);
        _b2_ground_body = nullptr;
    }
    _world.reset();
    _scale = 1.0;
}

void Box2DWorld::createNewObject(const Box2DObjectDescription &object_desc) {
    Box2DObjectPtr object(new Box2DObject(object_desc, shared_from_this()));
    if (_objects.count(object->getName())) {
        std::string new_name(object->getName());
        new_name = new_name + std::to_string(_objects.size());
        object->setName(new_name);
    }
    _objects[object->getName()] = object;
}

void Box2DWorld::createNewRobot(const Box2DObjectDescription &robot_desc) {
    Box2DRobotPtr robot(new Box2DRobot(robot_desc, shared_from_this()));
    if (_robots.count(robot->getName())) {
        std::string new_name(robot->getName());
        new_name = new_name + std::to_string(_robots.size());
        robot->setName(new_name);
    }
    _robots[robot->getName()] = robot;
}

void Box2DWorld::createGround(const Eigen::Vector4f &world_bounds) {
    if (_b2_ground_body) {
        _world->DestroyBody(_b2_ground_body);
        _b2_ground_body = nullptr;
    }
    // Create ground body
    b2BodyDef ground_body_def;
    ground_body_def.type = b2_staticBody;
    ground_body_def.position.Set(0.5f * (world_bounds[0] + world_bounds[2]),
                                 0.5f * (world_bounds[1] + world_bounds[3]));
    _b2_ground_body = _world->CreateBody(&ground_body_def);
    b2FixtureDef fd;
    b2PolygonShape ground_shape;
    ground_shape.SetAsBox(world_bounds[2] - world_bounds[0], world_bounds[3] - world_bounds[1]);
    fd.shape = &ground_shape;
    fd.isSensor = true;
    _b2_ground_body->CreateFixture(&fd);
}

void Box2DWorld::createWorld(const Box2DEnvironmentDescription &env_desc) {
    _world.reset(new b2World(b2Vec2(0.0, 0.0)));
    _scale = env_desc.scale;
    // Get the dimension of the ground plane
    Eigen::Vector4f world_bounds(env_desc.world_bounds);
    if (world_bounds.norm() == 0.0) {
        world_bounds << float(GROUND_DEFAULT_MIN_X), float(GROUND_DEFAULT_MIN_Y),
                float(GROUND_DEFAULT_MAX_X), float(GROUND_DEFAULT_MAX_Y);
        world_bounds = _scale * world_bounds;
    }
    createGround(world_bounds);
    for (auto &robot_desc : env_desc.robots) {
        createNewRobot(robot_desc);
    }
    for (auto &object_desc : env_desc.objects) {
        createNewObject(object_desc);
    }
    for (auto &state_desc : env_desc.states) {
        ObjectPtr object = getObject(state_desc.first);
        if (object == nullptr) {
            object = getRobot(state_desc.first);
            if (object == nullptr) {
                std::string err_msg = boost::str(boost::format("Could not find object %1 in scene. Skipping this...")
                                                 % state_desc.first);
                _logger->logErr(err_msg,
                                "sim_env/Box2DWorld.cpp::createWorld");
                continue;
            }
        }
        object->setDOFPositions(state_desc.second.configuration);
        object->setDOFVelocities(state_desc.second.velocity);
    }
}


