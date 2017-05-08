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

using namespace sim_env;
namespace bg = boost::geometry;

////////////////////////////////////////////////////////////////////////////////
/////////////////////* Definition of Box2DLink members *//////////////////////
////////////////////////////////////////////////////////////////////////////////
Box2DLink::Box2DLink(const Box2DLinkDescription &link_desc, Box2DWorldPtr world, bool is_static) {
    _name = link_desc.name;
    _world = world;
    // Create Box2D body
    std::shared_ptr<b2World> box2d_world = _world->getRawBox2DWorld();
    b2BodyDef body_def;
    body_def.allowSleep = true;
    body_def.awake = true;
    body_def.type = is_static ? b2_staticBody : b2_dynamicBody;
    _body = box2d_world->CreateBody(&body_def);
    // run over polygons and create Box2D shape definitions + compute area
    float area = 0.0f;
    std::vector<b2PolygonShape> shape_defs;
    for (auto& polygon : link_desc.polygons) {
        b2PolygonShape b2_shape; // shape type for box2d
        std::vector<b2Vec2> b2_polygon; // temporal buffer for vertices
        bg::model::polygon<bg::model::d2::point_xy<float> > boost_polygon; // boost polygon for area
        assert(polygon.size() % 2 == 0);
        // run over this polygon
        for (unsigned int i = 0; i < polygon.size(); ++i) {
            // scale it
            float x = _world->getScale() * polygon.at(2 * i);
            float y = _world->getScale() * polygon.at(2 * i + 1);
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
    for (b2PolygonShape& shape_def : shape_defs) {
        b2FixtureDef fixture_def;
        fixture_def.isSensor = false;
        fixture_def.density = link_desc.mass / area;
        fixture_def.friction = link_desc.contact_friction;
        fixture_def.restitution = link_desc.restitution;
        fixture_def.shape = &shape_def;
        _body->CreateFixture(&fixture_def);
    }
    // Finally create a friction joint between this link and the ground plane
    float gravity = _world->getGravity();
    b2FrictionJointDef friction_joint_def;
    friction_joint_def.localAnchorA.SetZero();
    friction_joint_def.localAnchorB.SetZero();
    friction_joint_def.bodyA = _world->getGroundBody();
    friction_joint_def.bodyB = _body;
    friction_joint_def.collideConnected = false;
    friction_joint_def.maxForce = link_desc.trans_friction * _body->GetMass() * gravity;
    friction_joint_def.maxTorque = link_desc.rot_friction * _body->GetMass() * gravity;
    _friction_joint = box2d_world->CreateJoint(&friction_joint_def);
}

Box2DLink::~Box2DLink() {
    std::shared_ptr<b2World> box2d_world = _world->getRawBox2DWorld();
    box2d_world->DestroyJoint(_friction_joint);
    box2d_world->DestroyBody(_body);
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

Eigen::Affine3d Box2DLink::getTransform() const {
    b2Vec2 pos = _body->GetPosition();
    float orientation = _body->GetAngle();
    Eigen::Transform transform(Eigen::Translation(_world->getInverseScale() * pos.x,
                                                  _world->getInverseScale() * pos.y,
                                                  0.0));
    transform.rotate(Eigen::AngleAxis(orientation, Eigen::Vector3f::UnitZ()));
    return transform.matrix();
}

WorldPtr Box2DLink::getWorld() const {
    return _world;
}

WorldConstPtr Box2DLink::getConstWorld() const {
    return _world;
}

b2Body* Box2DLink::getBody() {
    return _body;
}
////////////////////////////////////////////////////////////////////////////////
/////////////////////* Definition of Box2DObject members *//////////////////////
////////////////////////////////////////////////////////////////////////////////

Box2DObject::Box2DObject(const Box2DObjectDescription &obj_desc, Box2DWorldPtr world) {
    _name = obj_desc.name;
    _is_static = obj_desc.is_static;
    for (auto& link_desc : obj_desc.links) {
        Box2DLinkPtr link = std::make_shared<Box2DLink>(link_desc, world, _is_static);
        _links[link->getName()] = link;
    }

    for (auto& joint_desc : obj_desc.joints) {
        Box2DLinkPtr link_a = _links[joint_desc.link_a];
        Box2DLinkPtr link_b = _links[joint_desc.link_b];
        Box2DJointPtr joint = std::make_shared<Box2DJoint>(obj_desc, world);
        _joints[joint->getName()] = joint;
    }
}

Box2DObject::~Box2DObject() {
}

std::string Box2DObject::getName() const {
    return _name;
}

EntityType Box2DObject::getType() const {
    return EntityType::Object;
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

void Box2DObject::setName(const std::string &name) {

}

bool Box2DObject::isStatic() const {
    return _is_static;
}


////////////////////////////////////////////////////////////////////////////////
/////////////////////* Definition of Box2DRobot members *///////////////////////
////////////////////////////////////////////////////////////////////////////////

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

////////////////////////////// PRIVATE FUNCTIONS //////////////////////////////
void Box2DWorld::eraseWorld() {
    _robots.clear();
    _objects.clear();
    if (_b2_ground_body) {
        _world->DestroyBody(_b2_ground_body);
        _b2_ground_body = nullptr;
    }
    _world.reset(nullptr);
    _scale = 1.0;
}

void Box2DWorld::createNewObject(const Box2DObjectDescription &object_desc) {
    Box2DObjectPtr object = std::make_shared<Box2DObject>(object_desc, shared_from_this());
    if (_objects.count(object->getName())) {
        std::string new_name(object->getName());
        new_name = new_name + std::to_string(_objects.size());
        object->setName(new_name);
    }
    _objects[object->getName()] = object;
}

void Box2DWorld::createNewRobot(const Box2DObjectDescription &robot_desc) {
    Box2DRobotPtr robot = std::make_shared<Box2DRobot>(robot_desc, shared_from_this());
    if (_robots.count(robot->getName())) {
        std::string new_name(robot->getName());
        new_name = new_name + std::to_string(_robots.size());
        robot->setName(new_name);
    }
    _robots[robot->getName()] = robot;
}

void Box2DWorld::createGround(const Eigen::Vector4f& world_bounds) {
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

void Box2DWorld::createWorld(const Box2DEnvironmentDescription& env_desc) {
    _world.reset(new b2World(b2Vec2(0.0, 0.0)));
    _scale = env_desc.scale;
    // Get the dimension of the ground plane
    Eigen::Vector4f world_bounds(env_desc.world_bounds);
    if (world_bounds.norm() == 0.0) {
        world_bounds << GROUND_DEFAULT_MIN_X, GROUND_DEFAULT_MIN_Y,
                        GROUND_DEFAULT_MAX_X, GROUND_DEFAULT_MAX_Y;
        world_bounds = _scale * world_bounds;
    }
    createGround(world_bounds);
    for (auto& robot_desc : env_desc.robots) {
        createNewRobot(robot_desc);
    }
    for (auto& object_desc : env_desc.objects) {
        createNewObject(object_desc);
    }
    for (auto& state_desc : env_desc.states) {
        ObjectPtr object = getObject(state_desc.first);
        object->setDOFPositions(state_desc.second.configuration);
        object->setDOFVelocities(state_desc.second.velocity);
    }
}


////////////////////////////////////////////////////////////////////////////////
/////////////////////* Definition of Box2DJoint members *///////////////////////
////////////////////////////////////////////////////////////////////////////////

Box2DJoint::Box2DJoint(const Box2DJointDescription& joint_desc, Box2DLinkPtr link_a, Box2DLinkPtr link_b,
                       Box2DWorldPtr world) {
    _world = world;
    _name = joint_desc.name;
    _link_a = link_a;
    _link_b = link_b;
    assert(link_a->getName() == joint_desc.link_a);
    assert(link_b->getName() == joint_desc.link_b);
    // Now create a box2d joint
    std::shared_ptr<b2World> box2d_world = _world->getRawBox2DWorld();
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
            b2Vec2 box2d_axis(_world->getScale() * joint_desc.axis[0],
                              _world->getScale() * joint_desc.axis[1]);
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
            // TODO we need a reference point here for the prismatic joint -> need to extend yaml
            joint_def.Initialize(link_a->getBody(), link_b->getBody(), box2d_axis);
            joint_def.lowerTranslation = _world->getScale() * joint_desc.limits[0];
            joint_def.upperTranslation = _world->getScale() * joint_desc.limits[1];
            joint_def.enableLimit = joint_desc.limits.norm() > 0.0;
            joint_def.maxMotorForce = joint_desc.max_torque;
            joint_def.enableMotor = joint_desc.actuated;
            _joint = box2d_world->CreateJoint(&joint_def);
        }
    }
}

Box2DJoint::~Box2DJoint() {
    std::shared_ptr<b2World> box2d_world = _world->getRawBox2DWorld();
    box2d_world->DestroyJoint(_joint);
}

float Box2DJoint::getPosition() const {
    return 0;
}

void Box2DJoint::setPosition(float v) {

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
    return nullptr;
}

std::string Box2DJoint::getName() const {
    return nullptr;
}

void Box2DJoint::setName(const std::string &name) {

}

EntityType Box2DJoint::getType() const {
    return nullptr;
}

Eigen::Transform Box2DJoint::getTransform() const {
    return nullptr;
}

WorldPtr Box2DJoint::getWorld() const {
    return nullptr;
}

WorldConstPtr Box2DJoint::getConstWorld() const {
    return nullptr;
}

