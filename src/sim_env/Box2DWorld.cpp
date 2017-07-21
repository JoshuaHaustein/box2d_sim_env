//
// Created by joshua on 4/20/17.
//
#include <vector>
#include <map>
#include <exception>
#include "sim_env/Box2DWorld.h"
#include "sim_env/Box2DWorldViewer.h"
#include "sim_env/YamlUtils.h"
#include <boost/filesystem.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/format.hpp>

using namespace sim_env;
namespace bg = boost::geometry;

////////////////////////////////////////////////////////////////////////////////
/////////////////////* Definition of Box2DLink members *////////////////////////
////////////////////////////////////////////////////////////////////////////////
Box2DLink::Box2DLink(const Box2DLinkDescription &link_desc, Box2DWorldPtr world,
                     bool is_static, const std::string& object_name):
    _destroyed(false) {
    Box2DWorldLock lock(world->world_mutex);
    _name = link_desc.name;
    _world = world;
    _object_name = object_name;
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

void Box2DLink::setObjectName(const std::string& name) {
    _object_name = name;
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

void Box2DLink::getPose(Eigen::Vector3f& pose) const {
    b2Vec2 pos = _body->GetPosition();
    Box2DWorldConstPtr world = getBox2DWorld();
    float orientation = _body->GetAngle();
    pose[0] = world->getInverseScale() * pos.x;
    pose[1] = world->getInverseScale() * pos.y;
    pose[2] = orientation;
}

Eigen::Vector3f Box2DLink::getPose() const {
    Eigen::Vector3f pose;
    getPose(pose);
    return pose;
}

void Box2DLink::getVelocityVector(Eigen::Vector3f& vel_vector) const {
    Box2DWorldConstPtr world = getBox2DWorld();
    b2Vec2 b2_vel = _body->GetLinearVelocity();
    vel_vector[0] = world->getInverseScale() * b2_vel.x;
    vel_vector[1] = world->getInverseScale() * b2_vel.y;
    vel_vector[2] = _body->GetAngularVelocity();
}

Eigen::Vector3f Box2DLink::getVelocityVector() const {
    Eigen::Vector3f vel;
    getVelocityVector(vel);
    return vel;
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

ObjectPtr Box2DLink::getObject() const {
    WorldPtr world = getWorld();
    return world->getObject(_object_name, false);
}

ObjectConstPtr Box2DLink::getConstObject() const {
    return getObject();
}

b2Body *Box2DLink::getBody() {
    return _body;
}

void Box2DLink::registerChildJoint(Box2DJointPtr joint) {
    _child_joints.push_back(Box2DJointWeakPtr(joint));
}

void Box2DLink::registerParentJoint(Box2DJointPtr joint) {
    _parent_joints.push_back(Box2DJointWeakPtr(joint));
}

void Box2DLink::getGeometry(std::vector<std::vector<Eigen::Vector2f> > &geometry) const {
    b2Fixture* fixture = _body->GetFixtureList();
    while (fixture) {
        b2Shape* shape = fixture->GetShape();
        std::vector<Eigen::Vector2f> polygon;
        if (shape->GetType() == b2Shape::Type::e_polygon) {
            b2PolygonShape* polygon_shape = static_cast<b2PolygonShape*>(shape);
            for (int32 v = 0; v < polygon_shape->GetVertexCount(); ++v) {
                b2Vec2 point = polygon_shape->GetVertex(v);
                Eigen::Vector2f eigen_point(point.x, point.y);
                polygon.push_back(eigen_point);
            }
            geometry.push_back(polygon);
        } else {
            throw std::runtime_error("[sim_env::Box2DLink::getGeometry] Could not retrieve geometry. Box2D shape is not a polygon.");
        }
        fixture = fixture->GetNext();
    }
}

void Box2DLink::setTransform(const Eigen::Affine3f &tf) {
    Box2DWorldPtr world = getBox2DWorld();
    auto rotation_matrix = tf.rotation();
    float theta = (float) acos(rotation_matrix(0, 0));
    theta = rotation_matrix(1,0) > 0.0 ? theta: -theta;
    float x = tf.translation()(0);
    float y = tf.translation()(1);
    setPose(Eigen::Vector3f(x, y, theta));
}

void Box2DLink::setPose(const Eigen::Vector3f& pose, bool update_children, bool joint_override) {
    Box2DWorldPtr world = getBox2DWorld();
    // first save joint values of child joints
    std::vector<float> child_joint_values;
    if (update_children) {
        for (auto& weak_child_joint : _child_joints) {
            JointPtr child_joint = weak_child_joint.lock();
            if (not joint_override) {
                child_joint_values.push_back(child_joint->getPosition());
            } else  {
                child_joint_values.push_back(0.0f);
            }
        }
    }
    // now set the new pose for this link
    b2Vec2 base_link_translation(world->getScale() * pose[0], world->getScale() * pose[1]);
    float theta = pose[2];
    _body->SetTransform(base_link_translation, theta);
    // next, update children if requested
    if (update_children) {
        int child_id = 0;
        for (auto& weak_child_joint : _child_joints) {
            Box2DJointPtr child_joint = weak_child_joint.lock();
            child_joint->resetPosition(child_joint_values.at(child_id), joint_override);
            ++child_id;
        }
    }
}

void Box2DLink::setVelocityVector(const Eigen::Vector3f& velocity, bool relative) {
    Box2DWorldPtr world = getBox2DWorld();
    b2Vec2 linear_vel_change(world->getScale() * velocity[0], world->getScale() * velocity[1]);
    float angular_vel_change = velocity[2];
    if (not relative) {
        linear_vel_change = linear_vel_change - _body->GetLinearVelocity();
        angular_vel_change = angular_vel_change - _body->GetAngularVelocity();
    }
    b2Vec2 current_vel = _body->GetLinearVelocity();
    _body->SetLinearVelocity(current_vel + linear_vel_change);
    _body->SetAngularVelocity(_body->GetAngularVelocity() + angular_vel_change);

    for (auto& weak_child_joint : _child_joints) {
        JointPtr child_joint = weak_child_joint.lock();
        Box2DLinkPtr child_link = std::static_pointer_cast<Box2DLink>(child_joint->getChildLink());
        // TODO test this
        child_link->propagateVelocityChange(linear_vel_change.x, linear_vel_change.y,
                                            angular_vel_change, _body);

    }
}
///////////////////////////////// PRIVATE //////////////////////////////////////
void Box2DLink::propagateVelocityChange(const float& dx,
                                        const float& dy,
                                        const float& dw,
                                        const b2Body* parent) {
    // the linear velocity applies to each body in a chain in the same way
    // the change of angular velocity, however, is a bit more tricky. we need to
    // compute the radial velocity
    // let's first compute radius vector from parent's origin to this link's frame
    // TODO I'm afraid this may still be wrong. Need to figure our where all velocities apply
    // TODO to. I have a feeling the center of mass may be the reference point for angular
    // TODO velocity and not the origin of the frame
    b2MassData mass_data;
   _body->GetMassData(&mass_data);
    b2Vec2 center_of_mass_in_world = mass_data.center;
    b2Vec2 origin_in_parent = parent->GetLocalPoint(center_of_mass_in_world);
    b2Vec2 radial_vel(origin_in_parent.y * dw, -origin_in_parent.x * dw);
    b2Vec2 current_lin_vel = _body->GetLinearVelocity();
    b2Vec2 lin_vel(dx, dy);
    _body->SetLinearVelocity(current_lin_vel + lin_vel + parent->GetWorldVector(radial_vel));
    // propagate change further down in the kinematic chain
    for (auto& child : _child_joints) {
        if (child.expired()) {
            throw std::logic_error("[Box2DLink::propagateVelocityChange] Child joint not available anymore.");
        }
        Box2DJointPtr child_joint = child.lock();
        Box2DLinkPtr child_link = child_joint->getChildBox2DLink();
        child_link->propagateVelocityChange(dx, dy, dw, parent);
    }
}
////////////////////////////////////////////////////////////////////////////////
/////////////////////* Definition of Box2DJoint members *///////////////////////
////////////////////////////////////////////////////////////////////////////////

Box2DJoint::Box2DJoint(const Box2DJointDescription &joint_desc, Box2DLinkPtr link_a, Box2DLinkPtr link_b,
                       Box2DWorldPtr world, const std::string& object_name):_destroyed(false) {
    Box2DWorldLock lock(world->world_mutex);
    _world = world;
    _name = joint_desc.name;
    _link_a = link_a;
    _link_b = link_b;
    _index = 0;
    _object_name = object_name;
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
        case JointType::Revolute: {
            b2RevoluteJointDef joint_def;
            b2Vec2 box2d_axis(world->getScale() * joint_desc.axis[0],
                              world->getScale() * joint_desc.axis[1]);
            joint_def.bodyA = link_a->getBody();
            joint_def.bodyB = link_b->getBody();
            joint_def.localAnchorA = box2d_axis; // axis in link_a frame
            joint_def.localAnchorB = b2Vec2(0, 0); // axis in link_b frame
            // base orientation of link_b frame relative to link_a frame
            joint_def.referenceAngle = joint_desc.axis_orientation;
            joint_def.lowerAngle = joint_desc.limits[0];
            joint_def.upperAngle = joint_desc.limits[1];
            joint_def.enableLimit = joint_desc.limits.norm() > 0.0;
            joint_def.maxMotorTorque = joint_desc.max_torque;
            joint_def.enableMotor = joint_desc.actuated;
            _joint = box2d_world->CreateJoint(&joint_def);
            _position_limits[0] = joint_desc.limits[0];
            _position_limits[1] = joint_desc.limits[1];
            // TODO what about unlimited joints?
            break;
        }
        case JointType::Prismatic: {
            b2PrismaticJointDef joint_def;
            b2Vec2 box2d_axis(joint_desc.axis[0], joint_desc.axis[1]);
            b2Vec2 box2d_direction(std::cos(joint_desc.axis_orientation), std::sin(joint_desc.axis_orientation));
            joint_def.bodyA = link_a->getBody();
            joint_def.bodyB = link_b->getBody();
            joint_def.referenceAngle = joint_desc.axis_orientation;
            joint_def.localAxisA = box2d_direction;
            joint_def.localAnchorA = box2d_axis;
            joint_def.localAnchorB = b2Vec2(0,0);
            joint_def.lowerTranslation = world->getScale() * joint_desc.limits[0];
            joint_def.upperTranslation = world->getScale() * joint_desc.limits[1];
            joint_def.enableLimit = joint_desc.limits.norm() > 0.0;
            joint_def.maxMotorForce = joint_desc.max_torque;
            joint_def.enableMotor = joint_desc.actuated;
            _joint = box2d_world->CreateJoint(&joint_def);
            _position_limits[0] = joint_desc.limits[0];
            _position_limits[1] = joint_desc.limits[1];
            break;
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
            return revolute_joint->GetJointAngle();
        }
        case JointType::Prismatic: {
            b2PrismaticJoint* prismatic_joint = static_cast<b2PrismaticJoint*>(_joint);
            return prismatic_joint->GetJointTranslation();
        }
    }
    return 0;
}

void Box2DJoint::setPosition(float v) {
    resetPosition(v, false);
}

void Box2DJoint::resetPosition(float value, bool child_joint_override) {
    auto world = getBox2DWorld();
    Box2DWorldLock lock(world->world_mutex);
    auto logger = world->getLogger();
    if (value < _position_limits[0] || value > _position_limits[1]) {
        std::stringstream ss;
        ss << "Position " << value << " is out of limits (" << _position_limits << " for joint " << getIndex();
        logger->logWarn(ss.str(), "[sim_env::Box2DJoint::resetPosition]");
        value = value < _position_limits[0] ? _position_limits[0] : _position_limits[1];
    }
    switch (_joint_type) {
        case JointType::Revolute: {
            b2RevoluteJoint* revolute_joint = static_cast<b2RevoluteJoint*>(_joint);
            // We need to manually compute how to change the child link's pose such
            // that the revolute joint reaches the desired angle
            // Since the child link's frame is centered at the joint axis, we only
            // need to update the link's orientation and ensure that is placed at the joint axis
            float new_angle = revolute_joint->GetBodyA()->GetAngle() + value
                              + revolute_joint->GetReferenceAngle();
            b2Vec2 axis_world = revolute_joint->GetAnchorA();
            Eigen::Vector3f new_pose_b;
            new_pose_b[0] = world->getInverseScale() * axis_world.x;
            new_pose_b[1] = world->getInverseScale() * axis_world.y;
            new_pose_b[2] = new_angle;
            Box2DLinkPtr link_b = getChildBox2DLink();
            link_b->setPose(new_pose_b, true, child_joint_override);
            break;
        }
        case JointType::Prismatic: {
            b2PrismaticJoint* prismatic_joint = static_cast<b2PrismaticJoint*>(_joint);
//            prismatic_joint->GetJointTranslation();
            //TODO same here, so probably best if this can be done at the end of this switch-case block
            break;
        }
    }
}

float Box2DJoint::getVelocity() const {
    switch (_joint_type) {
        case JointType::Revolute: {
            b2RevoluteJoint* revolute_joint = static_cast<b2RevoluteJoint*>(_joint);
            return revolute_joint->GetJointSpeed();
        }
        case JointType::Prismatic: {
            b2PrismaticJoint* prismatic_joint = static_cast<b2PrismaticJoint*>(_joint);
            return prismatic_joint->GetJointSpeed();
        }
        default:
            throw std::logic_error("[sim_env::Box2DJoint::getVelocity] This joint has an unsupported type");
    }
}

void Box2DJoint::setVelocity(float v) {
    auto world = getBox2DWorld();
    Box2DWorldLock lock(world->world_mutex);
    switch (_joint_type) {
        case JointType::Revolute: {
            Box2DLinkPtr link_a = getParentBox2DLink();
            Box2DLinkPtr link_b = getChildBox2DLink();
            Eigen::Vector3f velocity_a;
            link_a->getVelocityVector(velocity_a);
            Eigen::Vector3f velocity_b;
            link_b->getVelocityVector(velocity_b);
            velocity_b[2] = velocity_a[2] + v;
            link_b->setVelocityVector(velocity_b);
            break;
        }
        case JointType::Prismatic: {
            // TODO
            throw std::logic_error("setVelocity for pristmatic joints is not implemented yet");
        }
        default:
            throw std::logic_error("[sim_env::Box2DJoint::setVelocity] This joint has an unsupported type");
    }
}

Eigen::Array2f Box2DJoint::getPositionLimits() const {
    return _position_limits;
}

Eigen::Array2f Box2DJoint::getVelocityLimits() const {
    throw std::logic_error("[sim_env::Box2DJoint::getVelocityLimits not implemented yet");
}

unsigned int Box2DJoint::getIndex() const {
    return _index;
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

void Box2DJoint::setObjectName(const std::string& name) {
    _object_name = name;
}

void Box2DJoint::setIndex(unsigned int index) {
    _index = index;
}

EntityType Box2DJoint::getType() const {
    return EntityType::Joint;
}

Eigen::Affine3f Box2DJoint::getTransform() const {
    LinkPtr child_link = getChildLink();
    return child_link->getTransform();
}

LinkPtr Box2DJoint::getChildLink() const {
    return getChildBox2DLink();
}

Box2DLinkPtr Box2DJoint::getChildBox2DLink() const {
    if (_link_b.expired()) {
        throw std::logic_error("[sim_env::Box2DJoint::getChildBox2DLink] Child link does not exist anymore. This joint should not exist!");
    }
    return _link_b.lock();
}

LinkPtr Box2DJoint::getParentLink() const {
    return getParentBox2DLink();
}

Box2DLinkPtr Box2DJoint::getParentBox2DLink() const {
    if (_link_a.expired()) {
        throw std::logic_error("[sim_env::Box2DJoint::getParentBox2DLink] Parent link does not exist anymore. This joint should not exist!");
    }
    return _link_a.lock();
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

ObjectPtr Box2DJoint::getObject() const {
    WorldPtr world = getWorld();
    return world->getObject(_object_name, false);
}

ObjectConstPtr Box2DJoint::getConstObject() const {
    return getObject();
}

void Box2DJoint::setControlTorque(float value) {
    switch (_joint_type) {
        case JointType::Prismatic: {
            // TODO
            throw std::logic_error("setControlTorque for pristmatic joints is not implemented yet");
            break;
        }
        case JointType::Revolute: {
            b2RevoluteJoint* revolute_joint = static_cast<b2RevoluteJoint*>(_joint);
            revolute_joint->EnableMotor(true);
            // TODO this is a hack. does it work?
            revolute_joint->SetMaxMotorTorque(value);
            revolute_joint->SetMotorSpeed(std::numeric_limits<float>::max());
            break;
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
/////////////////////* Definition of Box2DObject members *//////////////////////
////////////////////////////////////////////////////////////////////////////////

Box2DObject::Box2DObject(const Box2DObjectDescription &obj_desc, Box2DWorldPtr world):_destroyed(false) {
    _name = obj_desc.name;
    _is_static = obj_desc.is_static;
    _world = Box2DWorldWeakPtr(world);
    // first create links
    for (auto &link_desc : obj_desc.links) {
        Box2DLinkPtr link(new Box2DLink(link_desc, world, _is_static, _name));
        _links[link->getName()] = link;
    }
    _base_link = _links[obj_desc.base_link];

    // next create joints
    for (auto &joint_desc : obj_desc.joints) {
        Box2DLinkPtr link_a = _links[joint_desc.link_a];
        Box2DLinkPtr link_b = _links[joint_desc.link_b];
        Box2DJointPtr joint(new Box2DJoint(joint_desc, link_a, link_b, world, _name));
        link_a->registerChildJoint(joint);
        link_b->registerParentJoint(joint);
        _joints[joint->getName()] = joint;
        _sorted_joints.push_back(joint);
        joint->setIndex((unsigned int)(_sorted_joints.size()) - 1);
    }
    unsigned int base_dofs = _is_static ? 0 : 3;
    _num_dofs = (unsigned int) (base_dofs + _sorted_joints.size());
    // by default set all dofs active
    _active_dof_indices = getDOFIndices();
    // finally initialize all links to be placed correctly (the initial pose at the origin does not matter)
    _base_link->setPose(Eigen::Vector3f(0.0f, 0.0f, 0.0f), true, true);
}

Box2DObject::~Box2DObject() {
    if (!_destroyed) {
        throw std::logic_error("[Box2DObject::~Box2DObject] Object destructor called before destroy()");
    }
}

void Box2DObject::destroy(const std::shared_ptr<b2World>& b2world) {
    // Unfortunately, this can not be called in the destructor
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
    return _base_link->getTransform();
}

Eigen::Vector3f Box2DObject::getPose() const {
    return _base_link->getPose();
}

void Box2DObject::getPose(Eigen::Vector3f& pose) const {
    return _base_link->getPose(pose);
}

void Box2DObject::setTransform(const Eigen::Affine3f &tf) {
    auto world = getBox2DWorld();
    Box2DWorldLock lock(world->world_mutex);
    _base_link->setTransform(tf);
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
    // TODO
    return false;
}

bool Box2DObject::checkCollision(const std::vector<CollidableConstPtr> &object_list) const {
    // TODO
    return false;
}

void Box2DObject::setActiveDOFs(const Eigen::VectorXi &indices) {
    auto world = getBox2DWorld();
    Box2DWorldLock lock(world->world_mutex);
    _active_dof_indices = indices;
}

Eigen::VectorXi Box2DObject::getActiveDOFs() const {
    return _active_dof_indices;
}

Eigen::VectorXi Box2DObject::getDOFIndices() const {
    Eigen::VectorXi dofs(_num_dofs);
    for (unsigned int i = 0; i < _num_dofs; ++i) {
        dofs[i] = i;
    }
    return dofs;
}

Eigen::VectorXf Box2DObject::getDOFPositions(const Eigen::VectorXi &indices) const {
    Eigen::VectorXi dofs_to_retrieve = indices;
    if (dofs_to_retrieve.size() == 0) {
        dofs_to_retrieve = _active_dof_indices;
    }
    Eigen::VectorXf output(dofs_to_retrieve.size());
    for (unsigned int i = 0; i < dofs_to_retrieve.size(); ++i) {
        int dof = dofs_to_retrieve[i];
        if (dof < 3 && not _is_static) {
            Eigen::Vector3f pose;
            _base_link->getPose(pose);
            output[i] = pose[dof];
        } else {
            int joint_index = _is_static ? dof : dof - 3;
            output[i] = _sorted_joints[joint_index]->getPosition();
        }
    }
    return output;
}

Eigen::ArrayX2f Box2DObject::getDOFPositionLimits(const Eigen::VectorXi& indices) const {
    Eigen::VectorXi dofs_to_retrieve = indices;
    if (dofs_to_retrieve.size() == 0) {
        dofs_to_retrieve = _active_dof_indices;
    }
    Eigen::ArrayX2f limits(dofs_to_retrieve.size(), 2);
    for (int i = 0; i < dofs_to_retrieve.size(); ++i) {
        int dof = dofs_to_retrieve[i];
        if (dof < 3 and not _is_static) {
            // TODO we could get bounds from the world here
            limits(i, 0) = std::numeric_limits<float>::min();
            limits(i, 1) = std::numeric_limits<float>::max();
        } else {
            int joint_index = _is_static ? dof : dof - 3;
            Eigen::Array2f joint_limit = _sorted_joints.at(joint_index)->getPositionLimits();
            limits(i, 0) = joint_limit[0];
            limits(i, 1) = joint_limit[1];
        }
    }
    return limits;
}

void Box2DObject::setDOFPositions(const Eigen::VectorXf &values, const Eigen::VectorXi &indices) {
    auto world = getBox2DWorld();
    Box2DWorldLock lock(world->world_mutex);
    Eigen::VectorXi dofs_to_set = indices;
    if (dofs_to_set.size() == 0) {
        dofs_to_set = _active_dof_indices;
    }
    // TODO do we need to be sure these are sorted?
    if (dofs_to_set.size() != values.size()) {
        throw std::runtime_error("[sim_env::Box2DObject::setDOFPositions] Could not set DOF positions."
                                         " Number of indices to set and number of provided values"
                                         " do not match.");
    }
    for (unsigned int i = 0; i < dofs_to_set.size(); ++i) {
        int dof = dofs_to_set[i];
        if (dof < 3 && not _is_static) {
            Eigen::Vector3f current_pose;
            _base_link->getPose(current_pose);
            current_pose[dof] = values[i];
            _base_link->setPose(current_pose);
        } else {
            int joint_index = _is_static ? dof : dof - 3;
            _sorted_joints[joint_index]->setPosition(values[i]);
        }
    }
}

Eigen::VectorXf Box2DObject::getDOFVelocities(const Eigen::VectorXi &indices) const {
    Eigen::VectorXi dofs_to_retrieve = indices;
    if (dofs_to_retrieve.size() == 0) {
        dofs_to_retrieve = _active_dof_indices;
    }
    Eigen::VectorXf output(dofs_to_retrieve.size());
    for (unsigned int i = 0; i < dofs_to_retrieve.size(); ++i) {
        int dof = dofs_to_retrieve[i];
        if (not _is_static and dof < 3) {
            Eigen::Vector3f current_velocity;
            _base_link->getVelocityVector(current_velocity);
            output[i] = current_velocity[dof];
        } else {
            int joint_index = _is_static ? dof : dof - 3;
            output[i] = _sorted_joints[joint_index]->getVelocity();
        }
    }
    return output;
}

Eigen::ArrayX2f Box2DObject::getDOFVelocityLimits(const Eigen::VectorXi& indices) const {
    throw std::logic_error("Box2DObject::getDOFVelocityLimits is not implemented yet ");
}

void Box2DObject::setDOFVelocities(const Eigen::VectorXf &values, const Eigen::VectorXi &indices) {
    auto world = getBox2DWorld();
    Box2DWorldLock lock(world->world_mutex);
    Eigen::VectorXi dofs_to_set = indices;
    if (dofs_to_set.size() == 0) {
        dofs_to_set = _active_dof_indices;
    }
    if (dofs_to_set.size() != values.size()) {
        throw std::runtime_error("[sim_env::Box2DObject::setDOFVelocities] Could not set DOF velocities."
                                         " Number of indices to set and number of provided values"
                                         " do not match.");
    }
    for (unsigned int i = 0; i < dofs_to_set.size(); ++i) {
        int dof = dofs_to_set[i];
        if (dof < 3 && not _is_static) {
            Eigen::Vector3f current_velocity;
            _base_link->getVelocityVector(current_velocity);
            current_velocity[dof] = values[i];
            _base_link->setVelocityVector(current_velocity);
        } else {
            int joint_index = _is_static ? dof : dof - 3;
            _sorted_joints[joint_index]->setVelocity(values[i]);
        }
    }
}

void Box2DObject::setName(const std::string &name) {
    _name = name;
    // let all links and joints know that we have a new name
    for (auto& link : _links) {
        link.second->setObjectName(name);
    }
    for (auto& joint : _joints) {
        joint.second->setObjectName(name);
    }
}

bool Box2DObject::isStatic() const {
    return _is_static;
}

void Box2DObject::getLinks(std::vector<LinkPtr>& links) {
    for (auto &iter : _links) {
        links.push_back(iter.second);
    }
}

void Box2DObject::getLinks(std::vector<LinkConstPtr>& links) const {
    for (auto &iter : _links) {
        links.push_back(iter.second);
    }
}

LinkPtr Box2DObject::getLink(const std::string &link_name) {
    if (_links.find(link_name) != _links.end()) {
        return _links.at(link_name);
    }
    return LinkPtr(nullptr);
}

LinkConstPtr Box2DObject::getConstLink(const std::string &link_name) const {
    if (_links.find(link_name) != _links.end()) {
        return _links.at(link_name);
    }
    return LinkConstPtr(nullptr);
}

void Box2DObject::getJoints(std::vector<JointPtr>& joints) {
    for (auto &iter : _joints) {
        joints.push_back(iter.second);
    }
}

void Box2DObject::getJoints(std::vector<JointConstPtr>& joints) const {
    for (auto &iter : _joints) {
        joints.push_back(iter.second);
    }
}

JointPtr Box2DObject::getJoint(const std::string &joint_name) {
    if (_joints.find(joint_name) != _joints.end()) {
        return _joints.at(joint_name);
    }
    return JointPtr(nullptr);
}

JointConstPtr Box2DObject::getConstJoint(const std::string &joint_name) const {
    if (_joints.find(joint_name) != _joints.end()) {
        return _joints.at(joint_name);
    }
    return JointConstPtr(nullptr);
}

LinkPtr Box2DObject::getBaseLink() {
    return _base_link;
}

Box2DLinkPtr Box2DObject::getBox2DBaseLink() {
   return _base_link;
}

unsigned int Box2DObject::getNumDOFs() const {
    return _num_dofs;
}

Box2DJointPtr Box2DObject::getJoint(unsigned int idx) {
    return _sorted_joints.at(idx);
}
////////////////////////////////////////////////////////////////////////////////
/////////////////////* Definition of Box2DRobot members *///////////////////////
////////////////////////////////////////////////////////////////////////////////
Box2DRobot::Box2DRobot(const Box2DObjectDescription &robot_desc, Box2DWorldPtr world):_destroyed(false) {
    // A robot is essentially an actuated object. Rather than implementing
    // the same functionalities twice (for object and robot), we use composition here.
    // Also we do not wanna have diamond inheritance
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

Eigen::VectorXi Box2DRobot::getActiveDOFs() const {
    return _robot_object->getActiveDOFs();
}

Eigen::VectorXi Box2DRobot::getDOFIndices() const {
    return _robot_object->getDOFIndices();
}

Eigen::VectorXf Box2DRobot::getDOFPositions(const Eigen::VectorXi &indices) const {
    return _robot_object->getDOFPositions(indices);
}

Eigen::ArrayX2f Box2DRobot::getDOFPositionLimits(const Eigen::VectorXi& indices) const {
    return _robot_object->getDOFPositionLimits(indices);
}

void Box2DRobot::setDOFPositions(const Eigen::VectorXf &values, const Eigen::VectorXi &indices) {
    _robot_object->setDOFPositions(values, indices);
}

Eigen::VectorXf Box2DRobot::getDOFVelocities(const Eigen::VectorXi &indices) const {
    return _robot_object->getDOFVelocities(indices);
}

Eigen::ArrayX2f Box2DRobot::getDOFVelocityLimits(const Eigen::VectorXi& indices) const {
    return _robot_object->getDOFVelocityLimits(indices);
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

void Box2DRobot::getLinks(std::vector<LinkPtr>& links) {
    _robot_object->getLinks(links);
}

void Box2DRobot::getLinks(std::vector<LinkConstPtr>& links) const {
    _robot_object->getLinks(links);
}

void Box2DRobot::getJoints(std::vector<JointPtr>& joints) {
    _robot_object->getJoints(joints);
}

void Box2DRobot::getJoints(std::vector<JointConstPtr>& joints) const {
    _robot_object->getJoints(joints);
}

LinkPtr Box2DRobot::getLink(const std::string &link_name) {
    return _robot_object->getLink(link_name);
}

LinkConstPtr Box2DRobot::getConstLink(const std::string &link_name) const {
    return _robot_object->getConstLink(link_name);
}

JointPtr Box2DRobot::getJoint(const std::string &joint_name) {
    return _robot_object->getJoint(joint_name);
}

JointConstPtr Box2DRobot::getConstJoint(const std::string &joint_name) const {
    return _robot_object->getConstJoint(joint_name);
}

LinkPtr Box2DRobot::getBaseLink() {
    return _robot_object->getBaseLink();
}

unsigned int Box2DRobot::getNumDOFs() const {
    return _robot_object->getNumDOFs();
}

void Box2DRobot::control(float timestep) {
    // Only do sth if there is actually a controller registered
    if (not _controller_callback) {
        return;
    }
    // Get positions of currently active DoFs
    Eigen::VectorXf positions = getDOFPositions();
    // Get velocities of currently active DoFs
    Eigen::VectorXf velocitites = getDOFVelocities();
    assert(velocitites.size() == positions.size());
    // Create efforts array
    Eigen::VectorXf efforts(velocitites.size());
    bool success = _controller_callback(positions, velocitites, timestep, shared_from_this(), efforts);
    if (success) { // we have a control to apply
        assert(efforts.size() == positions.size());
        commandEfforts(efforts);
    } else {// else we don't do anything apart from logging
        LoggerPtr logger = getWorld()->getLogger();
        logger->logWarn("Controller indicated failure. Not applying any efforts.", "[sim_env::Box2DRobot::control]");
    }
}

void Box2DRobot::setController(ControlCallback controll_fn) {
    _controller_callback = controll_fn;
}

void Box2DRobot::commandEfforts(const Eigen::VectorXf &target) {
    Eigen::VectorXi active_dofs = _robot_object->getActiveDOFs();
    for (int i = 0; i < active_dofs.size(); ++i) {
        int dof_idx = active_dofs[i];
        // check whether we have to move the base link
        if (not isStatic() and dof_idx < 3) {
            Box2DLinkPtr base_link = _robot_object->getBox2DBaseLink();
            b2Body* body = base_link->getBody();
            switch(dof_idx) {
                case 0: {
                    // force in x
                    body->ApplyForceToCenter(b2Vec2(target[i], 0.0f), true);
                    break;
                }
                case 1: {
                    // force in y
                    body->ApplyForceToCenter(b2Vec2(target[i], 0.0f), true);
                    break;
                }
                case 2: {
                    // torque
                    body->ApplyTorque(target[i], true);
                    break;
                }
                default: {
                    // impossible case
                    throw std::logic_error("[sim_env::Box2DRobot::commandEfforts] Encountered an impossible state.");
                    break;
                }
            }
        } else {
            // torque/effort for joint
            int dof_offset = isStatic() ? 0 : 3;
            int joint_idx = dof_idx - dof_offset;
            assert(joint_idx > 0);
            Box2DJointPtr joint = _robot_object->getJoint((unsigned int) joint_idx);
            joint->setControlTorque(target[i]);
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
/////////////////////* Definition of Box2DWorld members *///////////////////////
////////////////////////////////////////////////////////////////////////////////

Box2DWorld::Box2DWorld() : _world(nullptr), _b2_ground_body(nullptr), _time_step(0.01f),
    _velocity_steps(10), _position_steps(10) {
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
    return getBox2DRobot(name);
}

Box2DRobotPtr Box2DWorld::getBox2DRobot(const std::string &name) const {
    if (_robots.count(name)) {
        return _robots.at(name);
    }
    return nullptr;
}

ObjectPtr Box2DWorld::getObject(const std::string &name, bool exclude_robots) const {
    ObjectPtr object = getBox2DObject(name);
    if (not object and not exclude_robots) {
        if (_robots.count(name)) {
            object = _robots.at(name);
        }
    }
    return object;
}

Box2DObjectPtr Box2DWorld::getBox2DObject(const std::string& name) const {
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

void Box2DWorld::stepPhysics(int steps) {
    Box2DWorldLock lock(world_mutex);
    for (int i = 0; i < steps; ++i) {
        // call control functions of all robots in the scene
        for (auto& robot_map_iter : _robots) {
            robot_map_iter.second->control(_time_step);
        }
        // simulate physics
        _world->Step(_time_step, _velocity_steps, _position_steps);
    }
}

bool Box2DWorld::supportsPhysics() const {
    return true;
}

void Box2DWorld::setPhysicsTimeStep(float physics_step) {
    _time_step = physics_step;
}

void Box2DWorld::setVelocitySteps(int velocity_steps) {
    _velocity_steps = velocity_steps;
}

void Box2DWorld::setPositionSteps(int position_steps) {
    _position_steps = position_steps;
}

float Box2DWorld::getPhysicsTimeStep() const {
    return _time_step;
}

int Box2DWorld::getVelocitySteps() const {
    return _velocity_steps;
}

int Box2DWorld::getPositionSteps() const {
    return _position_steps;
}

WorldViewerPtr Box2DWorld::getViewer() {
    // TODO
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
        std::string prefix("[sim_env::Box2DWorld::getInverseScale]");
        std::string msg("Invalid scale: 0.0. Can not invert scale.""Invalid scale: 0.0. Can not invert scale.");
        _logger->logErr(msg, prefix);
        throw std::runtime_error(prefix + msg);
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
    Box2DWorldLock lock(world_mutex);
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
    Box2DWorldLock lock(world_mutex);
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
