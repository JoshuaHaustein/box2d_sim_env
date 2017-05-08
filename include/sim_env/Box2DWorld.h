//
// Created by joshua on 4/20/17.
//
#ifndef BOX2D_SIM_ENV_BOX2DWORLD_H
#define BOX2D_SIM_ENV_BOX2DWORLD_H

#include "sim_env/SimEnv.h"
#include "Box2DIOUtils.h"
#include <boost/filesystem/path.hpp>
#include <yaml-cpp/yaml.h>
#include <exception>

// Forward declarations of Box2D stuff
class b2World;
class b2Body;
class b2Joint;
class b2BodyDef;

namespace sim_env{

    class Box2DObject;
    typedef std::shared_ptr<Box2DObject> Box2DObjectPtr;
    typedef std::shared_ptr<const Box2DObject> Box2DObjectConstPtr;

    class Box2DWorld;
    typedef std::shared_ptr<Box2DWorld> Box2DWorldPtr;
    typedef std::shared_ptr<const Box2DWorld> Box2DWorldConstPtr;

    class Box2DRobot;
    typedef std::shared_ptr<Box2DRobot> Box2DRobotPtr;
    typedef std::shared_ptr<const Box2DRobot> Box2DRobotConstPtr;

    class Box2DLink;
    typedef std::shared_ptr<Box2DLink> Box2DLinkPtr;
    typedef std::shared_ptr<const Box2DLink> Box2DLinkConstPtr;

    class Box2DJoint;
    typedef std::shared_ptr<Box2DJoint> Box2DJointPtr;
    typedef std::shared_ptr<const Box2DJoint> Box2DJointConstPtr;

    class Box2DLink : public Link {
        friend class Box2DJoint;
    public:
        Box2DLink(const Box2DLinkDescription& link_desc, Box2DWorldPtr world, bool is_static);
        Box2DLink(const Box2DLink& link) = delete;
        Box2DLink& operator=(const Box2DLink&) = delete;
        Box2DLink& operator=(Box2DLink&&) = delete;
        ~Box2DLink();

        bool checkCollision(CollidableConstPtr other) const override;

        bool checkCollision(const std::vector<CollidableConstPtr> &others) const override;

        std::string getName() const override;

        void setName(const std::string &name) override;

        EntityType getType() const override;

        Eigen::Transform getTransform() const override;

        WorldPtr getWorld() const override;

        WorldConstPtr getConstWorld() const override;

    protected:
        b2Body* getBody();
    private:
        Box2DWorldPtr _world;
        b2Body* _body;
        b2Joint* _friction_joint;
        std::string _name;
    };

    class Box2DJoint : public Joint {
    public:
        Box2DJoint(const Box2DJointDescription& joint_desc,
                   Box2DLinkPtr link_a,
                   Box2DLinkPtr link_b,
                   Box2DWorldPtr world);
        Box2DJoint(const Box2DJoint& joint) = delete;
        Box2DJoint& operator=(const Box2DJoint& joint) = delete;
        Box2DJoint& operator=(Box2DJoint&&) = delete;
        ~Box2DJoint();
        float getPosition() const override;
        void setPosition(float v) override;
        float getVelocity() const override;
        void setVelocity(float v) override;
        unsigned int getIndex() const override;
        JointType getJointType() const override;
        std::string getName() const override;
        void setName(const std::string &name) override;
        EntityType getType() const override;
        Eigen::Transform getTransform() const override;
        WorldPtr getWorld() const override;
        WorldConstPtr getConstWorld() const override;

    private:
        Box2DWorldPtr _world;
        std::string _name;
        Box2DLinkPtr _link_a;
        Box2DLinkPtr _link_b;
        b2Joint* _joint;
        JointType _joint_type;

    };

    class Box2DObject : public Object {
    public:
        Box2DObject(const Box2DObjectDescription& obj_desc, Box2DWorldPtr world);
        ~Box2DObject();

        virtual std::string getName() const override;

        virtual void setName(const std::string &name) override;

        virtual EntityType getType() const override;

        virtual Eigen::Transform getTransform() const override;

        virtual void setTransform(const Eigen::Transform& tf) override;

        virtual WorldPtr getWorld() const override;

        virtual WorldConstPtr getConstWorld() const override;

        virtual bool checkCollision(CollidableConstPtr other_object) const override;

        virtual bool checkCollision(const std::vector<CollidableConstPtr>& object_list) const override;

        virtual void setActiveDOFs(const Eigen::VectorXi& indices) override;

        virtual Eigen::VectorXi getActiveDOFs() override;

        virtual Eigen::VectorXi getDOFIndices() override;

        virtual Eigen::VectorXd getDOFPositions(const Eigen::VectorXi& indices=Eigen::VectorXi()) const override;

        virtual void setDOFPositions(const Eigen::VectorXd& values, const Eigen::VectorXi& indices=Eigen::VectorXi()) override;

        virtual Eigen::VectorXd getDOFVelocities(const Eigen::VectorXi& indices=Eigen::VectorXi()) const override;

        virtual void setDOFVelocities(const Eigen::VectorXd& values, const Eigen::VectorXi& indices=Eigen::VectorXi()) override;

        virtual bool isStatic() const override;

    protected:
    private:
        std::string _name;
        Eigen::Affine3d _transform;
        Box2DWorldPtr _world;
        Eigen::VectorXi _active_dof_indices;
        std::map<std::string, Box2DLinkPtr> _links;
        std::map<std::string, Box2DJointPtr> _joints;
        bool _is_static;

    };

    class Box2DRobot : public Robot {
    public:

    };

    class Box2DWorld : public World {
        friend class Box2DLink; // these classes need access to the underlying Box2D world
        friend class Box2DJoint;
    public:
        static const float GROUND_DEFAULT_MIN_X = -100.0f;
        static const float GROUND_DEFAULT_MIN_Y = -100.0f;
        static const float GROUND_DEFAULT_MAX_X = 100.0f;
        static const float GROUND_DEFAULT_MAX_Y = 100.0f;
        static const float GRAVITY = 9.81f;
        Box2DWorld();
        Box2DWorld(const Box2DWorld& joint) = delete;
        Box2DWorld& operator=(const Box2DWorld& joint) = delete;
        Box2DWorld& operator=(Box2DWorld&&) = delete;
        ~Box2DWorld(); // we can ignore the warning that we are hiding enable_shared_from_this' destructor

        void loadWorld(const std::string &path) override;

        RobotPtr getRobot(const std::string &name) const override;

        ObjectPtr getObject(const std::string &name) const override;

        void getObjects(std::vector<ObjectPtr> &objects, bool exclude_robots) const override;

        void getRobots(std::vector<RobotPtr> &robots) const override;

        void stepPhysics(int steps) const override;

        bool supportsPhysics() const override;

        void setPhysicsTimeStep(double physics_step) override;

        void getPhysicsTimeStep() const override;

        WorldViewerPtr getViewer() override;

        LoggerPtr getLogger() override;

        float getScale() const;
        float getInverseScale() const;

        float getGravity() const;
    protected:
        std::shared_ptr<b2World> getRawBox2DWorld();
        b2Body* getGroundBody();
    private:
        b2Body* _b2_ground_body;
        std::shared_ptr<b2World> _world;
        LoggerPtr _logger;
        float _scale;
        std::map<std::string, Box2DObjectPtr> _objects;
        std::map<std::string, Box2DRobotPtr> _robots;

        void eraseWorld();
        void createWorld(const Box2DEnvironmentDescription& env_desc);
        void createNewRobot(const Box2DObjectDescription& robot_desc);
        void createNewObject(const Box2DObjectDescription& object_desc);
        void createGround(const Eigen::Vector4f& world_bounds);

    };
}

#endif //BOX2D_SIM_ENV_BOX2DWORLD_H
