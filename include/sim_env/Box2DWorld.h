//
// Created by joshua on 4/20/17.
//
#ifndef BOX2D_SIM_ENV_BOX2DWORLD_H
#define BOX2D_SIM_ENV_BOX2DWORLD_H

#include <mutex>
#include <stack>
#include "sim_env/SimEnv.h"
#include "sim_env/Controller.h"
#include "Box2DIOUtils.h"
#include <boost/filesystem/path.hpp>
#include <yaml-cpp/yaml.h>
#include "Box2D/Box2D.h"

namespace sim_env{
    class Box2DObject;
    typedef std::shared_ptr<Box2DObject> Box2DObjectPtr;
    typedef std::weak_ptr<Box2DObject> Box2DObjectWeakPtr;
    typedef std::shared_ptr<const Box2DObject> Box2DObjectConstPtr;
    typedef std::weak_ptr<const Box2DObject> Box2DObjectConstWeakPtr;

    class Box2DWorld;
    typedef std::shared_ptr<Box2DWorld> Box2DWorldPtr;
    typedef std::weak_ptr<Box2DWorld> Box2DWorldWeakPtr;
    typedef std::shared_ptr<const Box2DWorld> Box2DWorldConstPtr;
    typedef std::weak_ptr<const Box2DWorld> Box2DWorldConstWeakPtr;

    class Box2DRobot;
    typedef std::shared_ptr<Box2DRobot> Box2DRobotPtr;
    typedef std::weak_ptr<Box2DRobot> Box2DRobotWeakPtr;
    typedef std::shared_ptr<const Box2DRobot> Box2DRobotConstPtr;
    typedef std::weak_ptr<const Box2DRobot> Box2DRobotConstWeakPtr;

    class Box2DLink;
    typedef std::shared_ptr<Box2DLink> Box2DLinkPtr;
    typedef std::weak_ptr<Box2DLink> Box2DLinkWeakPtr;
    typedef std::shared_ptr<const Box2DLink> Box2DLinkConstPtr;
    typedef std::weak_ptr<const Box2DLink> Box2DLinkConstWeakPtr;

    class Box2DJoint;
    typedef std::shared_ptr<Box2DJoint> Box2DJointPtr;
    typedef std::weak_ptr<Box2DJoint> Box2DJointWeakPtr;
    typedef std::shared_ptr<const Box2DJoint> Box2DJointConstPtr;
    typedef std::weak_ptr<const Box2DJoint> Box2DJointConstWeakPtr;

    class Box2DCollidable;
    typedef std::shared_ptr<Box2DCollidable> Box2DCollidablePtr;
    typedef std::shared_ptr<const Box2DCollidable> Box2DCollidableConstPtr;

    class Box2DWorldViewer;
    typedef std::shared_ptr<Box2DWorldViewer> Box2DWorldViewerPtr;
    typedef std::shared_ptr<const Box2DWorldViewer> Box2DWorldViewerConstPtr;

    class Box2DCollisionChecker;
    typedef std::shared_ptr<Box2DCollisionChecker> Box2DCollisionCheckerPtr;
    typedef std::shared_ptr<const Box2DCollisionChecker> Box2DCollisionCheckerConstPtr;

    namespace viewer {
        class Box2DDrawingInterface;
        typedef std::shared_ptr<Box2DDrawingInterface> Box2DDrawingInterfacePtr;
        typedef std::shared_ptr<const Box2DDrawingInterface> Box2DDrawingInterfaceConstPtr;
    }

    class Box2DCollidable {
        friend class Box2DCollisionChecker;
    protected:
        // TODO do we need this intermediate interface?
        // TODO here we could define the interface that we need to check whether two Box2DEntities collide
        /**
         * Returns all b2Bodies that this collidable contains.
         * @param bodies list of all b2Bodies
         */
        virtual void getBodies(std::vector<b2Body*>& bodies) = 0;

        /**
         * Returns the link associated with the given body.
         * If this collidable is a link, it returns itself.
         * If this collidable is an object, it returns the link with this body.
         * If this collidable is not associated with the given body, null_ptr is returned.
         * @param body - body to get the link for
         * @return - shared pointer to the link, or null_ptr
         */
        virtual LinkPtr getLink(b2Body* body) = 0;
    };

    /**
     * Box2DLink - The box2d implementation of a robot/object link.
     */
    class Box2DLink : public Link, public Box2DCollidable, public std::enable_shared_from_this<Box2DLink> {
        friend class Box2DWorld;
        friend class Box2DJoint;
        friend class Box2DObject;
        friend class Box2DRobot;
    public:
        /**
         * A link can only be instantiated by an Box2DObject.
         */
        Box2DLink() = delete;
        Box2DLink(const Box2DLink& link) = delete;
        Box2DLink& operator=(const Box2DLink&) = delete;
        Box2DLink& operator=(Box2DLink&&) = delete;
        ~Box2DLink();

        bool checkCollision() override;
        bool checkCollision(std::vector<Contact> &contacts) override;
        bool checkCollision(const std::vector<LinkPtr> &other_links) override;
        bool checkCollision(const std::vector<LinkPtr> &other_links, std::vector<Contact> &contacts) override;
        bool checkCollision(const std::vector<ObjectPtr> &other_objects) override;
        bool checkCollision(const std::vector<ObjectPtr> &other_objects, std::vector<Contact> &contacts) override;

        std::string getName() const override;
        EntityType getType() const override;

        Eigen::Affine3f getTransform() const override;
        void getPose(Eigen::Vector3f& pose) const;
        Eigen::Vector3f getPose() const;

        void getVelocityVector(Eigen::Vector3f& vel_vector) const;
        Eigen::Vector3f getVelocityVector() const;

        WorldPtr getWorld() const override;
        ObjectPtr getObject() const override;

        WorldConstPtr getConstWorld() const override;
        ObjectConstPtr getConstObject() const override;

        virtual void getChildJoints(std::vector<JointPtr>& child_joints);
        virtual void getConstChildJoints(std::vector<JointConstPtr>& child_joints) const;
        virtual void getParentJoints(std::vector<JointPtr>& parent_joints);
        virtual void getConstParentJoints(std::vector<JointConstPtr>& parent_joints) const;

        void getGeometry(std::vector< std::vector<Eigen::Vector2f> >& geometry) const;
        // Box2D specific
        Eigen::Vector2f getCenterOfMass() const;
        void getCenterOfMass(Eigen::Vector2f& com) const;
        float getMass() const;
        float getInertia() const;
        Box2DWorldPtr getBox2DWorld() const;

//        Box2DObjectPtr getBox2DObject() const;

    protected:
        // the constructor is protected to ensure that links can only be created within objects.
        Box2DLink(const Box2DLinkDescription& link_desc, Box2DWorldPtr world, bool is_static,
                  const std::string& object_name);
        // for cleanup, we need destroy functions. these are called when the Box2DWorld is destroyed.
        void destroy(const std::shared_ptr<b2World>& b2world); // TODO do we need to make this thread-safe?
        // set the name of this link
        void setName(const std::string &name) override;
        void setObjectName(const std::string& name);
        // returns the underlying box2d body
        b2Body* getBody();
        // registers a child joint
        void registerChildJoint(Box2DJointPtr joint);
        // registers a parent joint
        void registerParentJoint(Box2DJointPtr joint);
        // sets the transform of this link. Should not be called externally. Calls setPose(pose, true, false)
        void setTransform(const Eigen::Affine3f& tf);
        /*
         * Sets the pose of this link. If update_children is true, all child links are moved together with this link.
         * if joint_override is true, the relative transforms between this link and its children is set
         * to the initial transform defined by the joint connecting both links (i.e. it is set to joint position 0)
         */
        void setPose(const Eigen::Vector3f& pose, bool update_children=true, bool joint_override=false);
        // sets the velocity (translational x,y and rotational) of this link. Should not be called externally.
        void setVelocityVector(const Eigen::Vector3f& velocity, bool relative=false);
        // resets the origin of this link to its center of mass
        void setOriginToCenterOfMass();
        // Adds the body of this link to the given vector - used for collision checks
        void getBodies(std::vector<b2Body *> &bodies) override;
        // returns a pointer to this link
        LinkPtr getLink(b2Body* body) override;
    private:
        Box2DWorldWeakPtr _world;
        b2Body* _body;
        b2Vec2 _local_origin_offset; // optional offset from the body's frame of reference to link frame of reference
        b2Joint* _friction_joint;
        std::string _name;
        std::string _object_name;
        std::vector<Box2DJointWeakPtr> _child_joints;
        std::vector<Box2DJointWeakPtr> _parent_joints;
        bool _destroyed;

        /*
         * Recursively propagates a velocity change through a kinematic chain.
         * @param lin_vel linear velocity change in global frame (scaled to box2d world)
         * @param dw angular velocity change
         * @param parent pointer to the parent body (used for coordinate transformations)
         */
        void propagateVelocityChange(const float& dx, const float& dy, const float& dw, const b2Body* parent);
    };

    /**
     * Box2DJoint - This class implements robot/object joints in box2d.
     */
    class Box2DJoint : public Joint {
        friend class Box2DWorld;
        friend class Box2DObject;
        friend class Box2DLink;
        friend class Box2DRobot;
    public:
        /**
         * A Box2DJoint can only be instantiated by an object/robot.
         */
        Box2DJoint() = delete;
        Box2DJoint(const Box2DJoint& joint) = delete;
        Box2DJoint& operator=(const Box2DJoint& joint) = delete;
        Box2DJoint& operator=(Box2DJoint&&) = delete;
        ~Box2DJoint();
        float getPosition() const override;
        void setPosition(float v) override;
        float getVelocity() const override;
        void setVelocity(float v) override;
        Eigen::Array2f getPositionLimits() const override;
        void getPositionLimits(Eigen::Array2f& limits) const override;
        Eigen::Array2f getVelocityLimits() const override;
        void getVelocityLimits(Eigen::Array2f& limits) const override;
        Eigen::Array2f getAccelerationLimits() const;
        void getAccelerationLimits(Eigen::Array2f& limits) const;
        unsigned int getJointIndex() const override;
        unsigned int getDOFIndex() const override;
        JointType getJointType() const override;
        std::string getName() const override;
        EntityType getType() const override;
        Eigen::Affine3f getTransform() const override;
        WorldPtr getWorld() const override;
        ObjectPtr getObject() const override;
        virtual LinkPtr getChildLink() const override;
        virtual LinkPtr getParentLink() const override;
        WorldConstPtr getConstWorld() const override;
        ObjectConstPtr getConstObject() const override;
        void getDOFInformation(DOFInformation& info) const override;
        DOFInformation getDOFInformation() const override;
//        Box2DObjectPtr getBox2DObject() const;
        // Box2D specific interface
        Box2DLinkPtr getChildBox2DLink() const;
        Box2DLinkPtr getParentBox2DLink() const;
        Box2DWorldPtr getBox2DWorld() const;
        Eigen::Vector2f getAxisPosition() const;

    protected:
        // ensure we can only create this from friend classes
        Box2DJoint(const Box2DJointDescription& joint_desc,
                   Box2DLinkPtr link_a,
                   Box2DLinkPtr link_b,
                   Box2DWorldPtr world,
                   const std::string& object_name);
        // called when Box2DWorld is destroyed
        void destroy(const std::shared_ptr<b2World>& b2world);
        void setName(const std::string &name) override;
        void setObjectName(const std::string& name);
        void setJointIndex(unsigned int index);
        void setDOFIndex(unsigned int index);
        void resetPosition(float value, bool child_joint_override);
        void setControlTorque(float value);
    private:
        Box2DWorldWeakPtr _world;
        std::string _name;
        Box2DLinkWeakPtr _link_a;
        Box2DLinkWeakPtr _link_b;
        b2Joint* _joint;
        JointType _joint_type;
        bool _destroyed;
        unsigned int _joint_index;
        unsigned int _dof_index;
        Eigen::Array2f _position_limits;
        Eigen::Array2f _velocity_limits;
        Eigen::Array2f _acceleration_limits;
        std::string _object_name;

    };

    /**
     * Box2DObject - This class implements an Object in box2d.
     */
    class Box2DObject : public Object, public Box2DCollidable, public std::enable_shared_from_this<Box2DObject> {
        friend class Box2DWorld;
        friend class Box2DRobot;
    public:
        /**
         * A Box2DObject can only be constructed by a Box2DWorld.
         */
        Box2DObject() = delete;
        //TODO what about copy constructor?
        ~Box2DObject();

        virtual std::string getName() const override;
        virtual EntityType getType() const override;
        virtual Eigen::Affine3f getTransform() const override;
        virtual WorldPtr getWorld() const override;
        virtual WorldConstPtr getConstWorld() const override;
        Box2DWorldPtr getBox2DWorld() const;
        Eigen::Vector3f getPose() const;
        void getPose(Eigen::Vector3f& pose) const;
        virtual void setTransform(const Eigen::Affine3f& tf) override;

        bool checkCollision() override;
        bool checkCollision(std::vector<Contact> &contacts) override;
        bool checkCollision(ObjectPtr other_object) override;
        bool checkCollision(ObjectPtr other_object, std::vector<Contact>& contacts) override;
        bool checkCollision(const std::vector<ObjectPtr> &other_objects) override;
        bool checkCollision(const std::vector<ObjectPtr> &other_objects, std::vector<Contact> &contacts) override;

        virtual void setActiveDOFs(const Eigen::VectorXi& indices) override;
        virtual Eigen::VectorXi getActiveDOFs() const override;
        virtual unsigned int getNumActiveDOFs() const override;
        virtual unsigned int getNumDOFs() const override;
        unsigned int getNumBaseDOFs() const override;
        virtual Eigen::VectorXi getDOFIndices() const override;
        virtual Eigen::VectorXf getDOFPositions(const Eigen::VectorXi& indices=Eigen::VectorXi()) const override;
        virtual Eigen::ArrayX2f getDOFPositionLimits(const Eigen::VectorXi& indices=Eigen::VectorXi()) const override;

        virtual DOFInformation getDOFInformation(unsigned int dof_index) const;
        virtual void getDOFInformation(unsigned int dof_index, DOFInformation& info) const;
        virtual void setDOFPositions(const Eigen::VectorXf& values, const Eigen::VectorXi& indices=Eigen::VectorXi()) override;

        virtual Eigen::VectorXf getDOFVelocities(const Eigen::VectorXi& indices=Eigen::VectorXi()) const override;
        virtual Eigen::ArrayX2f getDOFVelocityLimits(const Eigen::VectorXi& indices=Eigen::VectorXi()) const override;
        virtual Eigen::ArrayX2f getDOFAccelerationLimits(const Eigen::VectorXi& indices=Eigen::VectorXi()) const override;

        virtual void setDOFVelocities(const Eigen::VectorXf& values, const Eigen::VectorXi& indices=Eigen::VectorXi()) override;

        virtual bool isStatic() const override;

        virtual void getLinks(std::vector<LinkPtr>& links) override;
        virtual void getLinks(std::vector<LinkConstPtr>& links) const override;
        virtual LinkPtr getLink(const std::string &link_name) override;
        virtual LinkConstPtr getConstLink(const std::string &link_name) const override;
        virtual LinkPtr getBaseLink() override;

        virtual void getJoints(std::vector<JointPtr>& joints) override;
        virtual void getJoints(std::vector<JointConstPtr>& joints) const override;
        JointPtr getJoint(const std::string &joint_name) override;
        JointPtr getJoint(unsigned int joint_idx) override;
        JointPtr getJointFromDOFIndex(unsigned int dof_idx) override;
        JointConstPtr getConstJoint(const std::string &joint_name) const override;
        JointConstPtr getConstJoint(unsigned int joint_idx) const override;
        JointConstPtr getConstJointFromDOFIndex(unsigned int dof_idx) const override;

        // Box2D specific methods
        /**
         * Returns the total mass of this object.
         * (i.e. the sum of the mass of all its bodies)
         * @return mass
         */
        float getMass() const; //TODO maybe make it part of the sim_env interface
        /**
         * Returns the moment of inertia of this object (given the current configuration).
         * @return moment of inertia
         */
        float getInertia() const; //TODO maybe make it part of the sim_env interface (with Eigen::MatrixXf as return type?)
        virtual Box2DLinkPtr getBox2DBaseLink();
        void getBox2DLinks(std::vector<Box2DLinkPtr>& links);

        void getState(ObjectState &object_state) const override;

        ObjectState getState() const override;

        void setState(const ObjectState &object_state) override;


    protected:
        // ensure only friend classes can construct this
        Box2DObject(const Box2DObjectDescription& obj_desc, Box2DWorldPtr world);
        // function for destruction. called by Box2DWorld when it is destroyed.
        void destroy(const std::shared_ptr<b2World>& b2world);
        virtual void setName(const std::string &name) override;
        Box2DJointPtr getBox2DJoint(unsigned int idx);
        // puts all link bodies into the given list (for collision checking)
        void getBodies(std::vector<b2Body *> &bodies) override;
        LinkPtr getLink(b2Body* body) override;

        // these can be overwritten by Box2DRobot
        DOFInformation _x_dof_info;
        DOFInformation _y_dof_info;
        DOFInformation _theta_dof_info;
    private:
        std::string _name;
        Box2DWorldWeakPtr _world;
        float _mass;
        Eigen::VectorXi _active_dof_indices;
        unsigned int _num_dofs;
        std::map<std::string, Box2DLinkPtr> _links; // the object is responsible for the lifetime of its links
        Box2DLinkPtr _base_link;
        std::map<std::string, Box2DJointPtr> _joints; // the object is responsible for the lifetime of its links
        std::vector<Box2DJointPtr> _sorted_joints; // sorted list of objects
        bool _is_static;
        bool _destroyed;
    };

    /**
     * Box2DRobot - This class implements a Robot in box2d.
     * See sim_env::Object and sim_env::Robot for documentation.
     */
    class Box2DRobot : public Robot, public Box2DCollidable, public std::enable_shared_from_this<Box2DRobot>{
        friend class Box2DWorld;
    public:
        /**
         * A Box2DRobot can only be instantiated by a Box2DWorld.
         */
        Box2DRobot() = delete;
        ~Box2DRobot();
        // entity functions
        virtual std::string getName() const override;
        void setTransform(const Eigen::Affine3f &tf) override;
        EntityType getType() const override;
        Eigen::Affine3f getTransform() const override;
        WorldPtr getWorld() const override;
        WorldConstPtr getConstWorld() const override;
        // DOFs
        void setActiveDOFs(const Eigen::VectorXi &indices) override;
        virtual unsigned int getNumDOFs() const override;
        virtual unsigned int getNumActiveDOFs() const override;
        unsigned int getNumBaseDOFs() const override;
        virtual DOFInformation getDOFInformation(unsigned int dof_index) const;
        virtual void getDOFInformation(unsigned int dof_index, DOFInformation& info) const;
        Eigen::VectorXi getActiveDOFs() const override;
        Eigen::VectorXi getDOFIndices() const override;
        Eigen::VectorXf getDOFPositions(const Eigen::VectorXi &indices=Eigen::VectorXi()) const override;
        virtual Eigen::ArrayX2f getDOFPositionLimits(const Eigen::VectorXi& indices=Eigen::VectorXi()) const override;
        void setDOFPositions(const Eigen::VectorXf &values, const Eigen::VectorXi &indices=Eigen::VectorXi()) override;
        Eigen::VectorXf getDOFVelocities(const Eigen::VectorXi &indices=Eigen::VectorXi()) const override;
        virtual Eigen::ArrayX2f getDOFVelocityLimits(const Eigen::VectorXi& indices=Eigen::VectorXi()) const override;
        void setDOFVelocities(const Eigen::VectorXf &values, const Eigen::VectorXi &indices=Eigen::VectorXi()) override;
        virtual Eigen::ArrayX2f getDOFAccelerationLimits(const Eigen::VectorXi& indices=Eigen::VectorXi()) const override;
        bool isStatic() const override;
        // collision checking
        bool checkCollision() override;
        bool checkCollision(std::vector<Contact> &contacts) override;
        bool checkCollision(ObjectPtr other_object) override;
        bool checkCollision(ObjectPtr other_object, std::vector<Contact>& contacts) override;
        bool checkCollision(const std::vector<ObjectPtr> &other_objects) override;
        bool checkCollision(const std::vector<ObjectPtr> &other_objects, std::vector<Contact> &contacts) override;
        // links
        virtual void getLinks(std::vector<LinkPtr>& links) override;
        virtual void getLinks(std::vector<LinkConstPtr>& links) const override;
        virtual void getBox2DLinks(std::vector<Box2DLinkPtr>& links);
        virtual LinkPtr getLink(const std::string &link_name) override;
        virtual LinkConstPtr getConstLink(const std::string &link_name) const override;
        virtual LinkPtr getBaseLink() override;
        Box2DLinkPtr getBox2DBaseLink();
        // joints
        virtual void getJoints(std::vector<JointPtr>& joints) override;
        virtual void getJoints(std::vector<JointConstPtr>& joints) const override;
        virtual JointPtr getJoint(const std::string &joint_name) override;
        virtual JointConstPtr getConstJoint(const std::string &joint_name) const override;
        JointPtr getJoint(unsigned int joint_idx) override;
        JointPtr getJointFromDOFIndex(unsigned int dof_idx) override;
        JointConstPtr getConstJoint(unsigned int joint_idx) const override;
        JointConstPtr getConstJointFromDOFIndex(unsigned int dof_idx) const override;
        // control
        virtual void setController(ControlCallback control_fn) override;
        float getMass() const;
        float getInertia() const;
        // state retrieval
        void getState(ObjectState &object_state) const override;
        ObjectState getState() const override;
        void setState(const ObjectState &object_state) override;

    protected:
        // protected constructor to ensure construction is only done by friend classes
        Box2DRobot(const Box2DRobotDescription &robot_desc, Box2DWorldPtr world);
        // destruction is issued by Box2DWorld
        void destroy(const std::shared_ptr<b2World>& b2world);
        virtual void setName(const std::string &name) override;
        // calls control callbacks. should be called only by step function of Box2DWorld
        void control(float timestep);
        // retrieve all box2d bodies (for collision checking)
        void getBodies(std::vector<b2Body *> &bodies) override;
        LinkPtr getLink(b2Body* body) override;

    private:
        bool _destroyed;
        Box2DObjectPtr _robot_object;
        mutable std::recursive_mutex _controller_mutex; // for locks to access _controller_callback
        ControlCallback _controller_callback;

        // applies the given efforts to the underlying Box2D bodies. Locks the Box2D world.
        void commandEfforts(const Eigen::VectorXf& target);
    };

    class Box2DCollisionChecker : public b2ContactListener {
    public:
        Box2DCollisionChecker(Box2DWorldPtr world);
        ~Box2DCollisionChecker();
        /**
         * Checks whether both provided collidables collide.
         * @param collidable_a
         * @param collidable_b
         * @return true iff there is a collision between both collidables
         */
        bool checkCollision(Box2DCollidablePtr collidable_a, Box2DCollidablePtr collidable_b);

        /**
         * Checks whether both provided collidables collide.
         * @param collidable_a
         * @param collidable_b
         * @param contacts - all detected contacts are stored in this list
         * @return true iff there is a collision between both collidables
         */
        bool checkCollision(Box2DCollidablePtr collidable_a, Box2DCollidablePtr collidable_b, std::vector<Contact>& contacts);

        /**
         * Checks whether both provided collidables collide.
         * @param collidable_a
         * @return true iff collidable is in collision with any other object
         */
        bool checkCollision(Box2DCollidablePtr collidable);

        /**
         * Checks whether both provided collidables collide.
         * @param collidable_a
         * @param contacts - all detected contacts are stored in this list
         * @return true iff collidable is in collision with any other object
         */
        bool checkCollision(Box2DCollidablePtr collidable, std::vector<Contact>& contacts);

        /**
         * Checks whether collidable_a is in collision with any of the provided collidables
         * @param collidable_a
         * @param collidables
         * @return  true iff collidable_a collides with any of the provided objects in collidables
         */
        bool checkCollision(Box2DCollidablePtr collidable_a, const std::vector<Box2DCollidablePtr>& collidables);

        /**
         * Checks whether collidable_a is in collision with any of the provided collidables
         * @param collidable_a
         * @param collidables
         * @param contacts - all detected contacts are stored in this list
         * @return  true iff collidable_a collides with any of the provided objects in collidables
         */
        bool checkCollision(Box2DCollidablePtr collidable_a,
                            const std::vector<Box2DCollidablePtr>& collidables,
                            std::vector<Contact>& contacts);

        // b2ContactListener
        void BeginContact(b2Contact* contact);
        void EndContact(b2Contact* contact);
        void PreSolve(b2Contact* contact, const b2Manifold* oldManifold);
        void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse);
    private:
        Box2DWorldWeakPtr _weak_world;
        bool _record_contacts;
        float _scale;
        typedef std::map<b2Body*, Contact> ContactMap;
        std::map<b2Body*, ContactMap> _contact_maps;
        std::map<b2Body*, LinkWeakPtr> _body_to_link_map;

        // TODO instead of always updating all contacts, we could listen to the world and only update
        // TODO contacts if there were changes
        void updateContacts();
        // TODO maybe we can make some of these inline?
        void addContact(b2Body* body_a, b2Body* body_b, b2Contact* contact);
        bool areInContact(b2Body* body_a, b2Body* body_b) const;
        bool hasContacts(b2Body* body) const;
        LinkPtr getLink(b2Body* body, Box2DCollidablePtr collidable);
    };

    /**
     * Box2DWorld - This class implements the World interface for box2d.
     */
    class Box2DWorld : public World, public std::enable_shared_from_this<Box2DWorld> {
        friend class Box2DLink; // these classes need access to the underlying Box2D world
        friend class Box2DJoint;
        friend class Box2DObject;
        friend class Box2DRobot;
        friend class Box2DCollisionChecker;
    public:
        static constexpr float GROUND_DEFAULT_MIN_X = -100.0f;
        static constexpr float GROUND_DEFAULT_MIN_Y = -100.0f;
        static constexpr float GROUND_DEFAULT_MAX_X = 100.0f;
        static constexpr float GROUND_DEFAULT_MAX_Y = 100.0f;
        static constexpr float GRAVITY = 9.81f;
        Box2DWorld();
        Box2DWorld(const Box2DWorld& world) = delete;
        Box2DWorld& operator=(const Box2DWorld& world) = delete;
        Box2DWorld& operator=(Box2DWorld&&) = delete;
        // Destroys all its entities
        ~Box2DWorld(); // we can ignore the warning that we are hiding enable_shared_from_this' destructor

        void loadWorld(const std::string &path) override;

        RobotPtr getRobot(const std::string &name) const override;
        Box2DRobotPtr getBox2DRobot(const std::string& name) const;
        void getRobots(std::vector<RobotPtr> &robots) const override;

        ObjectPtr getObject(const std::string &name, bool exclude_robots=true) const override;
        Box2DObjectPtr getBox2DObject(const std::string& name) const;
        void getObjects(std::vector<ObjectPtr> &objects, bool exclude_robots=true) const override;

        void stepPhysics(int steps) override;
        bool supportsPhysics() const override;
        void setPhysicsTimeStep(float physics_step) override;
        void setVelocitySteps(int velocity_steps);
        void setPositionSteps(int position_steps);
        float getPhysicsTimeStep() const override;
        int getVelocitySteps() const;
        int getPositionSteps() const;

        WorldViewerPtr getViewer() override;
        LoggerPtr getLogger() override;

        bool checkCollision(LinkPtr link) override;
        bool checkCollision(LinkPtr link, std::vector<Contact> &contacts) override;
        bool checkCollision(LinkPtr link, const std::vector<LinkPtr> &other_links) override;
        bool checkCollision(LinkPtr link, const std::vector<LinkPtr> &other_links,
                            std::vector<Contact> &contacts) override;
        bool checkCollision(ObjectPtr object_a, ObjectPtr object_b) override;
        bool checkCollision(ObjectPtr object_a, ObjectPtr object_b, std::vector<Contact> &contacts) override;
        bool checkCollision(ObjectPtr object_a, LinkPtr link_b) override;
        bool checkCollision(ObjectPtr object_a, LinkPtr link_b, std::vector<Contact> &contacts) override;
        bool checkCollision(ObjectPtr object) override;
        bool checkCollision(ObjectPtr object, std::vector<Contact> &contacts) override;
        bool checkCollision(ObjectPtr object_a, const std::vector<ObjectPtr> &other_objects) override;
        bool checkCollision(ObjectPtr object_a, const std::vector<ObjectPtr> &other_objects,
                            std::vector<Contact> &contacts) override;
        bool checkCollision(LinkPtr link_a, const std::vector<ObjectPtr> &other_objects) override;
        bool checkCollision(LinkPtr link_a, const std::vector<ObjectPtr> &other_objects,
                            std::vector<Contact> &contacts) override;
        std::recursive_mutex& getMutex() const;
        void saveState();
        bool restoreState();
        WorldState getWorldState() const override;
        void getWorldState(WorldState &state) const override;
        bool setWorldState(WorldState &state) override;

        // Box2D specific
        float getScale() const;
        float getInverseScale() const;
        float getGravity() const;

    protected:
        std::shared_ptr<b2World> getRawBox2DWorld();
        b2Body* getGroundBody();
        LinkPtr getLink(b2Body* body);
    private:
        mutable std::recursive_mutex _world_mutex;
        b2Body* _b2_ground_body;
        std::shared_ptr<b2World> _world;
        LoggerPtr _logger;
        float _scale;
        float _time_step;
        int _velocity_steps;
        int _position_steps;
        std::map<std::string, Box2DObjectPtr> _objects;
        std::map<std::string, Box2DRobotPtr> _robots;
        Box2DCollisionCheckerPtr _collision_checker;
        std::stack<WorldState> _state_stack;

        void eraseWorld();
        void createWorld(const Box2DEnvironmentDescription& env_desc);
        void createNewRobot(const Box2DRobotDescription& robot_desc);
        void createNewObject(const Box2DObjectDescription& object_desc);
        void createGround(const Eigen::Vector4f& world_bounds);
        Box2DCollidablePtr castCollidable(ObjectPtr object) const;
        Box2DCollidablePtr castCollidable(LinkPtr link) const;
    };

}

#endif //BOX2D_SIM_ENV_BOX2DWORLD_H
