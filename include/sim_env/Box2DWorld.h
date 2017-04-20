//
// Created by joshua on 4/20/17.
//
#ifndef BOX2D_SIM_ENV_BOX2DWORLD_H
#define BOX2D_SIM_ENV_BOX2DWORLD_H

#include "sim_env/SimEnv.h"

// Forward declarations of Box2D stuff
class b2World;
class b2Body;

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

    class Box2DObject : public Object {
    public:

        Box2DObject(const std::string& object_description, Box2DWorldPtr world);
        ~Box2DObject();

        virtual std::string getName() const override;

        virtual SimEnvEntityType getType() const override;

        virtual Eigen::Affine3d getTransform() const override;

        virtual void setTransform(const Eigen::Affine3d& tf) override;

        virtual WorldPtr getWorld() const override;

        virtual WorldConstPtr getConstWorld() const override;

        virtual bool checkCollision(ObjectConstPtr other_object) const override;

        virtual bool checkCollision(const std::vector<ObjectConstPtr>& object_list) const override ;

        virtual void setActiveDOFs(const Eigen::VectorXi& indices) override;

        virtual Eigen::VectorXi getActiveDOFs() override;

        virtual Eigen::VectorXi getDOFIndices() override;

        virtual Eigen::VectorXd getDOFPositions(const Eigen::VectorXi& indices=Eigen::VectorXi()) const override;

        virtual void setDOFPositions(const Eigen::VectorXd& values, const Eigen::VectorXi& indices=Eigen::VectorXi()) override;

        virtual Eigen::VectorXd getDOFVelocities(const Eigen::VectorXi& indices=Eigen::VectorXi()) const override;

        virtual void setDOFVelocities(const Eigen::VectorXd& values, const Eigen::VectorXi& indices=Eigen::VectorXi()) override;

    private:
        std::string _name;
        Eigen::Affine3d _transform;
        Box2DWorldPtr _world;
        Eigen::VectorXi _active_dof_indices;
        b2Body* _body;
    };

    class Box2DRobot : public Robot {
    public:

    };

    class Box2DWorld : public World {
    public:
        Box2DWorld();
        ~Box2DWorld();

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

    };
}

#endif //BOX2D_SIM_ENV_BOX2DWORLD_H
