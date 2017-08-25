//
// Created by joshua on 7/28/17.
//
#include <queue>
#include "sim_env/Box2DController.h"
#include "sim_env/utils/EigenUtils.h"

using namespace sim_env;

Box2DRobotVelocityController::Box2DRobotVelocityController(Box2DRobotPtr robot) {
    _box2d_robot = robot;
    _b_target_available = false;
}

Box2DRobotVelocityController::~Box2DRobotVelocityController() {
}

unsigned int Box2DRobotVelocityController::getTargetDimension() {
    Box2DRobotPtr robot = lockRobot();
    return robot->getNumActiveDOFs();
}

void Box2DRobotVelocityController::setTargetVelocity(const Eigen::VectorXf& velocity) {

    Box2DRobotPtr robot = lockRobot();
    LoggerPtr logger = robot->getWorld()->getLogger();
    if (velocity.size() != robot->getNumActiveDOFs()) {
        throw std::runtime_error("[sim_env::Box2DRobotVelocityController::setTargetVelocity]"
                                         "Provided target velocity has invalid dimension.");
    }
    _target_velocity = velocity;
    utils::eigen::ScalingResult scaling_result = utils::eigen::scaleToLimits(_target_velocity,
                                                                             robot->getDOFVelocityLimits());
    if (scaling_result == utils::eigen::ScalingResult::Failure) {
        logger->logErr("Could not scale input velocity to limits.",
                       "[sim_env::Box2DRObotVelocityController::setTargetVelocity]");
        // TODO do we want to throw an exception here?
    }
//    std::stringstream ss;
//    ss << "Target velocity is " << _target_velocity.transpose();
//    logger->logDebug(ss.str());
    _b_target_available = true;
}

bool Box2DRobotVelocityController::control(const Eigen::VectorXf& positions,
                                           const Eigen::VectorXf& velocities,
                                           float timestep,
                                           RobotConstPtr robot,
                                           Eigen::VectorXf& output) {
    std::string prefix("[sim_env::Box2DRobotVelocityController::control]");
    LoggerPtr logger = getLogger();
    if (not _b_target_available) {
        // only do anything if we have a target
        logger->logWarn("No target available.", prefix);
        return false;
    }
    Box2DRobotPtr box2d_robot = lockRobot();
    if (robot != box2d_robot) {
        // check whether we have the correct robot
        logger->logErr("The provided robot is different from the robot this controller is created for.",
                       prefix);
        return false;
    }
    Eigen::VectorXi active_dofs = box2d_robot->getActiveDOFs();
    Eigen::MatrixX2f acceleration_limits = box2d_robot->getDOFAccelerationLimits();
    output.resize(active_dofs.size());
    // control each dof independently
    for (unsigned int i = 0; i < active_dofs.size(); ++i) {
        float mass_inertia = 0.0f;
        int dof_idx = active_dofs[i];
        assert((0 <= dof_idx) and (dof_idx < box2d_robot->getNumDOFs()));
        if (dof_idx < box2d_robot->getNumBaseDOFs()) { // if the dof is x, y or theta
            if (dof_idx == 0 or dof_idx == 1) {
                mass_inertia = box2d_robot->getMass();
            } else {
                mass_inertia = box2d_robot->getInertia();
            }
        } else {
            JointPtr root_joint = box2d_robot->getJointFromDOFIndex(dof_idx);
            mass_inertia = computeKinematicChainInertia(root_joint);
        }
        // compute output forces / torques
        // we would like to reach the velocity in the next time step:
        float accel = (_target_velocity[i] - velocities[i]) / timestep;
        // we need to limit the acceleration though
        accel = std::min(acceleration_limits(i, 1), std::max(accel, acceleration_limits(i, 0)));
        output[i] = mass_inertia * accel; // make it a force / torque
    }
    return true;
}

Box2DRobotPtr Box2DRobotVelocityController::lockRobot() {
    Box2DRobotPtr robot = _box2d_robot.lock();
    if (!robot) {
        LoggerPtr logger = DefaultLogger::getInstance();
        logger->logErr("Could not access Box2D robot. Instance does not exist anymore.",
                       "[sim_env::Box2DRobotVelocityController::lockRobot]");
        throw std::logic_error("[sim_env::Box2DRobotVelocityController::lockRobot] Weak pointer to Box2DRobot not valid anymore.");
    }
    return robot;
}

float Box2DRobotVelocityController::computeKinematicChainInertia(JointPtr root_joint) {
    Box2DJointPtr box2d_root_joint = std::dynamic_pointer_cast<Box2DJoint>(root_joint);
    if (not box2d_root_joint) {
        throw std::logic_error("[sim_env::Box2DRobotVelocityController::computeKinematicChainInertia] Could not cast joint"
                                       " to Box2DJoint. This controller only works with Box2DRobots!");
    }
    Eigen::Vector2f root_axis = box2d_root_joint->getAxisPosition();
    float inertia = 0.0f;
    std::queue<JointPtr> joint_queue;
    joint_queue.push(root_joint);
    // run over the whole kinematic sub-chain to extract the inertia
    while (not joint_queue.empty()) {
        auto current_joint = joint_queue.front();
        Box2DLinkPtr child_link = std::dynamic_pointer_cast<Box2DLink>(current_joint->getChildLink());
        if (not child_link) {
            throw std::logic_error("[sim_env::Box2DRObotVelocityController::computeKinematicChainInertia]"
            " Could not cast LinkPtr to Box2DLinkPtr. This means the robot is really messed up!");
        }
        float distance = (child_link->getCenterOfMass() - root_axis).norm();
        inertia += child_link->getInertia() + child_link->getMass() * distance * distance; // parallel axis theorem
        joint_queue.pop();
    }
    return inertia;
}

LoggerPtr Box2DRobotVelocityController::getLogger() const {
    Box2DRobotPtr robot = _box2d_robot.lock();
    if (not robot) {
        return DefaultLogger::getInstance();
    }
    return robot->getWorld()->getLogger();
}

