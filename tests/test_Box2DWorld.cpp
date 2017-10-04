//
// Created by joshua on 4/20/17.
//
#include <iostream>
#include <gtest/gtest.h>
#include "sim_env/test/sim_env_world_test.h"
#include"sim_env/Box2DWorld.h"

#define TEST_WORLD_PATH "/home/joshua/projects/planning_catkin/src/box2d_sim_env/test_data/test_env.yaml"
#define OBJECT_NAME_1 "obj1"
#define OBJECT_CONFIGURATION_1(X) X << 1.0, 2.0, 3.0
#define OBJECT_VELOCITY_1(X) X << 0.0, 1.0, 0.2
#define OBJECT_NAME_2 "obj2"
//#define OBJECT_CONFIGURATION_2(X) X << 2.0 << -1.0 << 0.4 << 0.1
//#define OBJECT_VELOCITY_2(X) X << 0.0 << 1.0 << 0.2 << -0.1
#define ROBOT_NAME "my_robot"
#define ROBOT_LINK_NAME "test_robot_finger_l_1"
#define ROBOT_JOINT_NAME "finger_l_joint_1"
#define ROBOT_CONFIGURATION(X) X << 0.0, 6.0, 2.0, 0.1, 0.1, 0.1, 0.1
#define ROBOT_VELOCITY(X) X << 0.0, 1.0, 0.2, 0.1, 0.2, 0.0, 0.1

sim_env::test::WorldTestData createBox2DWorldTestData() {
    sim_env::test::WorldTestData test_data;
    test_data.world = std::make_shared<sim_env::Box2DWorld>();
    test_data.world->loadWorld(TEST_WORLD_PATH);
    test_data.world->getLogger()->setLevel(sim_env::Logger::LogLevel::Debug);
    test_data.object_names = {OBJECT_NAME_1, OBJECT_NAME_2};
    test_data.robot_names = {ROBOT_NAME};
    return test_data;
}

sim_env::test::EntityTestData createBox2DObjectTestData() {
    sim_env::test::EntityTestData test_data;
    test_data.world = std::make_shared<sim_env::Box2DWorld>();
    test_data.world->getLogger()->setLevel(sim_env::Logger::LogLevel::Debug);
    test_data.world->loadWorld(TEST_WORLD_PATH);
    test_data.entity_name = OBJECT_NAME_1;
    test_data.initial_transform = Eigen::Translation<float, 3>(0.9, 1.0, 0.0) * Eigen::AngleAxisf(-1.0f, Eigen::Vector3f::UnitZ());
    test_data.entity_type = sim_env::EntityType::Object;
    sim_env::ObjectPtr object = test_data.world->getObject(OBJECT_NAME_1);
    test_data.entity = object;
    test_data.object = object;
    test_data.active_dofs = object->getDOFIndices();
    Eigen::VectorXf config(test_data.active_dofs.size());
    OBJECT_CONFIGURATION_1(config);
    test_data.valid_configurations.push_back(config);
    Eigen::VectorXf vel(test_data.active_dofs.size());
    OBJECT_VELOCITY_1(vel);
    test_data.valid_velocities.push_back(vel);
    return test_data;
}

sim_env::test::EntityTestData createBox2DRobotTestData() {
    sim_env::test::EntityTestData test_data;
    test_data.world = std::make_shared<sim_env::Box2DWorld>();
    test_data.world->getLogger()->setLevel(sim_env::Logger::LogLevel::Debug);
    test_data.world->loadWorld(TEST_WORLD_PATH);
    test_data.entity_name = ROBOT_NAME;
    test_data.initial_transform = Eigen::Translation<float, 3>(1.0, 2.0, 0.0) * Eigen::AngleAxisf(0.5, Eigen::Vector3f::UnitZ());
    test_data.entity_type = sim_env::EntityType::Robot;
    sim_env::RobotPtr robot = test_data.world->getRobot(ROBOT_NAME);
    test_data.entity = robot;
    test_data.object = robot;
    test_data.active_dofs = robot->getDOFIndices();
    Eigen::VectorXf config(test_data.active_dofs.size());
    ROBOT_CONFIGURATION(config);
    test_data.valid_configurations.push_back(config);
    Eigen::VectorXf vel(test_data.active_dofs.size());
    ROBOT_VELOCITY(vel);
    test_data.valid_velocities.push_back(vel);
    return test_data;
}

sim_env::test::EntityTestData createBox2DLinkTestData() {
    sim_env::test::EntityTestData test_data;
    test_data.world = std::make_shared<sim_env::Box2DWorld>();
    test_data.world->getLogger()->setLevel(sim_env::Logger::LogLevel::Debug);
    test_data.world->loadWorld(TEST_WORLD_PATH);
    test_data.entity_name = ROBOT_LINK_NAME;
    test_data.initial_transform = Eigen::Translation<float, 3>(0.9, 1.0, 0.0) * Eigen::AngleAxisf(-1.0f, Eigen::Vector3f::UnitZ());
    test_data.entity_type = sim_env::EntityType::Link;
    sim_env::RobotPtr robot = test_data.world->getRobot(ROBOT_NAME);
    test_data.entity = robot->getLink(ROBOT_LINK_NAME);
    return test_data;
}

sim_env::test::EntityTestData createBox2DJointTestData() {
    sim_env::test::EntityTestData test_data;
    test_data.world = std::make_shared<sim_env::Box2DWorld>();
    test_data.world->getLogger()->setLevel(sim_env::Logger::LogLevel::Debug);
    test_data.world->loadWorld(TEST_WORLD_PATH);
    test_data.entity_name = ROBOT_JOINT_NAME;
    // TODO put correct initial transform
    test_data.initial_transform = Eigen::Affine3f();
    test_data.entity_type = sim_env::EntityType::Joint;
    sim_env::RobotPtr robot = test_data.world->getRobot(ROBOT_NAME);
    test_data.entity = robot->getJoint(ROBOT_JOINT_NAME);
    return test_data;
}

namespace sim_env{
    namespace test {
        INSTANTIATE_TEST_CASE_P(
                Box2DWorldTest,
                SimEnvWorldTest,
                testing::Values(&createBox2DWorldTestData));
        INSTANTIATE_TEST_CASE_P(
                Box2DEntityTest,
                SimEnvEntityTest,
                testing::Values(&createBox2DObjectTestData, &createBox2DRobotTestData,
                &createBox2DLinkTestData, &createBox2DJointTestData));
        INSTANTIATE_TEST_CASE_P(
                Box2DObjectTest,
                SimEnvObjectTest,
                testing::Values(&createBox2DObjectTestData, &createBox2DRobotTestData));
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}