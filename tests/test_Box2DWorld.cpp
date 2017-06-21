//
// Created by joshua on 4/20/17.
//
#include <iostream>
#include <gtest/gtest.h>
#include"sim_env/Box2DWorld.h"

namespace {
    // The fixture for testing class Box2DWorld.
    class Box2DWorldTest : public ::testing::Test {
    protected:

        Box2DWorldTest() {
            // You can do set-up work for each test here.
            world = std::make_shared<sim_env::Box2DWorld>();
            robot_name = "my_robot";
            object_names.push_back("obj1");
            object_names.push_back("obj2");
        }

        virtual ~Box2DWorldTest() {
            // You can do clean-up work that doesn't throw exceptions here.
        }

        // If the constructor and destructor are not enough for setting up
        // and cleaning up each test, you can define the following methods:

        virtual void SetUp() {
            // Code here will be called immediately after the constructor (right
            // before each test).
            world->loadWorld("/home/joshua/projects/planning_catkin/src/box2d_sim_env/test_data/test_env.yaml");
        }

        virtual void TearDown() {
            // Code here will be called immediately after each test (right
            // before the destructor).
        }

        // Objects declared here can be used by all tests in the test case for Foo.
        sim_env::Box2DWorldPtr world;
        std::string robot_name;
        std::vector<std::string> object_names;
    };

    // Tests that the test robot is loaded
    TEST_F(Box2DWorldTest, HasRobot) {
        sim_env::RobotPtr robot = world->getRobot(robot_name);
        ASSERT_NE(robot, nullptr);
    }

    TEST_F(Box2DWorldTest, HasObjects) {
        sim_env::ObjectPtr object1 = world->getObject(object_names[0]);
        EXPECT_NE(object1, nullptr);
        EXPECT_EQ(object1->getName(), object_names[0]);
        sim_env::ObjectPtr object2 = world->getObject(object_names[1]);
        EXPECT_NE(object2, nullptr);
        EXPECT_EQ(object2->getName(), object_names[1]);
    }

    TEST_F(Box2DWorldTest, GetTypesWork) {
        sim_env::ObjectPtr object1 = world->getObject(object_names[0]);
        EXPECT_EQ(object1->getType(), sim_env::EntityType::Object);
        sim_env::RobotPtr robot = world->getRobot(robot_name);
        EXPECT_EQ(robot->getType(), sim_env::EntityType::Robot);
    }

}  // namespace

//TEST(LoggerTest, LoggerPrints) {
//    sim_env::Box2DWorld my_test_world;
//    sim_env::LoggerPtr logger = my_test_world.getLogger();
//    logger->setLevel(sim_env::Logger::LogLevel::Debug);
//    logger->logInfo("This is an info message.");
//    logger->logWarn("This is a warning", "Muhahaha");
//    logger->logErr("This is a fake error", "The cake is a lie");
//    logger->logDebug("This is Sparta! eh Debug");
//    // Let's try error lvl
//    logger->setLevel(sim_env::Logger::LogLevel::Error);
//    logger->logInfo("This should not be printed.");
//    logger->logWarn("This should not be printed.", "Muhahaha");
//    logger->logErr("This is a fake error", "The cake is a lie");
//    logger->logDebug("This should not be printed.");
//    logger->setLevel(sim_env::Logger::LogLevel::Info);
//    logger->logInfo("This is an info message.");
//    logger->logWarn("This is a warning", "Muhahaha");
//    logger->logErr("This is a fake error", "The cake is a lie");
//    logger->logDebug("This should not be printed.");
//    logger->setLevel(sim_env::Logger::LogLevel::Warn);
//    logger->logInfo("This should not be printed.");
//    logger->logWarn("This is a warning", "Muhahaha");
//    logger->logErr("This is a fake error", "The cake is a lie");
//    logger->logDebug("This should not be printed.");
//}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}