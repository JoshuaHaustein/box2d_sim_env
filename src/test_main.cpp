//
// Created by joshua on 4/20/17.
//
#include <iostream>
#include"sim_env/Box2DWorld.h"

// TODO should implement unit tests using Google Test instead of this file
void testLogger() {
    sim_env::Box2DWorld my_test_world;
    //TODO implement simple manual testing stuff here
    sim_env::LoggerPtr logger = my_test_world.getLogger();
    logger->setLevel(sim_env::Logger::LogLevel::Debug);
    logger->logInfo("This is an info message.");
    logger->logWarn("This is a warning", "Muhahaha");
    logger->logErr("This is a fake error", "The cake is a lie");
    logger->logDebug("This is Sparta! eh Debug");
    // Let's try error lvl
    logger->setLevel(sim_env::Logger::LogLevel::Error);
    logger->logInfo("This should not be printed.");
    logger->logWarn("This should not be printed.", "Muhahaha");
    logger->logErr("This is a fake error", "The cake is a lie");
    logger->logDebug("This should not be printed.");
    logger->setLevel(sim_env::Logger::LogLevel::Info);
    logger->logInfo("This is an info message.");
    logger->logWarn("This is a warning", "Muhahaha");
    logger->logErr("This is a fake error", "The cake is a lie");
    logger->logDebug("This should not be printed.");
    logger->setLevel(sim_env::Logger::LogLevel::Warn);
    logger->logInfo("This should not be printed.");
    logger->logWarn("This is a warning", "Muhahaha");
    logger->logErr("This is a fake error", "The cake is a lie");
    logger->logDebug("This should not be printed.");
}

void testYaml() {
    sim_env::Box2DWorld my_test_world;
    my_test_world.loadWorld("/home/joshua/projects/planning_catkin/src/box2d_sim_env/test_data/test_env.yaml");
}

int main(int argc, char **argv) {
    testLogger();
    testYaml();
    std::cout << "Test finished" << std::endl;
    return 0;
}
