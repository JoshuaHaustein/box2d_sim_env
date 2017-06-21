//
// Created by joshua on 6/21/17.
//

#include"sim_env/Box2DWorld.h"

int main(int argc, char **argv) {
    sim_env::Box2DWorldPtr world = std::make_shared<sim_env::Box2DWorld>();
    world->loadWorld("/home/joshua/projects/planning_catkin/src/box2d_sim_env/test_data/test_env.yaml");
}

