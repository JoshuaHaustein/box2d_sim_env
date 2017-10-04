//
// Created by joshua on 9/5/17.
//
#include <sim_env/Box2DIOUtils.h>

std::string sim_env::resolveFileName(const std::string& file_name, const boost::filesystem::path& root_path) {
    boost::filesystem::path object_path(file_name);
    std::string return_value(file_name);
    if (!object_path.is_absolute()) {
        object_path = boost::filesystem::canonical(object_path, root_path);
        return_value = object_path.string();
    }
    return return_value;
}

void sim_env::parseObjectDescriptions(const YAML::Node& node, const boost::filesystem::path& root_path,
                             std::vector<Box2DObjectDescription>& obj_descs) {
    // run over each object description
    for (auto yaml_entry : node) {
        std::string object_name;
        if (yaml_entry["name"]) {
            object_name = yaml_entry["name"].as<std::string>();
        }
        std::string object_file = yaml_entry["filename"].as<std::string>();
        // first check whether we have a relative path or not, if so resolve it
        object_file = resolveFileName(object_file, root_path);
        // now load the object
        YAML::Node object_node = YAML::LoadFile(object_file);
        Box2DObjectDescription object_desc = object_node.as<Box2DObjectDescription>();
        if (object_name.size() > 0) {
            object_desc.name = object_name;
        }
        if (yaml_entry["static"]) {
            object_desc.is_static = yaml_entry["static"].as<bool>();
        } else {
            object_desc.is_static = false;
        }
        obj_descs.push_back(object_desc);
    }
}

void sim_env::parseRobotDescriptions(const YAML::Node& node, const boost::filesystem::path& root_path,
                                     std::vector<Box2DRobotDescription>& robot_descs){
    // run over each robot description
    for (auto yaml_entry : node) {
        std::string robot_name;
        if (yaml_entry["name"]) {
            robot_name = yaml_entry["name"].as<std::string>();
        }
        std::string robot_file = yaml_entry["filename"].as<std::string>();
        // first check whether we have a relative path or not, if so resolve it
        robot_file = resolveFileName(robot_file, root_path);
        // now load the robot
        YAML::Node robot_node = YAML::LoadFile(robot_file);
        Box2DRobotDescription robot_desc = robot_node.as<Box2DRobotDescription>();
        if (robot_name.size() > 0) {
            robot_desc.object_description.name = robot_name;
        }
        robot_descs.push_back(robot_desc);
    }
}

void sim_env::parseYAML(const std::string& filename, Box2DEnvironmentDescription& ed) {
    // First get the root path of our environment file, we might need it to resolve relative paths.
    sim_env::LoggerPtr logger = sim_env::DefaultLogger::getInstance();
    boost::filesystem::path root_path(filename);
    if (not boost::filesystem::exists(root_path)) {
        logger->logErr(boost::str(boost::format("Could not open the file %s as it does not exist.") % filename));
    }
    root_path = root_path.parent_path();
    // load file
    YAML::Node node = YAML::LoadFile(filename);
    if (node["scale"]) {
        ed.scale = node["scale"].as<float>();
    } else {
        ed.scale = 1.0;
        logger->logWarn("No scale specified. Using default scale (1.0).", "[sim_env/Box2DIOUtils.h]");
    }
    if (node["world_bounds"]) {
        ed.world_bounds = node["world_bounds"].as<Eigen::Vector4f>();
    } else {
        ed.world_bounds << 0.0f, 0.0f, 0.0f, 0.0f;
        logger->logWarn("No world bounds specified. Using default world bounds.", "[sim_env/Box2DIOUtils.h]");
    }
    parseRobotDescriptions(node["robots"], root_path, ed.robots);
    parseObjectDescriptions(node["objects"], root_path, ed.objects);
    for (auto yaml_state : node["states"]) {
        std::string object_name = yaml_state["name"].as<std::string>();
        ed.states[object_name] = yaml_state["state"].as<Box2DStateDescription>();
    }
    //TODO do some sanity checks, e.g. check that we have a state for each movable object
}
