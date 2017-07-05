//
// Created by joshua on 4/26/17.
//

#ifndef BOX2D_SIM_ENV_BOX2DIOUTILS_H_H
#define BOX2D_SIM_ENV_BOX2DIOUTILS_H_H

#include <string>
#include <vector>
#include "sim_env/YamlUtils.h"
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

namespace sim_env {
    struct Box2DLinkDescription {
        std::string name;
        std::vector< std::vector<float> > polygons;
        float mass;
        float trans_friction;
        float rot_friction;
        float contact_friction;
        float restitution;
    };

    struct Box2DJointDescription {
        std::string name;
        std::string link_a;
        std::string link_b;
        Eigen::Vector2f axis;
        Eigen::Vector2f limits;
        std::string joint_type;
        float max_torque;
        bool actuated;
        float axis_orientation;
    };

    struct Box2DObjectDescription {
        std::string name;
        std::string base_link;
        std::vector<Box2DLinkDescription> links;
        std::vector<Box2DJointDescription> joints;
        bool is_static;
    };

    struct Box2DStateDescription {
        Eigen::VectorXf configuration;
        Eigen::VectorXf velocity;
    };

    struct Box2DEnvironmentDescription {
        float scale;
        Eigen::Vector4f world_bounds;
        std::map<std::string, Box2DStateDescription> states;
        std::vector<Box2DObjectDescription> robots;
        std::vector<Box2DObjectDescription> objects;
    };

    static void parseObjectDescriptions(const YAML::Node& node, const boost::filesystem::path& root_path,
                                 std::vector<Box2DObjectDescription>& obj_descs) {
        // run over each object description
        for (auto yaml_entry : node) {
            std::string object_name;
            if (yaml_entry["name"]) {
                object_name = yaml_entry["name"].as<std::string>();
            }
            std::string object_file = yaml_entry["filename"].as<std::string>();
            // first check whether we have a relative path or not, if so resolve it
            boost::filesystem::path object_path(object_file);
            if (!object_path.is_absolute()) {
                object_path = boost::filesystem::canonical(object_path, root_path);
                object_file = object_path.string();
            }
            // now load the object
            YAML::Node object_node = YAML::LoadFile(object_file);
            Box2DObjectDescription object_desc = object_node.as<Box2DObjectDescription>();
            if (object_name.size() > 0) {
                object_desc.name = object_name;
            }
            obj_descs.push_back(object_desc);
        }
    }

    static void parseYAML(const std::string& filename, Box2DEnvironmentDescription& ed) {
        // First get the root path of our environment file, we might it to resolve relative paths.
        sim_env::LoggerPtr logger = sim_env::DefaultLogger::getInstance();
        boost::filesystem::path root_path(filename);
        root_path = root_path.parent_path();
        // load file
        YAML::Node node = YAML::LoadFile(filename);
        if (node["scale"]) {
            ed.scale = node["scale"].as<float>();
        } else {
            ed.scale = 1.0;
            logger->logWarn("No scale specified. Using default scale (1.0).", "sim_env/Box2DIOUtils.h");
        }
        if (node["world_bounds"]) {
            ed.world_bounds = node["world_bounds"].as<Eigen::Vector4f>();
        } else {
            ed.world_bounds << 0.0f, 0.0f, 0.0f, 0.0f;
            logger->logWarn("No world bounds specified. Using default world bounds.", "sim_env/Box2DIOUtils.h");
        }
        parseObjectDescriptions(node["robots"], root_path, ed.robots);
        parseObjectDescriptions(node["objects"], root_path, ed.objects);
        for (auto yaml_state : node["states"]) {
            std::string object_name = yaml_state["name"].as<std::string>();
            ed.states[object_name] = yaml_state["state"].as<Box2DStateDescription>();
        }
        //TODO do some sanity checks, e.g. check that we have a state for each movable object
    }
}

namespace YAML {
    template<>
    struct convert<sim_env::Box2DStateDescription> {
        static Node encode(const sim_env::Box2DStateDescription& sd) {
            Node node;
            node["configuration"] = sd.configuration;
            node["velocity"] = sd.velocity;
            return node;
        }

        static bool decode(const Node& node, sim_env::Box2DStateDescription& sd) {
            sd.configuration = node["configuration"].as<Eigen::VectorXf>();
            sd.velocity = node["velocity"].as<Eigen::VectorXf>();
            return true;
        }
    };

    template<>
    struct convert<sim_env::Box2DLinkDescription> {
        static Node encode(const sim_env::Box2DLinkDescription& ld) {
            sim_env::LoggerPtr logger = sim_env::DefaultLogger::getInstance();
            Node node;
            node["name"] = ld.name;
            for (auto& polygon : ld.polygons) {
                if (polygon.size() % 2 != 0) {
                    logger->logErr("Invalid polygon encountered. A polygon must have an even number of floats.",
                                   "sim_env/Box2DIOUtils.h");
                    // TODO we should do a sanity check here, whether the geometry is counter clock wise
                    continue;
                }
                node["geometry"].push_back(polygon);
            }
            node["mass"] = ld.mass;
            node["trans_friction"] = ld.trans_friction;
            node["rot_friction"] = ld.rot_friction;
            node["contact_friction"] = ld.contact_friction;
            node["restitution"] = ld.restitution;
            return node;
        }

        static bool decode(const Node& node, sim_env::Box2DLinkDescription& ld) {
            ld.name = node["name"].as<std::string>();
            ld.mass = node["mass"].as<float>();
            ld.contact_friction = node["contact_friction"].as<float>();
            ld.restitution = node["restitution"].as<float>();
            ld.rot_friction = node["rot_friction"].as<float>();
            ld.trans_friction = node["trans_friction"].as<float>();
            ld.polygons.clear();
            for (auto& polygon : node["geometry"]) {
                ld.polygons.push_back(polygon.as< std::vector<float> >());
            }
            return true;
        }
    };

    template<>
    struct convert<sim_env::Box2DJointDescription> {
        static Node encode(const sim_env::Box2DJointDescription& jd) {
            Node node;
            node["name"] = jd.name;
            node["link_a"] = jd.link_a;
            node["link_b"] = jd.link_b;
            node["axis"] = jd.axis;
            node["limits"] = jd.limits;
            node["max_torque"] = jd.max_torque;
            node["actuated"] = jd.actuated;
            node["joint_type"] = jd.joint_type;
            node["axis_orientation"] = jd.axis_orientation;
            return node;
        }

        static bool decode(const Node& node, sim_env::Box2DJointDescription& ld) {
            // TODO see what the error messages here if elements are missing
            ld.name = node["name"].as<std::string>();
            ld.joint_type = node["joint_type"].as<std::string>();
            ld.axis_orientation = node["axis_orientation"].as<float>();
            ld.link_a = node["link_a"].as<std::string>();
            ld.link_b = node["link_b"].as<std::string>();
            ld.axis = node["axis"].as<Eigen::Vector2f>();
            ld.limits = node["limits"].as<Eigen::Vector2f>();
            ld.max_torque = node["max_torque"].as<float>();
            ld.actuated = node["actuated"].as<bool>();
            return true;
        }
    };

    template<>
    struct convert<sim_env::Box2DObjectDescription> {
        static Node encode(const sim_env::Box2DObjectDescription& od) {
            Node node;
            node["name"] = od.name;
            for (auto& link : od.links) {
                node["links"].push_back(link);
            }
            for (auto& joint : od.joints) {
                node["joints"].push_back(joint);
            }
            node["static"] = false;
            return node;
        }

        static bool decode(const Node& node, sim_env::Box2DObjectDescription& od) {
            od.name = node["name"].as<std::string>();
            // read in links
            const YAML::Node& links_node = node["links"];
            for (YAML::const_iterator iter = links_node.begin(); iter!= links_node.end(); iter++) {
                od.links.push_back(iter->as<sim_env::Box2DLinkDescription>());
            }
            // read in joints
            const YAML::Node& joints_node = node["joints"];
            for (YAML::const_iterator iter = joints_node.begin(); iter!= joints_node.end(); iter++) {
                od.joints.push_back(iter->as<sim_env::Box2DJointDescription>());
            }
            if (node["static"]) {
                od.is_static = true;
            } else {
                od.is_static = false;
            }
            // finally verify that we have a sane kinematic structure
            std::string base_link_name;
            if (not verifyKinematicStructure(od, base_link_name)) {
                auto logger = sim_env::DefaultLogger::getInstance();
                std::stringstream msg_stream;
                msg_stream << "The Kinematic structure of object " << od.name << " is invalid.";
                logger->logErr(msg_stream.str(), "[Box2DIOUtils.h::convert<sim_env::Box2DObjectDescription>::decode]");
                return false;
            }
            od.base_link = base_link_name;
            return true;
        }

        static bool verifyKinematicStructure(const sim_env::Box2DObjectDescription& od, std::string& base_link_name) {
            auto logger = sim_env::DefaultLogger::getInstance();
            std::stringstream msg_stream;
            std::string prefix("[Box2DIOUtils.h::convert<sim_env::Box2DObjectDescription>::verifyKinematicStructure]");
            std::map<std::string, int> num_parents;
            // initialize map with 0 for each link
            for (auto& link_desc : od.links) {
                num_parents[link_desc.name] = 0;
            }
            // test whether all links declared in the joint descriptions exist and count parents
            for (auto& joint_desc : od.joints) {
                auto iter_num_parents = num_parents.find(joint_desc.link_a);
                if (iter_num_parents == num_parents.end()) {
                    msg_stream << "Parent link \"" << joint_desc.link_a << "\"of joint \"" << joint_desc.name << "\" is unknown.";
                    msg_stream << "The link needs to be defined in the links description.";
                    logger->logErr(msg_stream.str(), prefix);
                    return false;
                }
                iter_num_parents = num_parents.find(joint_desc.link_b);
                if (iter_num_parents == num_parents.end()) {
                    msg_stream << "Parent link \"" << joint_desc.link_b << "\"of joint \"" << joint_desc.name << "\" is unknown.";
                    msg_stream << "The link needs to be defined in the links description.";
                    logger->logErr(msg_stream.str(), prefix);
                    return false;
                }
                iter_num_parents->second += 1;
            }
            // finally ensure that all links have at most one parent and there is exactly one link without a parent
            bool b_found_root = false;
            for (auto& link_desc : od.links) {
                int link_num_parents = num_parents[link_desc.name];
                if (link_num_parents == 0) {
                    if (b_found_root) {
                        msg_stream << "Found an additional link with no parent: \"" << link_desc.name << "\"";
                        msg_stream << "Previous found root link is \"" <<  base_link_name << "\"";
                        logger->logErr(msg_stream.str(), prefix);
                        return false;
                    } else {
                        b_found_root = true;
                        base_link_name = link_desc.name;
                    }
                }
            }
            return true;
        }
    };
}
#endif //BOX2D_SIM_ENV_BOX2DIOUTILS_H_H
