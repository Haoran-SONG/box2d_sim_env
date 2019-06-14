//
// Created by joshua on 4/26/17.
//

#ifndef BOX2D_SIM_ENV_BOX2DIOUTILS_H_H
#define BOX2D_SIM_ENV_BOX2DIOUTILS_H_H

#include "sim_env/utils/YamlUtils.h"
#include <Eigen/Dense>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace sim_env {
struct Box2DLinkDescription {
    std::string name;
    std::vector<std::vector<float>> polygons;
    std::vector<std::pair<Eigen::Vector3f, float>> balls;
    float mass;
    float ground_friction;
    float ground_torque_integral;
    float contact_friction;
    float restitution;
};

struct Box2DJointDescription {
    std::string name;
    std::string link_a;
    std::string link_b;
    Eigen::Vector2f axis;
    Eigen::Vector2f position_limits;
    Eigen::Vector2f velocity_limits;
    Eigen::Vector2f acceleration_limits;
    std::string joint_type;
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

struct Box2DRobotDescription {
    Box2DObjectDescription object_description;
    Eigen::Vector2f translational_velocity_limits;
    Eigen::Vector2f translational_acceleration_limits;
    Eigen::Vector2f rotational_velocity_limits;
    Eigen::Vector2f rotational_acceleration_limits;
    bool use_center_of_mass;
};

struct Box2DStateDescription {
    Eigen::VectorXf configuration;
    Eigen::VectorXf velocity;
};

struct Box2DEnvironmentDescription {
    float scale;
    Eigen::Vector4f world_bounds;
    std::map<std::string, Box2DStateDescription> states;
    std::vector<Box2DRobotDescription> robots;
    std::vector<Box2DObjectDescription> objects;
};

std::string resolveFileName(const std::string& file_name, const boost::filesystem::path& root_path);

void parseObjectDescriptions(const YAML::Node& node, const boost::filesystem::path& root_path,
    std::vector<Box2DObjectDescription>& obj_descs);

void parseRobotDescriptions(const YAML::Node& node, const boost::filesystem::path& root_path,
    std::vector<Box2DRobotDescription>& robot_descs);

void parseYAML(const std::string& filename, Box2DEnvironmentDescription& ed);
}

namespace YAML {
template <>
struct convert<sim_env::Box2DStateDescription> {
    static Node encode(const sim_env::Box2DStateDescription& sd)
    {
        Node node;
        node["configuration"] = sd.configuration;
        node["velocity"] = sd.velocity;
        return node;
    }

    static bool decode(const Node& node, sim_env::Box2DStateDescription& sd)
    {
        sd.configuration = node["configuration"].as<Eigen::VectorXf>();
        sd.velocity = node["velocity"].as<Eigen::VectorXf>();
        return true;
    }
};

template <>
struct convert<sim_env::Box2DLinkDescription> {
    static Node encode(const sim_env::Box2DLinkDescription& ld)
    {
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
        node["balls"] = ld.balls;
        node["mass"] = ld.mass;
        node["ground_friction"] = ld.ground_friction;
        node["ground_torque_integral"] = ld.ground_torque_integral;
        node["contact_friction"] = ld.contact_friction;
        node["restitution"] = ld.restitution;
        return node;
    }

    static bool decode(const Node& node, sim_env::Box2DLinkDescription& ld)
    {
        ld.name = node["name"].as<std::string>();
        ld.mass = node["mass"].as<float>();
        ld.contact_friction = node["contact_friction"].as<float>();
        ld.restitution = node["restitution"].as<float>();
        ld.ground_torque_integral = node["ground_torque_integral"].as<float>();
        ld.ground_friction = node["ground_friction"].as<float>();
        ld.polygons.clear();
        for (auto& polygon : node["geometry"]) {
            ld.polygons.push_back(polygon.as<std::vector<float>>());
        }
        ld.balls.clear();
        if (node["balls"]) {
            ld.balls = node["balls"].as<std::vector<std::pair<Eigen::Vector3f, float>>>();
        }
        return true;
    }
};

template <>
struct convert<sim_env::Box2DJointDescription> {
    static Node encode(const sim_env::Box2DJointDescription& jd)
    {
        Node node;
        node["name"] = jd.name;
        node["link_a"] = jd.link_a;
        node["link_b"] = jd.link_b;
        node["axis"] = jd.axis;
        node["position_limits"] = jd.position_limits;
        node["velocity_limits"] = jd.velocity_limits;
        node["acceleration_limits"] = jd.acceleration_limits;
        node["actuated"] = jd.actuated;
        node["joint_type"] = jd.joint_type;
        node["axis_orientation"] = jd.axis_orientation;
        return node;
    }

    static bool decode(const Node& node, sim_env::Box2DJointDescription& ld)
    {
        // TODO see what the error messages here if elements are missing
        ld.name = node["name"].as<std::string>();
        ld.joint_type = node["joint_type"].as<std::string>();
        ld.axis_orientation = node["axis_orientation"].as<float>();
        ld.link_a = node["link_a"].as<std::string>();
        ld.link_b = node["link_b"].as<std::string>();
        ld.axis = node["axis"].as<Eigen::Vector2f>();
        ld.position_limits = node["position_limits"].as<Eigen::Vector2f>();
        ld.velocity_limits = node["velocity_limits"].as<Eigen::Vector2f>();
        ld.acceleration_limits = node["acceleration_limits"].as<Eigen::Vector2f>();
        ld.actuated = node["actuated"].as<bool>();
        return true;
    }
};

template <>
struct convert<sim_env::Box2DObjectDescription> {
    static Node encode(const sim_env::Box2DObjectDescription& od)
    {
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

    static bool decode(const Node& node, sim_env::Box2DObjectDescription& od)
    {
        od.name = node["name"].as<std::string>();
        // read in links
        const YAML::Node& links_node = node["links"];
        for (YAML::const_iterator iter = links_node.begin(); iter != links_node.end(); iter++) {
            od.links.push_back(iter->as<sim_env::Box2DLinkDescription>());
        }
        // read in joints
        const YAML::Node& joints_node = node["joints"];
        for (YAML::const_iterator iter = joints_node.begin(); iter != joints_node.end(); iter++) {
            od.joints.push_back(iter->as<sim_env::Box2DJointDescription>());
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

    static bool verifyKinematicStructure(const sim_env::Box2DObjectDescription& od, std::string& base_link_name)
    {
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
                    msg_stream << "Previous found root link is \"" << base_link_name << "\"";
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

template <>
struct convert<sim_env::Box2DRobotDescription> {
    static Node encode(const sim_env::Box2DRobotDescription& rd)
    {
        Node node;
        node["object_description"] = rd.object_description;
        Node base_actuation_node;
        base_actuation_node["translational_velocity_limits"] = rd.translational_velocity_limits;
        base_actuation_node["translational_acceleration_limits"] = rd.translational_acceleration_limits;
        base_actuation_node["rotational_velocity_limits"] = rd.rotational_velocity_limits;
        base_actuation_node["rotational_acceleration_limits"] = rd.rotational_acceleration_limits;
        base_actuation_node["use_center_of_mass"] = rd.use_center_of_mass;
        node["base_actuation"] = base_actuation_node;
        return node;
    }

    static bool decode(const Node& node, sim_env::Box2DRobotDescription& rd)
    {
        rd.object_description = node.as<sim_env::Box2DObjectDescription>();
        if (node["base_actuation"]) {
            Node base_actuation_node = node["base_actuation"];
            rd.object_description.is_static = false;
            rd.rotational_acceleration_limits = base_actuation_node["rotational_acceleration_limits"].as<Eigen::Vector2f>();
            rd.rotational_velocity_limits = base_actuation_node["rotational_velocity_limits"].as<Eigen::Vector2f>();
            rd.translational_acceleration_limits = base_actuation_node["translational_acceleration_limits"].as<Eigen::Vector2f>();
            rd.translational_velocity_limits = base_actuation_node["translational_velocity_limits"].as<Eigen::Vector2f>();
            rd.use_center_of_mass = base_actuation_node["use_center_of_mass"].as<bool>();
        } else {
            rd.object_description.is_static = true;
        }
        return true;
    }
};
}
#endif //BOX2D_SIM_ENV_BOX2DIOUTILS_H_H
