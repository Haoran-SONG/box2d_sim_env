//
// Created by joshua on 7/28/17.
//
#include "sim_env/Box2DController.h"
#include "sim_env/utils/EigenUtils.h"
#include <queue>

using namespace sim_env;

Box2DRobotVelocityController::Box2DRobotVelocityController(Box2DRobotPtr robot)
{
    _box2d_robot = robot;
    _b_target_available = false;
}

Box2DRobotVelocityController::~Box2DRobotVelocityController()
{
}

void Box2DRobotVelocityController::setPositionProjectionFn(PositionProjectionFn pos_constraint)
{
    auto robot = _box2d_robot.lock();
    assert(robot);
    auto world = robot->getWorld();
    auto logger = world->getLogger();
    logger->logErr("Position projection not supported for Box2DVelocityController."
                   "Use a position controller to constrain robot positions",
        "[Box2DRobotVelocityController::setPositionProjectionFn]");
}

void Box2DRobotVelocityController::setVelocityProjectionFn(VelocityProjectionFn vel_constraint)
{
    _vel_proj_fn = vel_constraint;
}

unsigned int Box2DRobotVelocityController::getTargetDimension() const
{
    Box2DRobotPtr robot = lockRobot();
    return robot->getNumActiveDOFs();
}

void Box2DRobotVelocityController::setTargetVelocity(const Eigen::VectorXf& velocity)
{

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
    Eigen::VectorXf& output)
{
    //    return controlArm(positions, velocities, timestep, robot, output);
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
    Eigen::VectorXf target_velocity(_target_velocity);
    if (_vel_proj_fn) {
        _vel_proj_fn(target_velocity, robot);
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

bool Box2DRobotVelocityController::controlArm(const Eigen::VectorXf& positions,
    const Eigen::VectorXf& velocities,
    float timestep,
    RobotConstPtr robot,
    Eigen::VectorXf& output)
{
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
    assert((active_dofs.size() == 2 and robot->isStatic()) or (active_dofs.size() == 5 and not robot->isStatic()));
    Eigen::Matrix2f inertia_matrix;
    Eigen::Matrix2f coriolis_matrix;
    computeDynamics(active_dofs, positions, velocities, box2d_robot, inertia_matrix, coriolis_matrix);
    Eigen::Vector2f joint_pos(positions[robot->getNumBaseDOFs()], positions[robot->getNumBaseDOFs() + 1]);
    Eigen::Vector2f joint_vels(velocities[robot->getNumBaseDOFs()], velocities[robot->getNumBaseDOFs() + 1]);
    for (unsigned int i = 0; i < robot->getNumBaseDOFs(); ++i) {
        float accel = (_target_velocity[i] - velocities[i]) / timestep;
        float mass_inertia = i == 2 ? box2d_robot->getInertia() : box2d_robot->getMass();
        // we need to limit the acceleration though
        accel = std::min(acceleration_limits(i, 1), std::max(accel, acceleration_limits(i, 0)));
        output[i] = mass_inertia * accel; // make it a force / torque
    }
    Eigen::Vector2f joint_accels(_target_velocity[robot->getNumBaseDOFs()] - velocities[robot->getNumBaseDOFs()],
        _target_velocity[robot->getNumBaseDOFs() + 1] - velocities[robot->getNumBaseDOFs() + 1]);
    joint_accels = 1.0f / timestep * joint_accels;
    joint_accels[0] = std::min(acceleration_limits(robot->getNumBaseDOFs(), 1), std::max(joint_accels[0], acceleration_limits(robot->getNumBaseDOFs(), 0)));
    joint_accels[1] = std::min(acceleration_limits(robot->getNumBaseDOFs() + 1, 1), std::max(joint_accels[1], acceleration_limits(robot->getNumBaseDOFs() + 1, 0)));
    logger->logDebug(boost::format("Accelerations: %1%") % joint_accels, prefix);
    logger->logDebug(boost::format("Inertia matrix: %1%") % inertia_matrix, prefix);
    Eigen::Vector2f joint_torques = inertia_matrix * joint_accels + coriolis_matrix * joint_vels;
    logger->logDebug(boost::format("joint_torques: %1%") % joint_torques, prefix);
    output[robot->getNumBaseDOFs()] = joint_torques[0];
    output[robot->getNumBaseDOFs() + 1] = joint_torques[1];
    return true;
}

Box2DRobotPtr Box2DRobotVelocityController::getBox2DRobot() const
{
    return lockRobot();
}

RobotPtr Box2DRobotVelocityController::getRobot() const
{
    return lockRobot();
}

RobotVelocityControllerPtr Box2DRobotVelocityController::clone(WorldPtr world){
    auto box2d_world = std::dynamic_pointer_cast<Box2DWorld>(world);
    if (!box2d_world) {
        throw std::logic_error("Could not clone velocity controller. The provided world is not a Box2DWorld.");
    }
    auto my_robot = getBox2DRobot();
    assert(my_robot);
    auto new_robot = box2d_world->getBox2DRobot(my_robot->getName());
    if (!new_robot) {
        throw std::logic_error("Could not clone velocity controller. The provided world does not contain my robot");
    }
    auto new_controller = std::make_shared<Box2DRobotVelocityController>(new_robot);
    using namespace std::placeholders;
    sim_env::Robot::ControlCallback callback = std::bind(&sim_env::Box2DRobotVelocityController::control,
                                                                    new_controller,
                                                                    _1, _2, _3, _4, _5);
    new_robot->setController(callback);
    return new_controller;
}

Box2DRobotPtr Box2DRobotVelocityController::lockRobot() const {
    Box2DRobotPtr robot = _box2d_robot.lock();
    if (!robot) {
        LoggerPtr logger = DefaultLogger::getInstance();
        logger->logErr("Could not access Box2D robot. Instance does not exist anymore.",
            "[sim_env::Box2DRobotVelocityController::lockRobot]");
        throw std::logic_error("[sim_env::Box2DRobotVelocityController::lockRobot] Weak pointer to Box2DRobot not valid anymore.");
    }
    return robot;
}

float Box2DRobotVelocityController::computeKinematicChainInertia(JointConstPtr root_joint) const
{
    Box2DJointConstPtr box2d_root_joint = std::dynamic_pointer_cast<const Box2DJoint>(root_joint);
    if (not box2d_root_joint) {
        throw std::logic_error("[sim_env::Box2DRobotVelocityController::computeKinematicChainInertia] Could not cast joint"
                               " to Box2DJoint. This controller only works with Box2DRobots!");
    }
    Eigen::Vector2f root_axis = box2d_root_joint->getAxisPosition();
    float inertia = 0.0f;
    std::queue<JointConstPtr> joint_queue;
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

void Box2DRobotVelocityController::computeDynamics(const Eigen::VectorXi& active_dofs,
    const Eigen::VectorXf& positions,
    const Eigen::VectorXf& velocities,
    Box2DRobotConstPtr robot,
    Eigen::Matrix2f& inertia_matrix,
    Eigen::Matrix2f& coriolis_matrix) const
{
    Eigen::Array2f l_distances;
    Eigen::Array2f l_masses;
    Eigen::Array2f l_inertias;
    Eigen::Array2f m_inertias;
    Eigen::Array2f joint_distances;
    std::vector<Box2DJointConstPtr> joints;
    robot->getBox2DJoints(joints);
    Eigen::Vector2f prev_joint_axis(0.0f, 0.0f);
    for (unsigned int i = 0; i < active_dofs.size(); ++i) {
        unsigned int dof_idx = active_dofs[i];
        if (dof_idx < robot->getNumBaseDOFs()) {
            continue;
        }
        unsigned int joint_idx = dof_idx - robot->getNumBaseDOFs();
        assert(joint_idx < 2); // for now only two dofs allowed
        auto& joint = joints.at(joint_idx);
        auto parent_link = joint->getParentBox2DLink();
        auto child_link = joint->getChildBox2DLink();
        auto joint_axis = joint->getAxisPosition();
        l_masses[joint_idx] = child_link->getMass();
        l_distances[joint_idx] = (child_link->getCenterOfMass() - joint_axis).norm();
        l_inertias[joint_idx] = child_link->getInertia();
        joint_distances[joint_idx] = (prev_joint_axis - joint_axis).norm();
        m_inertias[joint_idx] = l_inertias[joint_idx] + l_distances[joint_idx] * l_distances[joint_idx] * l_masses[joint_idx];
        //        m_inertias[joint_idx] = 0.0f;
        prev_joint_axis = joint_axis;
    }
    Eigen::Array2f cosines;
    Eigen::Array2f sines;
    for (unsigned int i = 0; i < 2; ++i) {
        cosines[i] = std::cos(positions[i + robot->getNumBaseDOFs()]);
        sines[i] = std::sin(positions[i + robot->getNumBaseDOFs()]);
    }
    // set inertia matrix
    inertia_matrix(0, 0) = l_inertias[0] + l_masses[0] * l_distances[0] * l_distances[0] + m_inertias[0]
        + l_inertias[1]
        + l_masses[1] * (joint_distances[1] * joint_distances[1] + l_distances[1] * l_distances[1] + 2.0f * joint_distances[1] * l_distances[1] * cosines[1])
        + m_inertias[1];
    inertia_matrix(0, 1) = l_inertias[1] + l_masses[1] * (l_distances[1] * l_distances[1] + joint_distances[1] * l_distances[1] * cosines[1]);
    inertia_matrix(1, 0) = inertia_matrix(0, 1);
    inertia_matrix(1, 1) = l_inertias[1] + l_masses[1] * l_distances[1] * l_distances[1];
    // set coriolis matrix
    float h = -l_masses[1] * joint_distances[1] * l_distances[1] * sines[1];
    coriolis_matrix(0, 0) = h * velocities[robot->getNumBaseDOFs() + 1];
    coriolis_matrix(0, 1) = h * (velocities[robot->getNumBaseDOFs()] + velocities[robot->getNumBaseDOFs() + 1]);
    coriolis_matrix(1, 0) = -h * velocities[robot->getNumBaseDOFs()];
    coriolis_matrix(1, 1) = 0;
}

LoggerPtr Box2DRobotVelocityController::getLogger() const
{
    Box2DRobotPtr robot = _box2d_robot.lock();
    if (not robot) {
        return DefaultLogger::getInstance();
    }
    return robot->getWorld()->getLogger();
}
