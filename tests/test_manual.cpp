//
// Created by joshua on 6/21/17.
//

#include "sim_env/Box2DWorld.h"
#include <QApplication>
#include <QPushButton>
#include <sim_env/Box2DWorldViewer.h>

//void testLogger() {
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

int main(int argc, char** argv)
{
    sim_env::Box2DWorldPtr world = std::make_shared<sim_env::Box2DWorld>();
    world->getLogger()->setLevel(sim_env::Logger::LogLevel::Debug);
    world->loadWorld("/home/joshua/projects/planning_catkin/src/planner_tests/data/box2d/worlds/door_world_2m_3s.yaml");
    std::vector<sim_env::RobotPtr> robots;
    world->getRobots(robots);
    sim_env::RobotPtr robot = robots[0];
    Eigen::VectorXf new_vel = robot->getDOFVelocities();
    std::stringstream ss;
    ss << new_vel.transpose();
    sim_env::Box2DWorldViewerPtr world_viewer = std::make_shared<sim_env::Box2DWorldViewer>(world);
    world_viewer->init(argc, argv);
    world_viewer->show();
    auto robot_transform = robot->getTransform();
    world_viewer->drawFrame(robot_transform);
    return world_viewer->run();
}
