//
// Created by joshua on 6/21/17.
//

#include"sim_env/Box2DWorld.h"

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

int main(int argc, char **argv) {
    sim_env::Box2DWorldPtr world = std::make_shared<sim_env::Box2DWorld>();
    world->loadWorld("/home/joshua/projects/planning_catkin/src/box2d_sim_env/test_data/test_env.yaml");
    sim_env::ObjectPtr object = world->getObject("obj1");
    sim_env::WorldPtr world2 = object->getWorld();
}

