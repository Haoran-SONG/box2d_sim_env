//
// Created by joshua on 7/28/17.
//

#ifndef BOX2D_SIM_ENV_BOX2DCONTROLLER_H
#define BOX2D_SIM_ENV_BOX2DCONTROLLER_H

#include <sim_env/Box2DWorld.h>
#include <sim_env/Controller.h>

namespace sim_env {
    /**
     * Box2DRobot velocity controller.
     * A specialized robot controller for robots in Box2D.
     */
    class Box2DRobotVelocityController : public RobotVelocityController {
    public:
        Box2DRobotVelocityController(Box2DRobotPtr robot);
        ~Box2DRobotVelocityController();
        unsigned int getTargetDimension() override;
        void setTargetVelocity(const Eigen::VectorXf& velocity) override;
        bool control(const Eigen::VectorXf& positions,
                     const Eigen::VectorXf& velocities,
                     float timestep,
                     RobotConstPtr robot,
                     Eigen::VectorXf& output) override;
        bool controlArm(const Eigen::VectorXf& positions,
                    const Eigen::VectorXf& velocities,
                    float timestep,
                    RobotConstPtr robot,
                    Eigen::VectorXf& output);
        Box2DRobotPtr getBox2DRobot() const;
        RobotPtr getRobot() const override;
        RobotVelocityControllerPtr clone(WorldPtr world) override;
    protected:
        Box2DRobotPtr lockRobot() const;
        float computeKinematicChainInertia(JointConstPtr joint) const;
        void computeDynamics(const Eigen::VectorXi& active_dofs,
                             const Eigen::VectorXf& positions,
                             const Eigen::VectorXf& velocities,
                             Box2DRobotConstPtr robot,
                             Eigen::Matrix2f& inertia_matrix,
                             Eigen::Matrix2f& coriolis_matrix) const;
        LoggerPtr getLogger() const;
    private:
        Box2DRobotWeakPtr _box2d_robot;
        Eigen::VectorXf _target_velocity;
        bool _b_target_available;
    };

    typedef std::shared_ptr<Box2DRobotVelocityController> Box2DRobotVelocityControllerPtr;
    typedef std::shared_ptr<const Box2DRobotVelocityController> Box2DRobotVelocityControllerConstPtr;
    typedef std::weak_ptr<Box2DRobotVelocityController> Box2DRobotVelocityControllerWeakPtr;
    typedef std::weak_ptr<const Box2DRobotVelocityController> Box2DRobotVelocityControllerWeakConstPtr;
}
#endif //BOX2D_SIM_ENV_BOX2DCONTROLLER_H
