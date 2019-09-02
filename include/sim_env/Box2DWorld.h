//
// Created by joshua on 4/20/17.
//
#ifndef BOX2D_SIM_ENV_BOX2DWORLD_H
#define BOX2D_SIM_ENV_BOX2DWORLD_H

#include "Box2D/Box2D.h"
#include "Box2DIOUtils.h"
#include "sim_env/Controller.h"
#include "sim_env/SimEnv.h"
#include <boost/filesystem/path.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <mutex>
#include <stack>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

namespace sim_env {
class Box2DObject;
typedef std::shared_ptr<Box2DObject> Box2DObjectPtr;
typedef std::weak_ptr<Box2DObject> Box2DObjectWeakPtr;
typedef std::shared_ptr<const Box2DObject> Box2DObjectConstPtr;
typedef std::weak_ptr<const Box2DObject> Box2DObjectConstWeakPtr;

class Box2DWorld;
typedef std::shared_ptr<Box2DWorld> Box2DWorldPtr;
typedef std::weak_ptr<Box2DWorld> Box2DWorldWeakPtr;
typedef std::shared_ptr<const Box2DWorld> Box2DWorldConstPtr;
typedef std::weak_ptr<const Box2DWorld> Box2DWorldConstWeakPtr;

class Box2DRobot;
typedef std::shared_ptr<Box2DRobot> Box2DRobotPtr;
typedef std::weak_ptr<Box2DRobot> Box2DRobotWeakPtr;
typedef std::shared_ptr<const Box2DRobot> Box2DRobotConstPtr;
typedef std::weak_ptr<const Box2DRobot> Box2DRobotConstWeakPtr;

class Box2DLink;
typedef std::shared_ptr<Box2DLink> Box2DLinkPtr;
typedef std::weak_ptr<Box2DLink> Box2DLinkWeakPtr;
typedef std::shared_ptr<const Box2DLink> Box2DLinkConstPtr;
typedef std::weak_ptr<const Box2DLink> Box2DLinkConstWeakPtr;

class Box2DJoint;
typedef std::shared_ptr<Box2DJoint> Box2DJointPtr;
typedef std::weak_ptr<Box2DJoint> Box2DJointWeakPtr;
typedef std::shared_ptr<const Box2DJoint> Box2DJointConstPtr;
typedef std::weak_ptr<const Box2DJoint> Box2DJointConstWeakPtr;

class Box2DCollidable;
typedef std::shared_ptr<Box2DCollidable> Box2DCollidablePtr;
typedef std::shared_ptr<const Box2DCollidable> Box2DCollidableConstPtr;

class Box2DWorldViewer;
typedef std::shared_ptr<Box2DWorldViewer> Box2DWorldViewerPtr;
typedef std::shared_ptr<const Box2DWorldViewer> Box2DWorldViewerConstPtr;
typedef std::weak_ptr<Box2DWorldViewer> Box2DWorldViewerWeakPtr;
typedef std::weak_ptr<const Box2DWorldViewer> Box2DWorldViewerConstWeakPtr;

class Box2DImageRenderer;
typedef std::shared_ptr<Box2DImageRenderer> Box2DImageRendererPtr;
typedef std::shared_ptr<const Box2DImageRenderer> Box2DImageRendererConstPtr;
typedef std::weak_ptr<Box2DImageRenderer> Box2DImageRendererWeakPtr;
typedef std::weak_ptr<const Box2DImageRenderer> Box2DImageRendererConstWeakPtr;

class Box2DCollisionChecker;
typedef std::shared_ptr<Box2DCollisionChecker> Box2DCollisionCheckerPtr;
typedef std::shared_ptr<const Box2DCollisionChecker> Box2DCollisionCheckerConstPtr;

namespace viewer {
    class Box2DDrawingInterface;
    typedef std::shared_ptr<Box2DDrawingInterface> Box2DDrawingInterfacePtr;
    typedef std::shared_ptr<const Box2DDrawingInterface> Box2DDrawingInterfaceConstPtr;
}

class Box2DCollidable {
    friend class Box2DCollisionChecker;

protected:
    /**
         * Returns all b2Bodies that this collidable contains.
         * @param bodies list of all b2Bodies
         */
    virtual void getBodies(std::vector<b2Body*>& bodies) = 0;

    /**
         * Returns the link associated with the given body.
         * If this collidable is a link, it returns itself.
         * If this collidable is an object, it returns the link with this body.
         * If this collidable is not associated with the given body, null_ptr is returned.
         * @param body - body to get the link for
         * @return - shared pointer to the link, or null_ptr
         */
    virtual Box2DLinkPtr getBox2DLink(b2Body* body) = 0;
};

/**
     * Box2DLink - The box2d implementation of a robot/object link.
     */
class Box2DLink : public Link, public Box2DCollidable, public std::enable_shared_from_this<Box2DLink> {
    friend class Box2DWorld;
    friend class Box2DJoint;
    friend class Box2DObject;
    friend class Box2DRobot;

public:
    struct Box2DBodyUserData {
        std::string link_name;
        std::string object_name;
        bool b_collision_ignore;
        Box2DBodyUserData(const std::string& lname, const std::string& oname, bool collision_ignore = false);
        ~Box2DBodyUserData();
    };

    /**
         * A link can only be instantiated by a Box2DObject.
         */
    Box2DLink() = delete;
    Box2DLink(const Box2DLink& link) = delete;
    Box2DLink& operator=(const Box2DLink&) = delete;
    Box2DLink& operator=(Box2DLink&&) = delete;
    ~Box2DLink() override;

    bool checkCollision() override;
    bool checkCollision(std::vector<Contact>& contacts) override;
    bool checkCollision(const std::vector<LinkPtr>& other_links) override;
    bool checkCollision(const std::vector<LinkPtr>& other_links, std::vector<Contact>& contacts) override;
    bool checkCollision(const std::vector<ObjectPtr>& other_objects) override;
    bool checkCollision(const std::vector<ObjectPtr>& other_objects, std::vector<Contact>& contacts) override;

    void getBallApproximation(std::vector<Ball>& balls) const override;
    unsigned int getNumApproximationBalls() const;
    void getLocalBallApproximation(std::vector<Ball>& balls) const override;
    void updateBallApproximation(std::vector<Ball>& balls,
        std::vector<Ball>::iterator& start,
        std::vector<Ball>::iterator& end) const override;
    std::string getName() const override;
    EntityType getType() const override;

    Eigen::Affine3f getTransform() const override;
    void getPose(Eigen::Vector3f& pose) const;
    Eigen::Vector3f getPose() const;

    void getVelocityVector(Eigen::Vector3f& vel_vector) const;
    Eigen::Vector3f getVelocityVector() const;

    WorldPtr getWorld() const override;
    ObjectPtr getObject() const override;

    WorldConstPtr getConstWorld() const override;
    ObjectConstPtr getConstObject() const override;

    void getChildJoints(std::vector<JointPtr>& child_joints) override;
    void getConstChildJoints(std::vector<JointConstPtr>& child_joints) const override;
    JointPtr getParentJoint() override;
    JointConstPtr getConstParentJoint() const override;

    /**
      * Return the geometry of this link.
      */
    std::vector<Geometry> getGeometries() const override;
    void getGeometries(std::vector<Geometry>& geoms) const override;

    std::vector<Geometry> getFixtureGeometries() const;
    void getFixtureGeometries(std::vector<Geometry>& geoms) const;
    // void getGeometry(std::vector<std::vector<Eigen::Vector2f>>& geometry) const;
    void getBoostGeometry(std::vector<boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<float>, false>>& polygons) const;
    void getWorldBoostGeometry(std::vector<boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<float>, false>>& polygons) const;
    const std::vector<boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<float>, false>>& getBoostGeometry() const;
    float getMass() const override;
    float getGroundFrictionCoefficient() const override;
    void setGroundFrictionCoefficient(float mu) override;
    float getGroundFrictionLimitTorque() const override;
    float getGroundFrictionLimitForce() const override;
    void setGroundFrictionTorqueIntegral(float val) override;
    float getGroundFrictionTorqueIntegral() const override;
    float getContactFriction() const override;
    void setContactFriction(float coeff) override;
    void setMass(float mass) override;
    void getCenterOfMass(Eigen::Vector3f& com) const override;
    void getLocalCenterOfMass(Eigen::Vector3f& com) const override;
    // Box2D specific
    Eigen::Vector2f getCenterOfMass() const;
    void getCenterOfMass(Eigen::Vector2f& com) const;
    float getInertia() const;
    // return the inertia w.r.t. to the center of mass
    float getCOMInertia() const;
    Box2DWorldPtr getBox2DWorld() const;
    BoundingBox getLocalBoundingBox() const;
    void setEnabled(bool b_enable) override;
    bool isEnabled() const override;

    //        Box2DObjectPtr getBox2DObject() const;

protected:
    // the constructor is protected to ensure that links can only be created within objects.
    Box2DLink(const Box2DLinkDescription& link_desc, Box2DWorldPtr world, bool is_static,
        const std::string& object_name);
    // for cleanup, we need destroy functions. these are called when the Box2DWorld is destroyed.
    void destroy(const std::shared_ptr<b2World>& b2world); // TODO do we need to make this thread-safe?
    // set the name of this link
    void setName(const std::string& name) override;
    void setObjectName(const std::string& name);
    // returns the underlying box2d body
    b2Body* getBody();
    // registers a child joint
    void registerChildJoint(Box2DJointPtr joint);
    // registers a parent joint
    void registerParentJoint(Box2DJointPtr joint);
    // sets the transform of this link. Should not be called externally. Calls setPose(pose, true, false)
    //        void setTransform(const Eigen::Affine3f& tf);
    /*
         * Sets the pose of this link. If update_children is true, all child links are moved together with this link.
         * if joint_override is true, the relative transforms between this link and its children is set
         * to the initial transform defined by the joint connecting both links (i.e. it is set to joint position 0)
         */
    void setPose(const Eigen::Vector3f& pose, bool update_children = true, bool joint_override = false);
    // sets the velocity of the underlying box2d body. velocities need to be scaled to box2d scale
    void setVelocityVector(const Eigen::Vector3f& velocity);
    // resets the origin of this link to its center of mass (ONLY FOR BASE LINKS)
    void setOriginToCenterOfMass();
    // Adds the body of this link to the given vector - used for collision checks
    void getBodies(std::vector<b2Body*>& bodies) override;
    // returns a pointer to this link
    Box2DLinkPtr getBox2DLink(b2Body* body) override;

private:
    Box2DWorldWeakPtr _world;
    b2Body* _body;
    b2Vec2 _local_origin_offset; // optional offset from the body's frame of reference to link frame of reference
    b2Joint* _friction_joint;
    std::string _name;
    std::string _object_name;
    std::vector<Box2DJointWeakPtr> _child_joints;
    std::vector<Ball> _balls;
    Box2DJointWeakPtr _parent_joint;
    BoundingBox _local_aabb;
    std::vector<Geometry> _link_geometries;
    std::vector<boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<float>, false>> _boost_polygons;
    float _ground_torque_integral;
    float _ground_friction;
    float _contact_friction; // friction with other objects
    bool _destroyed;
    bool _enabled;

    void updateFrictionJoint();
    /*
         * Recursively propagates a new absolute velocity of a parent frame through a kinematic chain.
         * @param lin_vel new linear velocity of the parent in global frame (scaled to box2d world)
         * @param dw angular new angular velocity  of parent
         * @param joint_vel joint_velocity of the joint connecting the parent with this link
         * @param parent pointer to the parent body
         */
    //        void propagateVelocityChange(float v_x, float v_y, float v_theta, float v_joint, b2Body* parent);
    void updateBodyVelocities(const Eigen::VectorXf& all_dof_velocities,
        std::vector<std::pair<b2Vec2, unsigned int>>& parent_joints,
        unsigned int index_offset);
};

/**
     * Box2DJoint - This class implements robot/object joints in box2d.
     */
class Box2DJoint : public Joint {
    friend class Box2DWorld;
    friend class Box2DObject;
    friend class Box2DLink;
    friend class Box2DRobot;

public:
    /**
         * A Box2DJoint can only be instantiated by an object/robot.
         */
    Box2DJoint() = delete;
    Box2DJoint(const Box2DJoint& joint) = delete;
    Box2DJoint& operator=(const Box2DJoint& joint) = delete;
    Box2DJoint& operator=(Box2DJoint&&) = delete;
    ~Box2DJoint() override;
    float getPosition() const override;
    void setPosition(float v) override;
    float getVelocity() const override;
    void setVelocity(float v) override;
    Eigen::Array2f getPositionLimits() const override;
    void getPositionLimits(Eigen::Array2f& limits) const override;
    Eigen::Array2f getVelocityLimits() const override;
    void getVelocityLimits(Eigen::Array2f& limits) const override;
    Eigen::Array2f getAccelerationLimits() const override;
    void getAccelerationLimits(Eigen::Array2f& limits) const override;
    unsigned int getJointIndex() const override;
    unsigned int getDOFIndex() const override;
    JointType getJointType() const override;
    std::string getName() const override;
    EntityType getType() const override;
    Eigen::Affine3f getTransform() const override;
    WorldPtr getWorld() const override;
    ObjectPtr getObject() const override;
    LinkPtr getChildLink() const override;
    LinkPtr getParentLink() const override;
    WorldConstPtr getConstWorld() const override;
    ObjectConstPtr getConstObject() const override;
    void getDOFInformation(DOFInformation& info) const override;
    DOFInformation getDOFInformation() const override;
    //        Box2DObjectPtr getBox2DObject() const;
    // Box2D specific interface
    Box2DLinkPtr getChildBox2DLink() const;
    Box2DLinkPtr getParentBox2DLink() const;
    Box2DWorldPtr getBox2DWorld() const;
    Eigen::Vector2f getAxisPosition() const;

protected:
    // ensure we can only create this from friend classes
    Box2DJoint(const Box2DJointDescription& joint_desc,
        Box2DLinkPtr link_a,
        Box2DLinkPtr link_b,
        Box2DWorldPtr world,
        const std::string& object_name);
    // called when Box2DWorld is destroyed
    void destroy(const std::shared_ptr<b2World>& b2world);
    void setName(const std::string& name) override;
    void setObjectName(const std::string& name);
    void setJointIndex(unsigned int index);
    void setDOFIndex(unsigned int index);
    void resetPosition(float value, bool child_joint_override);
    void setControlTorque(float value);
    // retuns axis position in parent frame
    b2Vec2 getLocalAxisPosition() const;
    // returns axis position in global frame
    b2Vec2 getGlobalAxisPosition() const;
    // returns the box2d object this joint belongs to
    Box2DObjectPtr getBox2DObject();

private:
    Box2DWorldWeakPtr _world;
    std::string _name;
    Box2DLinkWeakPtr _link_a;
    Box2DLinkWeakPtr _link_b;
    b2Joint* _joint;
    JointType _joint_type;
    bool _destroyed;
    unsigned int _joint_index;
    unsigned int _dof_index;
    Eigen::Array2f _position_limits; // not a multiple of 16 bytes
    Eigen::Array2f _velocity_limits;
    Eigen::Array2f _acceleration_limits;
    std::string _object_name;
};

/**
     * Box2DObject - This class implements an Object in box2d.
     */
class Box2DObject : public Object, public Box2DCollidable, public std::enable_shared_from_this<Box2DObject> {
    friend class Box2DWorld;
    friend class Box2DRobot;

public:
    /**
         * A Box2DObject can only be constructed by a Box2DWorld.
         */
    Box2DObject() = delete;
    //TODO what about copy constructor?
    ~Box2DObject() override;

    std::string getName() const override;
    EntityType getType() const override;
    Eigen::Affine3f getTransform() const override;
    WorldPtr getWorld() const override;
    WorldConstPtr getConstWorld() const override;
    Box2DWorldPtr getBox2DWorld() const;
    Eigen::Vector3f getPose() const;
    void getPose(Eigen::Vector3f& pose) const;
    void setTransform(const Eigen::Affine3f& tf) override;

    bool checkCollision() override;
    bool checkCollision(std::vector<Contact>& contacts) override;
    bool checkCollision(ObjectPtr other_object) override;
    bool checkCollision(ObjectPtr other_object, std::vector<Contact>& contacts) override;
    bool checkCollision(const std::vector<ObjectPtr>& other_objects) override;
    bool checkCollision(const std::vector<ObjectPtr>& other_objects, std::vector<Contact>& contacts) override;

    void setActiveDOFs(const Eigen::VectorXi& indices) override;
    Eigen::VectorXi getActiveDOFs() const override;
    unsigned int getNumActiveDOFs() const override;
    unsigned int getNumDOFs() const override;
    unsigned int getNumBaseDOFs() const override;
    Eigen::VectorXi getDOFIndices() const override;
    Eigen::VectorXf getDOFPositions(const Eigen::VectorXi& indices = Eigen::VectorXi()) const override;
    Eigen::ArrayX2f getDOFPositionLimits(const Eigen::VectorXi& indices = Eigen::VectorXi()) const override;

    DOFInformation getDOFInformation(unsigned int dof_index) const override;
    void getDOFInformation(unsigned int dof_index, DOFInformation& info) const override;
    void setDOFPositions(const Eigen::VectorXf& values, const Eigen::VectorXi& indices = Eigen::VectorXi()) override;

    Eigen::VectorXf getDOFVelocities(const Eigen::VectorXi& indices = Eigen::VectorXi()) const override;
    Eigen::ArrayX2f getDOFVelocityLimits(const Eigen::VectorXi& indices = Eigen::VectorXi()) const override;
    Eigen::ArrayX2f getDOFAccelerationLimits(const Eigen::VectorXi& indices = Eigen::VectorXi()) const override;

    void setDOFVelocities(const Eigen::VectorXf& values, const Eigen::VectorXi& indices = Eigen::VectorXi()) override;
    void setToRest();
    bool atRest(float threshold = 0.0001f) const override;

    bool isStatic() const override;
    // ball approximation
    void getBallApproximation(std::vector<Ball>& balls) const override;
    void getLinks(std::vector<LinkPtr>& links) override;
    void getLinks(std::vector<LinkConstPtr>& links) const override;
    LinkPtr getLink(const std::string& link_name) override;
    LinkConstPtr getConstLink(const std::string& link_name) const override;
    LinkPtr getBaseLink() override;
    LinkConstPtr getConstBaseLink() const override;

    void getJoints(std::vector<JointPtr>& joints) override;
    void getBox2DJoints(std::vector<Box2DJointPtr>& joints);
    void getJoints(std::vector<JointConstPtr>& joints) const override;
    void getBox2DJoints(std::vector<Box2DJointConstPtr>& joints) const;
    JointPtr getJoint(const std::string& joint_name) override;
    JointPtr getJoint(unsigned int joint_idx) override;
    JointPtr getJointFromDOFIndex(unsigned int dof_idx) override;
    JointConstPtr getConstJoint(const std::string& joint_name) const override;
    JointConstPtr getConstJoint(unsigned int joint_idx) const override;
    JointConstPtr getConstJointFromDOFIndex(unsigned int dof_idx) const override;

    void getState(ObjectState& object_state) const override;
    ObjectState getState() const override;
    void setState(const ObjectState& object_state) override;
    void setEnabled(bool b_enable) override;
    bool isEnabled() const override;

    // Box2D specific methods
    /**
         * Returns the total mass of this object.
         * (i.e. the sum of the mass of all its bodies)
         * @return mass
         */
    float getMass() const override;
    /**
         * Returns the moment of inertia of this object (given the current configuration).
         * TODO specify frame
         * @return moment of inertia
         */
    float getInertia() const override;
    BoundingBox getLocalAABB() const override;
    // float getGroundFriction() const override;
    virtual Box2DLinkPtr getBox2DBaseLink();
    void getBox2DLinks(std::vector<Box2DLinkPtr>& links);
    void setPose(float x, float y, float theta);
    void setPose(const Eigen::Vector3f& pose);

protected:
    // ensure only friend classes can construct this
    Box2DObject(const Box2DObjectDescription& obj_desc, Box2DWorldPtr world);
    // function for destruction. called by Box2DWorld when it is destroyed.
    void destroy(const std::shared_ptr<b2World>& b2world);
    void setName(const std::string& name) override;
    Box2DJointPtr getBox2DJoint(unsigned int idx);
    // puts all link bodies into the given list (for collision checking)
    void getBodies(std::vector<b2Body*>& bodies) override;
    Box2DLinkPtr getBox2DLink(b2Body* body) override;
    void updateBodyVelocities(const Eigen::VectorXf& all_dof_velocities);

    // these can be overwritten by Box2DRobot
    DOFInformation _x_dof_info;
    DOFInformation _y_dof_info;
    DOFInformation _theta_dof_info;

private:
    std::string _name;
    Box2DWorldWeakPtr _world;
    float _mass;
    BoundingBox _local_bounding_box;
    Eigen::VectorXi _active_dof_indices;
    Eigen::VectorXi _all_dof_indices;
    unsigned int _num_dofs;
    unsigned int _num_balls;
    std::map<std::string, Box2DLinkPtr> _links; // the object is responsible for the lifetime of its links
    Box2DLinkPtr _base_link;
    std::map<std::string, Box2DJointPtr> _joints; // the object is responsible for the lifetime of its links
    std::vector<Box2DJointPtr> _sorted_joints; // sorted list of objects
    bool _is_static;
    bool _destroyed;
};

/**
     * Box2DRobot - This class implements a Robot in box2d.
     * See sim_env::Object and sim_env::Robot for documentation.
     */
class Box2DRobot : public Robot, public Box2DCollidable, public std::enable_shared_from_this<Box2DRobot> {
    friend class Box2DWorld;
    friend class Box2DJoint;

public:
    /**
         * A Box2DRobot can only be instantiated by a Box2DWorld.
         */
    Box2DRobot() = delete;
    ~Box2DRobot() override;
    // entity functions
    virtual std::string getName() const override;
    void setTransform(const Eigen::Affine3f& tf) override;
    EntityType getType() const override;
    Eigen::Affine3f getTransform() const override;
    WorldPtr getWorld() const override;
    WorldConstPtr getConstWorld() const override;
    // DOFs
    void setActiveDOFs(const Eigen::VectorXi& indices) override;
    unsigned int getNumDOFs() const override;
    unsigned int getNumActiveDOFs() const override;
    unsigned int getNumBaseDOFs() const override;
    DOFInformation getDOFInformation(unsigned int dof_index) const override;
    void getDOFInformation(unsigned int dof_index, DOFInformation& info) const override;
    Eigen::VectorXi getActiveDOFs() const override;
    Eigen::VectorXi getDOFIndices() const override;
    Eigen::VectorXf getDOFPositions(const Eigen::VectorXi& indices = Eigen::VectorXi()) const override;
    Eigen::ArrayX2f getDOFPositionLimits(const Eigen::VectorXi& indices = Eigen::VectorXi()) const override;
    void setDOFPositions(const Eigen::VectorXf& values, const Eigen::VectorXi& indices = Eigen::VectorXi()) override;
    Eigen::VectorXf getDOFVelocities(const Eigen::VectorXi& indices = Eigen::VectorXi()) const override;
    Eigen::ArrayX2f getDOFVelocityLimits(const Eigen::VectorXi& indices = Eigen::VectorXi()) const override;
    void setToRest();
    void setDOFVelocities(const Eigen::VectorXf& values, const Eigen::VectorXi& indices = Eigen::VectorXi()) override;
    Eigen::ArrayX2f getDOFAccelerationLimits(const Eigen::VectorXi& indices = Eigen::VectorXi()) const override;
    bool isStatic() const override;
    bool atRest(float threshold = 0.0001f) const override;
    // collision checking
    bool checkCollision() override;
    bool checkCollision(std::vector<Contact>& contacts) override;
    bool checkCollision(ObjectPtr other_object) override;
    bool checkCollision(ObjectPtr other_object, std::vector<Contact>& contacts) override;
    bool checkCollision(const std::vector<ObjectPtr>& other_objects) override;
    bool checkCollision(const std::vector<ObjectPtr>& other_objects, std::vector<Contact>& contacts) override;
    // ball approximation
    void getBallApproximation(std::vector<Ball>& balls) const override;
    // links
    void getLinks(std::vector<LinkPtr>& links) override;
    void getLinks(std::vector<LinkConstPtr>& links) const override;
    void getBox2DLinks(std::vector<Box2DLinkPtr>& links);
    LinkPtr getLink(const std::string& link_name) override;
    LinkConstPtr getConstLink(const std::string& link_name) const override;
    LinkPtr getBaseLink() override;
    LinkConstPtr getConstBaseLink() const override;
    Box2DLinkPtr getBox2DBaseLink();
    // joints
    void getJoints(std::vector<JointPtr>& joints) override;
    void getBox2DJoints(std::vector<Box2DJointPtr>& joints);
    void getJoints(std::vector<JointConstPtr>& joints) const override;
    void getBox2DJoints(std::vector<Box2DJointConstPtr>& joints) const;
    JointPtr getJoint(const std::string& joint_name) override;
    JointConstPtr getConstJoint(const std::string& joint_name) const override;
    JointPtr getJoint(unsigned int joint_idx) override;
    JointPtr getJointFromDOFIndex(unsigned int dof_idx) override;
    JointConstPtr getConstJoint(unsigned int joint_idx) const override;
    JointConstPtr getConstJointFromDOFIndex(unsigned int dof_idx) const override;
    // control
    void setController(ControlCallback control_fn) override;
    float getMass() const override;
    float getInertia() const override;
    BoundingBox getLocalAABB() const override;

    // state retrieval
    void getState(ObjectState& object_state) const override;
    ObjectState getState() const override;
    void setState(const ObjectState& object_state) override;
    Eigen::Vector3f getPose() const;
    void getPose(Eigen::Vector3f& pose) const;
    void setPose(const Eigen::Vector3f& pose);
    void setPose(float x, float y, float theta);

    void setEnabled(bool b_enable) override;
    bool isEnabled() const override;
    // returns the underlying box2d object
    Box2DObjectPtr getBox2DObject();

protected:
    // protected constructor to ensure construction is only done by friend classes
    Box2DRobot(const Box2DRobotDescription& robot_desc, Box2DWorldPtr world);
    // destruction is issued by Box2DWorld
    void destroy(const std::shared_ptr<b2World>& b2world);
    void setName(const std::string& name) override;
    // calls control callbacks. should be called only by step function of Box2DWorld
    void control(float timestep);
    // retrieve all box2d bodies (for collision checking)
    void getBodies(std::vector<b2Body*>& bodies) override;
    Box2DLinkPtr getBox2DLink(b2Body* body) override;

private:
    bool _destroyed;
    Box2DObjectPtr _robot_object;
    mutable std::recursive_mutex _controller_mutex; // for locks to access _controller_callback
    ControlCallback _controller_callback;

    // applies the given efforts to the underlying Box2D bodies. Locks the Box2D world.
    void commandEfforts(const Eigen::VectorXf& target);
};

// class Box2DCollisionRecorder : public b2ContactListener {
//     public:
//         void resetRecordings();
//         void getRecordedContacts(std::vector<Contact>& contacts) const;
//         // b2ContactListener
//         void BeginContact(b2Contact* contact) override;
//         void EndContact(b2Contact* contact) override;
//         void PreSolve(b2Contact* contact, const b2Manifold* oldManifold) override;
//         void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) override;
//     private:

// };

// class Box2DContactListenerDistributor : public b2ContactListener {
//     public:
//         Box2DContactListenerDistributor();
//         ~Box2DContactListenerDistributor();
//         void BeginContact(b2Contact* contact) override;
//         void EndContact(b2Contact* contact) override;
//         void PreSolve(b2Contact* contact, const b2Manifold* oldManifold) override;
//         void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) override;
//         std::vector<b2ContactListener*> listeners;
// };

class Box2DCollisionChecker : public b2ContactListener {
public:
    explicit Box2DCollisionChecker(Box2DWorldPtr world);
    ~Box2DCollisionChecker() override;
    /**
         * Checks whether both provided collidables collide.
         * @param collidable_a
         * @param collidable_b
         * @return true iff there is a collision between both collidables
         */
    bool checkCollision(Box2DCollidablePtr collidable_a, Box2DCollidablePtr collidable_b);

    /**
         * Checks whether both provided collidables collide.
         * @param collidable_a
         * @param collidable_b
         * @param contacts - all detected contacts are stored in this list
         * @return true iff there is a collision between both collidables
         */
    bool checkCollision(Box2DCollidablePtr collidable_a, Box2DCollidablePtr collidable_b, std::vector<Contact>& contacts);

    /**
         * Checks whether the provided collidable collides.
         * @param collidable_a
         * @return true iff collidable is in collision with any other object
         */
    bool checkCollision(Box2DCollidablePtr collidable);

    /**
         * Checks whether the provided collidable collides.
         * @param collidable_a
         * @param contacts - all detected contacts are stored in this list
         * @return true iff collidable is in collision with any other object
         */
    bool checkCollision(Box2DCollidablePtr collidable, std::vector<Contact>& contacts);

    /**
     * Checks whether the provided collidable collides with any link and returns all these links in links.
     *  @param collidable
     *  @param links - will contain all links colliding with collidable
     *  @return true iff collidable in collision with any link
     */
    bool checkCollision(Box2DCollidablePtr collidable, std::vector<Box2DLinkPtr>& links);

    /**
         * Checks whether collidable_a is in collision with any of the provided collidables
         * @param collidable_a
         * @param collidables
         * @return  true iff collidable_a collides with any of the provided objects in collidables
         */
    bool checkCollision(Box2DCollidablePtr collidable_a, const std::vector<Box2DCollidablePtr>& collidables);

    /**
         * Checks whether collidable_a is in collision with any of the provided collidables
         * @param collidable_a
         * @param collidables
         * @param contacts - all detected contacts are stored in this list
         * @return  true iff collidable_a collides with any of the provided objects in collidables
         */
    bool checkCollision(Box2DCollidablePtr collidable_a,
        const std::vector<Box2DCollidablePtr>& collidables,
        std::vector<Contact>& contacts);

    /**
         * Resets the internal contact recorder.
         */
    void startRecordings();

    /**
         *  Returns all contacts that occurred since resetRecordings was called the last time.
         */
    void getRecordedContacts(std::vector<Contact>& contacts);

    /**
         * Invalidates the internal contact cache
         */
    void invalidateCache();

    std::function<void(LinkPtr, LinkPtr)> contact_report_fn;

    // b2ContactListener
    void BeginContact(b2Contact* contact) override;
    void EndContact(b2Contact* contact) override;
    void PreSolve(b2Contact* contact, const b2Manifold* oldManifold) override;
    void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) override;

private:
    Box2DWorldWeakPtr _weak_world;
    bool _cache_invalid;
    bool _b_record_contacts;
    float _scale;
    typedef std::unordered_map<b2Body*, std::vector<Contact>> ContactMap;
    std::unordered_map<b2Body*, ContactMap> _body_contact_maps;
    std::unordered_map<b2Body*, Box2DLinkWeakPtr> _body_to_link_map;
    // a list of contacts registered during a physics propagation
    typedef std::tuple<b2Fixture*, b2Fixture*, Contact> ContactRecord;
    std::vector<ContactRecord> _registered_contacts;

    void updateContactCache();
    // TODO maybe we can make some of these inline?
    void updateContactMaps(b2Body* body_a, b2Body* body_b, b2Contact* contact);
    void addContactRecording(b2Body* body_a, b2Body* body_b, b2Contact* contact);
    bool areInContact(b2Body* body_a, b2Body* body_b) const;
    bool hasContacts(b2Body* body) const;
    Box2DWorldPtr getWorld();
    Box2DLinkPtr getBox2DLink(b2Body* body, Box2DCollidablePtr collidable = nullptr);
};

/**
     * Box2DWorld - This class implements the World interface for box2d.
     */
class Box2DWorld : public World, public std::enable_shared_from_this<Box2DWorld> {
    friend class Box2DLink; // these classes need access to the underlying Box2D world
    friend class Box2DJoint;
    friend class Box2DObject;
    friend class Box2DRobot;
    friend class Box2DCollisionChecker;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    static constexpr float GROUND_DEFAULT_MIN_X = -100.0f;
    static constexpr float GROUND_DEFAULT_MIN_Y = -100.0f;
    static constexpr float GROUND_DEFAULT_MAX_X = 100.0f;
    static constexpr float GROUND_DEFAULT_MAX_Y = 100.0f;
    static constexpr float GRAVITY = 9.81f;
    Box2DWorld();
    Box2DWorld(const Box2DWorld& world) = delete;
    Box2DWorld& operator=(const Box2DWorld& world) = delete;
    Box2DWorld& operator=(Box2DWorld&&) = delete;
    // Destroys all its entities
    ~Box2DWorld(); // we can ignore the warning that we are hiding enable_shared_from_this' destructor

    WorldPtr clone() const override;
    Box2DWorldPtr cloneBox2D() const;

    void loadWorld(const std::string& path) override;
    void loadWorld(const Box2DEnvironmentDescription& env_desc);

    RobotPtr getRobot(const std::string& name) override;
    RobotConstPtr getRobotConst(const std::string& name) const override;
    Box2DRobotPtr getBox2DRobot(const std::string& name);
    Box2DRobotConstPtr getBox2DRobotConst(const std::string& name) const;
    void getRobots(std::vector<RobotPtr>& robots) override;
    void getRobots(std::vector<RobotConstPtr>& robots) const override;
    void getBox2DRobots(std::vector<Box2DRobotPtr>& robots);
    void getBox2DRobots(std::vector<Box2DRobotConstPtr>& robots) const;
    bool isRobot(const std::string& name) const;

    ObjectPtr getObject(const std::string& name, bool exclude_robots) override;
    ObjectConstPtr getObjectConst(const std::string& name, bool exclude_robots) const override;
    Box2DObjectPtr getBox2DObject(const std::string& name);
    Box2DObjectConstPtr getBox2DObjectConst(const std::string& name) const;
    void getBox2DObjects(std::vector<Box2DObjectPtr>& objects);
    void getBox2DObjects(std::vector<Box2DObjectConstPtr>& objects) const;
    void getObjects(std::vector<ObjectPtr>& objects, bool exclude_robots) override;
    void getObjects(std::vector<ObjectConstPtr>& objects, bool exclude_robots) const override;

    void getObjects(const BoundingBox& aabb, std::vector<ObjectPtr>& objects, bool exclude_robots) override;
    void getObjects(const BoundingBox& aabb, std::vector<ObjectConstPtr>& objects,
        bool exclude_robots = true) const override;

    void stepPhysics(int steps) override;
    void stepPhysics(std::vector<Contact>& contacts, int steps = 1) override;
    bool stepPhysics(const std::function<bool(LinkPtr, LinkPtr)>& callback, int steps = 1) override;
    void stepPhysics(int steps, bool execute_controller, bool allow_sleeping);
    void stepPhysics(std::vector<Contact>& contacts, int steps, bool execute_controller, bool allow_sleeping);
    bool stepPhysics(const std::function<bool(LinkPtr, LinkPtr)>& callback, int steps, bool execute_controller, bool allow_sleeping);
    bool supportsPhysics() const override;
    void setPhysicsTimeStep(float physics_step) override;
    void setVelocitySteps(int velocity_steps);
    void setPositionSteps(int position_steps);
    float getPhysicsTimeStep() const override;
    int getVelocitySteps() const;
    int getPositionSteps() const;
    bool isPhysicallyFeasible() override;

    WorldViewer::ImageRendererPtr getImageRenderer() override;
    WorldViewerPtr getViewer() override;
    LoggerPtr getLogger() override;
    LoggerConstPtr getConstLogger() const override;

    bool checkCollision(std::vector<Contact>& contacts) override;
    bool checkCollision(LinkPtr link) override;
    bool checkCollision(LinkPtr link, std::vector<Contact>& contacts) override;
    bool checkCollision(LinkPtr link, const std::vector<LinkPtr>& other_links) override;
    bool checkCollision(LinkPtr link, const std::vector<LinkPtr>& other_links,
        std::vector<Contact>& contacts) override;
    bool checkCollision(ObjectPtr object_a, ObjectPtr object_b) override;
    bool checkCollision(ObjectPtr object_a, ObjectPtr object_b, std::vector<Contact>& contacts) override;
    bool checkCollision(ObjectPtr object_a, LinkPtr link_b) override;
    bool checkCollision(ObjectPtr object_a, LinkPtr link_b, std::vector<Contact>& contacts) override;
    bool checkCollision(ObjectPtr object) override;
    bool checkCollision(ObjectPtr object, std::vector<Contact>& contacts) override;
    bool checkCollision(ObjectPtr object, std::vector<ObjectPtr>& collidors) override;
    bool checkCollision(ObjectPtr object_a, const std::vector<ObjectPtr>& other_objects) override;
    bool checkCollision(ObjectPtr object_a, const std::vector<ObjectPtr>& other_objects,
        std::vector<Contact>& contacts) override;
    bool checkCollision(LinkPtr link_a, const std::vector<ObjectPtr>& other_objects) override;
    bool checkCollision(LinkPtr link_a, const std::vector<ObjectPtr>& other_objects,
        std::vector<Contact>& contacts) override;
    std::recursive_mutex& getMutex() const override;

    bool atRest(float threshold = 0.0001f) const override;

    void saveState() override;
    bool restoreState() override;
    void dropState() override;
    WorldState getWorldState() const override;
    void getWorldState(WorldState& state) const override;
    bool setWorldState(WorldState& state) override;
    void printWorldState(Logger::LogLevel level) const override;

    // Box2D specific
    inline float getScale() const
    {
        return _scale;
    }
    inline float getInverseScale() const
    {
        assert(_scale > 0.0f);
        return 1.0f / _scale;
    }
    inline float getGravity() const
    {
        return GRAVITY * getScale();
    }
    Eigen::Vector4f getWorldBounds() const;
    void invalidateCollisionCache();
    void setToRest();

protected:
    std::shared_ptr<b2World> getRawBox2DWorld();
    b2Body* getGroundBody();
    Box2DLinkPtr getBox2DLink(b2Body* body);

private:
    mutable std::recursive_mutex _world_mutex;
    Box2DEnvironmentDescription _env_desc;
    std::shared_ptr<b2World> _world;
    b2Body* _b2_ground_body;
    Box2DCollisionCheckerPtr _collision_checker;
    float _time_step;
    int _velocity_steps;
    int _position_steps;
    LoggerPtr _logger;
    float _scale;
    std::map<std::string, Box2DObjectPtr> _objects;
    std::map<std::string, Box2DRobotPtr> _robots;
    std::stack<WorldState> _state_stack;
    Eigen::Vector4f _world_bounds;
    WorldViewerPtr _world_viewer;

    void eraseWorld();
    void deleteGroundBody();
    void createWorld(const Box2DEnvironmentDescription& env_desc);
    void createNewRobot(const Box2DRobotDescription& robot_desc);
    void createNewObject(const Box2DObjectDescription& object_desc);
    void createGround(const Eigen::Vector4f& world_bounds);
    Box2DCollidablePtr castCollidable(ObjectPtr object) const;
    Box2DCollidablePtr castCollidable(LinkPtr link) const;
};
}

#endif //BOX2D_SIM_ENV_BOX2DWORLD_H
