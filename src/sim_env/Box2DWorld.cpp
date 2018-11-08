//
// Created by joshua on 4/20/17.
//
#include "sim_env/Box2DWorld.h"
#include "sim_env/Box2DImageRenderer.h"
#include "sim_env/Box2DWorldViewer.h"
#include "sim_env/utils/MathUtils.h"
#include "sim_env/utils/YamlUtils.h"
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <cmath>
#include <exception>
#include <map>
#include <unordered_set>
#include <vector>

#define FLOAT_NOISE_TOLERANCE 0.0001f
#define MAX_ALLOWED_INTERSECTION_AREA 0.0001f

using namespace sim_env;
namespace bg = boost::geometry;

typedef std::lock_guard<std::recursive_mutex> Box2DWorldLock;

////////////////////////////////////////////////////////////////////////////////
/////////////////////* Definition of Box2DLink members *////////////////////////
////////////////////////////////////////////////////////////////////////////////
Box2DLink::Box2DBodyUserData::Box2DBodyUserData(const std::string& lname, const std::string& oname, bool collision_ignore)
    : link_name(lname)
    , object_name(oname)
    , b_collision_ignore(collision_ignore)
{
}

Box2DLink::Box2DBodyUserData::~Box2DBodyUserData() = default;

////////////////////////////////////////////////////////////////////////////////
/////////////////////* Definition of Box2DLink members *////////////////////////
////////////////////////////////////////////////////////////////////////////////
Box2DLink::Box2DLink(const Box2DLinkDescription& link_desc, Box2DWorldPtr world,
    bool is_static, const std::string& object_name)
    : _destroyed(false)
    , _enabled(true)
{
    Box2DWorldLock lock(world->getMutex());
    _name = link_desc.name;
    _world = world;
    _object_name = object_name;
    _local_origin_offset = b2Vec2(0.0f, 0.0f);
    // save ball approximation
    for (auto& pair_ball : link_desc.balls) {
        _balls.emplace_back(Ball(pair_ball.first, pair_ball.second));
    }
    // Create Box2D body
    std::shared_ptr<b2World> box2d_world = world->getRawBox2DWorld();
    b2BodyDef body_def;
    body_def.allowSleep = true;
    body_def.awake = true;
    body_def.type = is_static ? b2_staticBody : b2_dynamicBody;
    body_def.userData = new Box2DBodyUserData(_name, _object_name);
    _body = box2d_world->CreateBody(&body_def);
    // run over polygons and create Box2D shape definitions + compute area
    float area = 0.0f;
    std::vector<b2PolygonShape> shape_defs;
    _local_aabb.min_corner[0] = std::numeric_limits<float>::max();
    _local_aabb.min_corner[1] = std::numeric_limits<float>::max();
    _local_aabb.max_corner[0] = std::numeric_limits<float>::lowest();
    _local_aabb.max_corner[1] = std::numeric_limits<float>::lowest();
    _local_aabb.min_corner[2] = 0.0f;
    _local_aabb.max_corner[2] = 0.0f;
    for (auto& polygon : link_desc.polygons) {
        b2PolygonShape b2_shape; // shape type for box2d
        std::vector<b2Vec2> b2_polygon; // temporal buffer for vertices
        // boost polygon for area, 'false' template parameter stands for counter clockwise (box2d standard)
        bg::model::polygon<bg::model::d2::point_xy<float>, false> boost_polygon;
        assert(polygon.size() % 2 == 0);
        // run over this polygon
        for (unsigned int i = 0; i < polygon.size() / 2; ++i) {
            _local_aabb.min_corner[0] = std::min(polygon.at(2 * i), _local_aabb.min_corner[0]);
            _local_aabb.min_corner[1] = std::min(polygon.at(2 * i + 1), _local_aabb.min_corner[1]);
            _local_aabb.max_corner[0] = std::max(polygon.at(2 * i), _local_aabb.max_corner[0]);
            _local_aabb.max_corner[1] = std::max(polygon.at(2 * i + 1), _local_aabb.max_corner[1]);
            //  put vertex in buffer (scaled) and in boost polygon
            b2_polygon.emplace_back(b2Vec2(world->getScale() * polygon.at(2 * i), world->getScale() * polygon.at(2 * i + 1)));
            bg::append(boost_polygon, bg::model::d2::point_xy<float>(polygon.at(2 * i), polygon.at(2 * i + 1)));
        }
        bg::correct(boost_polygon); // ensures that the polygon fulfills all criteria required for a valid boost polygon
        _boost_polygons.push_back(boost_polygon);
        // compute the area (scaled)
        area += world->getScale() * world->getScale() * bg::area(boost_polygon);
        // add the shape
        b2_shape.Set(b2_polygon.data(), (int32)b2_polygon.size());
        shape_defs.push_back(b2_shape);
    }
    assert(area > 0.0);
    // Now that we have the area and all shapes, we can create fixtures
    for (b2PolygonShape& shape_def : shape_defs) {
        b2FixtureDef fixture_def;
        fixture_def.isSensor = false;
        fixture_def.density = link_desc.mass / area;
        fixture_def.friction = link_desc.contact_friction;
        fixture_def.restitution = link_desc.restitution;
        fixture_def.shape = &shape_def;
        _body->CreateFixture(&fixture_def);
    }
    // Finally create a friction joint between this link and the ground plane
    // TODO we probably don't want friction between the ground and fingers -> make friction optional
    float gravity = world->getGravity();
    b2FrictionJointDef friction_joint_def;
    friction_joint_def.localAnchorA.SetZero();
    friction_joint_def.localAnchorB = _body->GetLocalCenter();
    friction_joint_def.bodyA = world->getGroundBody();
    friction_joint_def.bodyB = _body;
    friction_joint_def.collideConnected = false;
    friction_joint_def.maxForce = link_desc.trans_friction * _body->GetMass() * gravity;
    friction_joint_def.maxTorque = link_desc.rot_friction * _body->GetMass() * gravity;
    _ground_friction = link_desc.trans_friction;
    _friction_ratio = link_desc.rot_friction / link_desc.trans_friction;
    _friction_joint = box2d_world->CreateJoint(&friction_joint_def);
}

Box2DLink::~Box2DLink()
{
    if (!_destroyed) {
        auto logger = DefaultLogger::getInstance();
        logger->logErr("[Box2DLink::~Box2DLink] Destructor called without prior object destruction.");
    }
}

void Box2DLink::destroy(const std::shared_ptr<b2World>& b2world)
{
    //    sim_env::DefaultLogger::getInstance()->logDebug("Destroying Box2DLink", "[sim_env::Box2DLink::destroy]");
    b2world->DestroyJoint(_friction_joint);
    auto* user_data = static_cast<Box2DBodyUserData*>(_body->GetUserData());
    _body->SetUserData(nullptr);
    delete user_data;
    b2world->DestroyBody(_body);
    _destroyed = true;
}

std::string Box2DLink::getName() const
{
    return _name;
}

void Box2DLink::setName(const std::string& name)
{
    _name = name;
    auto user_data = static_cast<Box2DBodyUserData*>(_body->GetUserData());
    user_data->link_name = name;
}

void Box2DLink::setObjectName(const std::string& name)
{
    _object_name = name;
    auto user_data = static_cast<Box2DBodyUserData*>(_body->GetUserData());
    user_data->object_name = name;
}

EntityType Box2DLink::getType() const
{
    return EntityType::Link;
}

Eigen::Affine3f Box2DLink::getTransform() const
{
    Box2DWorldPtr world = getBox2DWorld();
    Box2DWorldLock lock(world->getMutex());
    b2Vec2 pos = _body->GetWorldPoint(_local_origin_offset);
    float orientation = _body->GetAngle();
    Eigen::Affine3f transform;
    transform.setIdentity();
    auto& matrix = transform.matrix();
    matrix(0, 0) = cos(orientation);
    matrix(0, 1) = -sin(orientation);
    matrix(1, 0) = -matrix(0, 1);
    matrix(1, 1) = matrix(0, 0);
    matrix(0, 3) = world->getInverseScale() * pos.x;
    matrix(1, 3) = world->getInverseScale() * pos.y;
    // transform = Eigen::Translation3f(world->getInverseScale() * pos.x,
    //                                world->getInverseScale() * pos.y,
    //                                0.0);
    // transform.rotate(Eigen::AngleAxisf(orientation, Eigen::Vector3f::UnitZ()));
    return transform;
}

void Box2DLink::getPose(Eigen::Vector3f& pose) const
{
    Box2DWorldConstPtr world = getBox2DWorld();
    Box2DWorldLock lock(world->getMutex());
    b2Vec2 pos = _body->GetWorldPoint(_local_origin_offset);
    float orientation = _body->GetAngle();
    pose[0] = world->getInverseScale() * pos.x;
    pose[1] = world->getInverseScale() * pos.y;
    pose[2] = orientation;
}

Eigen::Vector3f Box2DLink::getPose() const
{
    Eigen::Vector3f pose;
    getPose(pose);
    return pose;
}

void Box2DLink::getVelocityVector(Eigen::Vector3f& vel_vector) const
{
    Box2DWorldConstPtr world = getBox2DWorld();
    Box2DWorldLock lock(world->getMutex());
    b2Vec2 b2_vel = _body->GetLinearVelocity();
    vel_vector[0] = world->getInverseScale() * b2_vel.x;
    vel_vector[1] = world->getInverseScale() * b2_vel.y;
    vel_vector[2] = _body->GetAngularVelocity();
}

Eigen::Vector3f Box2DLink::getVelocityVector() const
{
    Eigen::Vector3f vel;
    getVelocityVector(vel);
    return vel;
}

Box2DWorldPtr Box2DLink::getBox2DWorld() const
{
    if (_world.expired()) {
        throw std::logic_error("[Box2DLink::getBox2DWorld] Can not access Box2DWorld. A link should not exist without a world.");
    }
    return _world.lock();
}

BoundingBox Box2DLink::getLocalBoundingBox() const
{
    return _local_aabb;
}

float Box2DLink::getGroundFriction() const
{
    return _ground_friction;
}

WorldPtr Box2DLink::getWorld() const
{
    return getBox2DWorld();
}

WorldConstPtr Box2DLink::getConstWorld() const
{
    return getWorld();
}

ObjectPtr Box2DLink::getObject() const
{
    WorldPtr world = getWorld();
    return world->getObject(_object_name, false);
}

ObjectConstPtr Box2DLink::getConstObject() const
{
    return getObject();
}

b2Body* Box2DLink::getBody()
{
    return _body;
}

void Box2DLink::registerChildJoint(Box2DJointPtr joint)
{
    _child_joints.push_back(Box2DJointWeakPtr(joint));
}

void Box2DLink::registerParentJoint(Box2DJointPtr joint)
{
    if (not _parent_joint.expired()) {
        auto logger = getWorld()->getLogger();
        logger->logWarn("Resetting parent joint.", "[sim_env::Box2DLink::registerParentJoint]");
    }
    _parent_joint = joint;
}

void Box2DLink::getGeometry(std::vector<std::vector<Eigen::Vector2f>>& geometry) const
{
    // TODO we could/should just read this from the boost polygons
    Box2DWorldPtr world = getBox2DWorld();
    Box2DWorldLock lock(world->getMutex());
    b2Fixture* fixture = _body->GetFixtureList();
    while (fixture) {
        b2Shape* shape = fixture->GetShape();
        std::vector<Eigen::Vector2f> polygon;
        if (shape->GetType() == b2Shape::Type::e_polygon) {
            b2PolygonShape* polygon_shape = static_cast<b2PolygonShape*>(shape);
            for (int32 v = 0; v < polygon_shape->GetVertexCount(); ++v) {
                b2Vec2 point = polygon_shape->GetVertex(v) - _local_origin_offset;
                Eigen::Vector2f eigen_point(world->getInverseScale() * point.x, world->getInverseScale() * point.y);
                polygon.push_back(eigen_point);
            }
            geometry.push_back(polygon);
        } else {
            throw std::runtime_error("[sim_env::Box2DLink::getGeometry] Could not retrieve geometry. Box2D shape is not a polygon.");
        }
        fixture = fixture->GetNext();
    }
}

void Box2DLink::getBoostGeometry(
    std::vector<boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<float>, false>>& polygons) const
{
    polygons = _boost_polygons;
}

void Box2DLink::getWorldBoostGeometry(
    std::vector<boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<float>, false>>& polygons) const
{

    struct TransformStruct {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Eigen::Affine3f my_transform;
        b2Vec2 shift_vec;
        TransformStruct(const Eigen::Affine3f& transform, b2Vec2 offset)
            : my_transform(transform)
            , shift_vec(offset)
        {
        }
        inline void operator()(bg::model::d2::point_xy<float>& point)
        {
            Eigen::Vector4f position(bg::get<0>(point) - shift_vec.x, bg::get<1>(point) - shift_vec.y, 0.0f, 1.0f);
            position = my_transform * position;
            bg::set<0>(point, position[0]);
            bg::set<1>(point, position[1]);
        }
    };

    auto world = getBox2DWorld();
    for (auto& polygon : _boost_polygons) {
        bg::model::polygon<bg::model::d2::point_xy<float>, false> world_polygon(polygon);
        bg::for_each_point(world_polygon, TransformStruct(getTransform(), world->getInverseScale() * _local_origin_offset));
        polygons.push_back(world_polygon);
    }
}
//void Box2DLink::setTransform(const Eigen::Affine3f &tf) {
//    if (not _parent_joint.expired()) {
//        throw std::logic_error("Setting transform of link within kinematic chain directly, is not supported."
//                                   "[sim_env::Box2DLink::setTransform]");
//    }
//    auto rotation_matrix = tf.rotation();
//    float theta = (float) acos(rotation_matrix(0, 0));
//    theta = rotation_matrix(1,0) > 0.0 ? theta: -theta;
//    float x = tf.translation()(0);
//    float y = tf.translation()(1);
//    Box2DObjectPtr object = getBox2DObject();
//    setPose(Eigen::Vector3f(x, y, theta));
//}

void Box2DLink::setPose(const Eigen::Vector3f& pose, bool update_children, bool joint_override)
{
    Box2DWorldPtr world = getBox2DWorld();
    Box2DWorldLock lock(world->getMutex());
    // first save joint values of child joints
    std::vector<float> child_joint_values;
    if (update_children) {
        for (auto& weak_child_joint : _child_joints) {
            JointPtr child_joint = weak_child_joint.lock();
            if (not joint_override) {
                child_joint_values.push_back(child_joint->getPosition());
            } else {
                child_joint_values.push_back(0.0f);
            }
        }
    }
    // now set the new pose for this link
    b2Vec2 base_link_translation(world->getScale() * pose[0], world->getScale() * pose[1]);
    float theta = pose[2];
    // we need to compute the world position of the body's origin (which might be different form the link's origin)
    b2Rot rotate_theta(theta);
    b2Vec2 offset = b2Mul(rotate_theta, _local_origin_offset);
    _body->SetTransform(base_link_translation - offset, theta);
    // next, update children if requested
    if (update_children) {
        int child_id = 0;
        for (auto& weak_child_joint : _child_joints) {
            Box2DJointPtr child_joint = weak_child_joint.lock();
            child_joint->resetPosition(child_joint_values.at(child_id), joint_override);
            ++child_id;
        }
    }
    world->invalidateCollisionCache();
}

void Box2DLink::setVelocityVector(const Eigen::Vector3f& velocity)
{
    _body->SetLinearVelocity(b2Vec2(velocity[0], velocity[1]));
    _body->SetAngularVelocity(velocity[2]);
    //    Box2DWorldPtr world = getBox2DWorld();
    //    Box2DWorldLock lock(world->getMutex());
    //    b2Vec2 linear_vel_change(world->getScale() * velocity[0], world->getScale() * velocity[1]);
    //    float angular_vel_change = velocity[2];
    //    if (not relative) {
    //        linear_vel_change = linear_vel_change - _body->GetLinearVelocity();
    //        angular_vel_change = angular_vel_change - _body->GetAngularVelocity();
    //    }
    //    // before resetting this link's velocity, let children update theirs first,
    //    // so they can keep their joint velocities
    //    b2Vec2 current_vel = _body->GetLinearVelocity();
    //    b2Vec2 new_vel = current_vel + linear_vel_change;
    //    float new_angular_vel = _body->GetAngularVelocity() + angular_vel_change;
    //
    //    for (auto& weak_child_joint : _child_joints) {
    //        JointPtr child_joint = weak_child_joint.lock();
    //        float joint_vel = child_joint->getVelocity();
    //        Box2DLinkPtr child_link = std::static_pointer_cast<Box2DLink>(child_joint->getChildLink());
    //        // TODO test this
    //        child_link->propagateVelocityChange(new_vel.x, new_vel.y,
    //                                            new_angular_vel, joint_vel, _body);
    //
    //    }
    //    _body->SetLinearVelocity(new_vel);
    //    _body->SetAngularVelocity(new_angular_vel);
}

float Box2DLink::getMass() const
{
    Box2DWorldPtr world = getBox2DWorld();
    Box2DWorldLock lock(world->getMutex());
    return _body->GetMass();
}

void Box2DLink::setMass(float mass)
{
    Box2DWorldPtr world = getBox2DWorld();
    Box2DWorldLock lock(world->getMutex());
    b2MassData data;
    _body->GetMassData(&data);
    float mass_ratio = mass / data.mass;
    data.mass = mass;
    data.I = data.I * mass_ratio;
    _body->SetMassData(&data);
}

void Box2DLink::setGroundFriction(float coeff)
{
    Box2DWorldPtr world = getBox2DWorld();
    Box2DWorldLock lock(world->getMutex());
    auto friction_joint = dynamic_cast<b2FrictionJoint*>(_friction_joint);
    float gravity = world->getGravity();
    friction_joint->SetMaxForce(coeff * _body->GetMass() * gravity);
    friction_joint->SetMaxTorque(_friction_ratio * coeff * _body->GetMass() * gravity);
    _ground_friction = coeff;
}

float Box2DLink::getInertia() const
{
    Box2DWorldPtr world = getBox2DWorld();
    Box2DWorldLock lock(world->getMutex());
    float inv_scale = world->getInverseScale();
    return inv_scale * inv_scale * _body->GetInertia();
}

void Box2DLink::getCenterOfMass(Eigen::Vector3f& com) const
{
    Eigen::Vector2f com2;
    getCenterOfMass(com2);
    com[0] = com2[0];
    com[1] = com2[1];
    com[2] = 0.0f;
}

void Box2DLink::getLocalCenterOfMass(Eigen::Vector3f& com) const
{
    Box2DWorldPtr world = getBox2DWorld();
    Box2DWorldLock lock(world->getMutex());
    auto b2center = _body->GetLocalCenter() - _local_origin_offset;
    float scale = world->getInverseScale();
    com[0] = b2center.x * scale;
    com[1] = b2center.y * scale;
    com[2] = 0.0f;
}

Eigen::Vector2f Box2DLink::getCenterOfMass() const
{
    Eigen::Vector2f com;
    getCenterOfMass(com);
    return com;
}

void Box2DLink::getCenterOfMass(Eigen::Vector2f& com) const
{
    Box2DWorldPtr world = getBox2DWorld();
    Box2DWorldLock lock(world->getMutex());
    float inv_scale = world->getInverseScale();
    b2Vec2 box2d_com = _body->GetWorldCenter();
    com[0] = inv_scale * box2d_com.x;
    com[1] = inv_scale * box2d_com.y;
}

void Box2DLink::getChildJoints(std::vector<JointPtr>& child_joints)
{
    Box2DWorldPtr world = getBox2DWorld();
    Box2DWorldLock lock(world->getMutex());
    for (auto& child_joint_wptr : _child_joints) {
        JointPtr child_joint = child_joint_wptr.lock();
        if (!child_joint) {
            throw std::logic_error("[sim_env::Box2DLink::getChildJoints] Child joint does not exist anymore.");
        }
        child_joints.push_back(child_joint);
    }
}

void Box2DLink::getConstChildJoints(std::vector<JointConstPtr>& child_joints) const
{
    Box2DWorldPtr world = getBox2DWorld();
    Box2DWorldLock lock(world->getMutex());
    for (auto& child_joint_wptr : _child_joints) {
        JointPtr child_joint = child_joint_wptr.lock();
        if (!child_joint) {
            throw std::logic_error("[sim_env::Box2DLink::getConstChildJoints] Child joint does not exist anymore.");
        }
        child_joints.push_back(child_joint);
    }
}

JointPtr Box2DLink::getParentJoint()
{
    Box2DWorldPtr world = getBox2DWorld();
    Box2DWorldLock lock(world->getMutex());
    JointPtr parent_joint = _parent_joint.lock();
    if (!parent_joint) {
        throw std::logic_error("[sim_env::Box2DLink::getParentJoints] parent joint does not exist anymore.");
    }
    return parent_joint;
}

JointConstPtr Box2DLink::getConstParentJoint() const
{
    Box2DWorldPtr world = getBox2DWorld();
    Box2DWorldLock lock(world->getMutex());
    JointPtr parent_joint = _parent_joint.lock();
    if (!parent_joint) {
        throw std::logic_error("[sim_env::Box2DLink::getConstParentJoints] parent joint does not exist anymore.");
    }
    return parent_joint;
}

void Box2DLink::setOriginToCenterOfMass()
{
    Box2DWorldPtr world = getBox2DWorld();
    Box2DWorldLock lock(world->getMutex());
    if (not _parent_joint.expired()) {
        world->getLogger()->logWarn("Resetting origin to center of mass for non-base link",
            "[sim_env::Box2DLink::setOriginToCenterOfMass]");
    }
    _local_origin_offset = _body->GetLocalCenter();
}

bool Box2DLink::checkCollision()
{
    Box2DWorldPtr world = getBox2DWorld();
    return world->checkCollision(shared_from_this());
}

bool Box2DLink::checkCollision(std::vector<Contact>& contacts)
{
    Box2DWorldPtr world = getBox2DWorld();
    return world->checkCollision(shared_from_this());
}

bool Box2DLink::checkCollision(const std::vector<LinkPtr>& other_links)
{
    Box2DWorldPtr world = getBox2DWorld();
    return world->checkCollision(shared_from_this(), other_links);
}

bool Box2DLink::checkCollision(const std::vector<LinkPtr>& other_links, std::vector<Contact>& contacts)
{
    Box2DWorldPtr world = getBox2DWorld();
    return world->checkCollision(shared_from_this(), other_links, contacts);
}

bool Box2DLink::checkCollision(const std::vector<ObjectPtr>& other_objects)
{
    Box2DWorldPtr world = getBox2DWorld();
    return world->checkCollision(shared_from_this(), other_objects);
}

bool Box2DLink::checkCollision(const std::vector<ObjectPtr>& other_objects, std::vector<Contact>& contacts)
{
    Box2DWorldPtr world = getBox2DWorld();
    return world->checkCollision(shared_from_this(), other_objects, contacts);
}

void Box2DLink::getBallApproximation(std::vector<Ball>& balls) const
{
    balls.reserve(balls.size() + _balls.size());
    Ball tmp_ball;
    auto tf = getTransform();
    for (auto& ball : _balls) {
        tmp_ball.radius = ball.radius;
        tmp_ball.center = tf * ball.center;
        balls.push_back(tmp_ball);
    }
}

unsigned int Box2DLink::getNumApproximationBalls() const
{
    return _balls.size();
}

void Box2DLink::getLocalBallApproximation(std::vector<Ball>& balls) const
{
    balls.reserve(balls.size() + _balls.size());
    balls.insert(balls.end(), _balls.begin(), _balls.end());
}

void Box2DLink::updateBallApproximation(std::vector<Ball>& balls,
    std::vector<Ball>::iterator& start,
    std::vector<Ball>::iterator& end) const
{
    auto my_balls_iter = _balls.begin();
    size_t num_balls_updated = 0;
    auto tf = getTransform();
    while (start != end and my_balls_iter != _balls.end()) {
        start->radius = my_balls_iter->radius;
        start->center = tf * my_balls_iter->center;
        ++start;
        ++my_balls_iter;
        ++num_balls_updated;
    }
    assert(num_balls_updated == _balls.size());
}

LinkPtr Box2DLink::getLink(b2Body* body)
{
    return shared_from_this();
}

void Box2DLink::getBodies(std::vector<b2Body*>& bodies)
{
    bodies.push_back(_body);
}

void Box2DLink::updateBodyVelocities(const Eigen::VectorXf& all_dof_velocities,
    std::vector<std::pair<b2Vec2, unsigned int>>& parent_joints,
    unsigned int index_offset)
{
    float scale = getBox2DWorld()->getScale();
    // init the linear velocity for this link with the base velocity
    b2Vec2 my_linear_velocity(scale * all_dof_velocities[0], scale * all_dof_velocities[1]);
    float my_angular_velocity = 0.0f;
    // now run over all parent joints and compute the tangential speed
    b2Vec2 my_com = _body->GetWorldCenter();
    for (unsigned int i = 0; i < parent_joints.size(); ++i) {
        // get the rotational axis of the parent joint (in world frame)
        b2Vec2 axis_i = parent_joints[i].first;
        unsigned int joint_idx_i = parent_joints[i].second;
        b2Vec2 com_i = my_com - axis_i; // vector pointing from axis to center of mass of this link
        // omega x r (i.e. joint_vel cross com_i)
        my_linear_velocity.x -= all_dof_velocities[joint_idx_i] * com_i.y;
        my_linear_velocity.y += all_dof_velocities[joint_idx_i] * com_i.x;
        // the angular velocity is the sum of all parent frames
        my_angular_velocity += all_dof_velocities[joint_idx_i];
    }
    // set the velocity
    _body->SetLinearVelocity(my_linear_velocity);
    _body->SetAngularVelocity(my_angular_velocity);
    // now propagate further down in the kinematic chain
    for (const auto& weak_child_joint : _child_joints) {
        auto child_joint = weak_child_joint.lock();
        if (not child_joint) {
            throw std::logic_error("[Box2DLink::updateBodyVelocities] Can not access child joint. Invalid weak pointer");
        }
        parent_joints.emplace_back(std::make_pair(child_joint->getGlobalAxisPosition(),
            child_joint->getDOFIndex() + index_offset));
        child_joint->getChildBox2DLink()->updateBodyVelocities(all_dof_velocities, parent_joints, index_offset);
        parent_joints.pop_back();
    }
}

const std::vector<boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<float>, false>>&
Box2DLink::getBoostGeometry() const
{
    return _boost_polygons;
}

void Box2DLink::setEnabled(bool b_enable)
{
    Box2DWorldPtr world = getBox2DWorld();
    Box2DWorldLock lock(world->getMutex());
    auto logger = world->getLogger();
    if (b_enable) {
        logger->logDebug("Setting link to be enabled");
    } else {
        logger->logDebug("Disabling link");
    }
    if (b_enable == _enabled)
        return;
    _enabled = b_enable;
    b2Fixture* fixture = _body->GetFixtureList();
    while (fixture) {
        b2Filter filter = fixture->GetFilterData();
        filter.categoryBits = b_enable;
        fixture->SetFilterData(filter);
        fixture = fixture->GetNext();
    }
}

bool Box2DLink::isEnabled() const
{
    return _enabled;
}

///////////////////////////////// PRIVATE //////////////////////////////////////
//void Box2DLink::propagateVelocityChange(float v_x,
//                                        float v_y,
//                                        float v_theta,
//                                        float v_joint,
//                                        b2Body* parent) {
//    // the linear velocity applies to each body in a chain in the same way
//    // the angular velocity applies also to each body in the same way.
////    b2MassData mass_data;
////   _body->GetMassData(&mass_data);
////    b2Vec2 center_of_mass_in_world = mass_data.center;
////    b2Vec2 origin_in_parent = parent->GetLocalPoint(center_of_mass_in_world);
//    Box2DJointPtr parent_joint = _parent_joint.lock();
//    if (!parent_joint) {
//        throw std::logic_error("[sim_env::Box2DLink::propagateVelocityChange] Could not access parent joint");
//    }
//    // TODO SOMETHING IS WRONG HERE
//    b2Vec2 origin_in_parent = parent_joint->getLocalAxisPosition();
//    b2Vec2 center_of_mass = _body->GetLocalCenter();
//    center_of_mass = origin_in_parent + center_of_mass;
//    b2Vec2 radial_vel(-center_of_mass.y * v_theta, center_of_mass.x * v_theta);
//    b2Vec2 new_lin_vel(v_x, v_y);
//    new_lin_vel += parent->GetWorldVector(radial_vel);
//    float new_angular_vel = v_theta + v_joint;
//    // propagate change further down in the kinematic chain
//    for (auto& child : _child_joints) {
//        if (child.expired()) {
//            throw std::logic_error("[Box2DLink::propagateVelocityChange] Child joint not available anymore.");
//        }
//        Box2DJointPtr child_joint = child.lock();
//        Box2DLinkPtr child_link = child_joint->getChildBox2DLink();
//        child_link->propagateVelocityChange(new_lin_vel.x, new_lin_vel.y,
//                                            new_angular_vel, child_joint->getVelocity(),
//                                            _body);
//    }
//    _body->SetLinearVelocity(new_lin_vel);
//    _body->SetAngularVelocity(new_angular_vel);
//}

////////////////////////////////////////////////////////////////////////////////
/////////////////////* Definition of Box2DJoint members *///////////////////////
////////////////////////////////////////////////////////////////////////////////

Box2DJoint::Box2DJoint(const Box2DJointDescription& joint_desc, Box2DLinkPtr link_a, Box2DLinkPtr link_b,
    Box2DWorldPtr world, const std::string& object_name)
    : _destroyed(false)
{
    Box2DWorldLock lock(world->getMutex());
    _world = world;
    _name = joint_desc.name;
    _link_a = link_a;
    _link_b = link_b;
    _joint_index = 0;
    _dof_index = 0;
    _object_name = object_name;
    assert(link_a->getName() == joint_desc.link_a);
    assert(link_b->getName() == joint_desc.link_b);
    // Now create a box2d joint
    std::shared_ptr<b2World> box2d_world = world->getRawBox2DWorld();
    if (joint_desc.joint_type == "revolute") {
        _joint_type = JointType::Revolute;
    } else if (joint_desc.joint_type == "prismatic") {
        _joint_type = JointType::Prismatic;
    } else {
        std::stringstream ss;
        ss << "Unknown joint type " << joint_desc.joint_type << ". Using revolute instead.";
        world->getLogger()->logErr(ss.str(), "[Box2DJoint::Box2DJoint]");
        _joint_type = JointType::Revolute;
    }
    switch (_joint_type) {
    case JointType::Revolute: {
        b2RevoluteJointDef joint_def;
        b2Vec2 box2d_axis(world->getScale() * joint_desc.axis[0],
            world->getScale() * joint_desc.axis[1]);
        joint_def.bodyA = link_a->getBody();
        joint_def.bodyB = link_b->getBody();
        joint_def.localAnchorA = box2d_axis; // axis in link_a frame
        joint_def.localAnchorB = b2Vec2(0, 0); // axis in link_b frame
        // base orientation of link_b frame relative to link_a frame
        joint_def.referenceAngle = joint_desc.axis_orientation;
        joint_def.lowerAngle = joint_desc.position_limits[0];
        joint_def.upperAngle = joint_desc.position_limits[1];
        joint_def.enableLimit = joint_desc.position_limits.norm() > 0.0;
        //            joint_def.maxMotorTorque = std::min(std::abs(joint_desc.acceleration_limits[0]),
        //                                                std::abs(joint_desc.acceleration_limits[1]));
        joint_def.enableMotor = joint_desc.actuated;
        _joint = box2d_world->CreateJoint(&joint_def);

        // TODO what about unlimited joints?
        break;
    }
    case JointType::Prismatic: {
        // TODO actually we don't support these right now
        b2PrismaticJointDef joint_def;
        b2Vec2 box2d_axis(joint_desc.axis[0], joint_desc.axis[1]);
        b2Vec2 box2d_direction(std::cos(joint_desc.axis_orientation), std::sin(joint_desc.axis_orientation));
        joint_def.bodyA = link_a->getBody();
        joint_def.bodyB = link_b->getBody();
        joint_def.referenceAngle = joint_desc.axis_orientation;
        joint_def.localAxisA = box2d_direction;
        joint_def.localAnchorA = box2d_axis;
        joint_def.localAnchorB = b2Vec2(0, 0);
        joint_def.lowerTranslation = world->getScale() * joint_desc.position_limits[0];
        joint_def.upperTranslation = world->getScale() * joint_desc.position_limits[1];
        joint_def.enableLimit = joint_desc.position_limits.norm() > 0.0;
        // TODO might have to transform acceleration here to force (force = mass * acceleration)
        joint_def.maxMotorForce = std::min(std::abs(joint_desc.acceleration_limits[0]),
            std::abs(joint_desc.acceleration_limits[1]));
        joint_def.enableMotor = joint_desc.actuated;
        _joint = box2d_world->CreateJoint(&joint_def);
        break;
    }
    }
    _position_limits[0] = joint_desc.position_limits[0];
    _position_limits[1] = joint_desc.position_limits[1];
    _velocity_limits[0] = joint_desc.velocity_limits[0];
    _velocity_limits[1] = joint_desc.velocity_limits[1];
    _acceleration_limits[0] = joint_desc.acceleration_limits[0];
    _acceleration_limits[1] = joint_desc.acceleration_limits[1];
}

Box2DJoint::~Box2DJoint()
{
    if (!_destroyed) {
        auto logger = DefaultLogger::getInstance();
        logger->logErr("[Box2DJoint::~Box2DJoint] Destructor called without prior object destruction.");
    }
}

void Box2DJoint::destroy(const std::shared_ptr<b2World>& b2world)
{
    //    sim_env::DefaultLogger::getInstance()->logDebug("Destroying Box2DJoint", "[sim_env::Box2DJoint::destroy]");
    b2world->DestroyJoint(_joint);
    _destroyed = true;
}

float Box2DJoint::getPosition() const
{
    WorldPtr world = getWorld();
    Box2DWorldLock lock(world->getMutex());
    switch (_joint_type) {
    case JointType::Revolute: {
        b2RevoluteJoint* revolute_joint = static_cast<b2RevoluteJoint*>(_joint);
        return revolute_joint->GetJointAngle();
    }
    case JointType::Prismatic: {
        b2PrismaticJoint* prismatic_joint = static_cast<b2PrismaticJoint*>(_joint);
        return prismatic_joint->GetJointTranslation();
    }
    }
    return 0;
}

void Box2DJoint::setPosition(float v)
{
    Box2DObjectPtr object = getBox2DObject();
    Eigen::VectorXf val(1);
    Eigen::VectorXi indices(1);
    val[0] = v;
    indices[0] = getDOFIndex();
    object->setDOFPositions(val, indices);
}

void Box2DJoint::resetPosition(float value, bool child_joint_override)
{
    auto world = getBox2DWorld();
    Box2DWorldLock lock(world->getMutex());
    auto logger = world->getLogger();
    float clamped_value = sim_env::utils::math::clamp(value, _position_limits[0], _position_limits[1]);
    if (std::abs(clamped_value - value) > FLOAT_NOISE_TOLERANCE) {
        std::stringstream ss;
        ss << "Position " << value << " is out of limits (" << _position_limits.transpose() << ") for joint " << getJointIndex();
        logger->logWarn(ss.str(), "[sim_env::Box2DJoint::resetPosition]");
    }
    switch (_joint_type) {
    case JointType::Revolute: {
        b2RevoluteJoint* revolute_joint = static_cast<b2RevoluteJoint*>(_joint);
        // We need to manually compute how to change the child link's pose such
        // that the revolute joint reaches the desired angle
        // Since the child link's frame is centered at the joint axis, we only
        // need to update the link's orientation and ensure that is placed at the joint axis
        float new_angle = revolute_joint->GetBodyA()->GetAngle() + clamped_value
            + revolute_joint->GetReferenceAngle();
        b2Vec2 axis_world = revolute_joint->GetAnchorA();
        Eigen::Vector3f new_pose_b;
        new_pose_b[0] = world->getInverseScale() * axis_world.x;
        new_pose_b[1] = world->getInverseScale() * axis_world.y;
        new_pose_b[2] = new_angle;
        Box2DLinkPtr link_b = getChildBox2DLink();
        link_b->setPose(new_pose_b, true, child_joint_override);
        break;
    }
    case JointType::Prismatic: {
        //            b2PrismaticJoint* prismatic_joint = static_cast<b2PrismaticJoint*>(_joint);
        //            prismatic_joint->GetJointTranslation();
        //TODO same here, so probably best if this can be done at the end of this switch-case block
        break;
    }
    }
}

float Box2DJoint::getVelocity() const
{
    WorldPtr world = getWorld();
    Box2DWorldLock lock(world->getMutex());
    switch (_joint_type) {
    case JointType::Revolute: {
        b2RevoluteJoint* revolute_joint = static_cast<b2RevoluteJoint*>(_joint);
        return revolute_joint->GetJointSpeed();
    }
    case JointType::Prismatic: {
        b2PrismaticJoint* prismatic_joint = static_cast<b2PrismaticJoint*>(_joint);
        return prismatic_joint->GetJointSpeed();
    }
    default:
        throw std::logic_error("[sim_env::Box2DJoint::getVelocity] This joint has an unsupported type");
    }
}

void Box2DJoint::setVelocity(float v)
{
    Box2DObjectPtr object = getBox2DObject();
    Eigen::VectorXi indices(1);
    indices[0] = getDOFIndex();
    Eigen::VectorXf value(1);
    value[0] = v;
    object->setDOFVelocities(value, indices);
}

Eigen::Array2f Box2DJoint::getPositionLimits() const
{
    return _position_limits;
}

void Box2DJoint::getPositionLimits(Eigen::Array2f& limits) const
{
    limits = _position_limits;
}

Eigen::Array2f Box2DJoint::getVelocityLimits() const
{
    return _velocity_limits;
}

void Box2DJoint::getVelocityLimits(Eigen::Array2f& limits) const
{
    limits = _velocity_limits;
}

unsigned int Box2DJoint::getJointIndex() const
{
    return _joint_index;
}

unsigned int Box2DJoint::getDOFIndex() const
{
    return _dof_index;
}

Joint::JointType Box2DJoint::getJointType() const
{
    return _joint_type;
}

std::string Box2DJoint::getName() const
{
    return _name;
}

void Box2DJoint::setName(const std::string& name)
{
    _name = name;
}

void Box2DJoint::setObjectName(const std::string& name)
{
    _object_name = name;
}

void Box2DJoint::setJointIndex(unsigned int index)
{
    _joint_index = index;
}

void Box2DJoint::setDOFIndex(unsigned int index)
{
    _dof_index = index;
}

EntityType Box2DJoint::getType() const
{
    return EntityType::Joint;
}

Eigen::Affine3f Box2DJoint::getTransform() const
{
    LinkPtr child_link = getChildLink();
    return child_link->getTransform();
}

LinkPtr Box2DJoint::getChildLink() const
{
    return getChildBox2DLink();
}

Box2DLinkPtr Box2DJoint::getChildBox2DLink() const
{
    if (_link_b.expired()) {
        throw std::logic_error("[sim_env::Box2DJoint::getChildBox2DLink] Child link does not exist anymore. This joint should not exist!");
    }
    return _link_b.lock();
}

LinkPtr Box2DJoint::getParentLink() const
{
    return getParentBox2DLink();
}

Box2DLinkPtr Box2DJoint::getParentBox2DLink() const
{
    if (_link_a.expired()) {
        throw std::logic_error("[sim_env::Box2DJoint::getParentBox2DLink] Parent link does not exist anymore. This joint should not exist!");
    }
    return _link_a.lock();
}

WorldPtr Box2DJoint::getWorld() const
{
    return getBox2DWorld();
}

WorldConstPtr Box2DJoint::getConstWorld() const
{
    return getWorld();
}

Box2DWorldPtr Box2DJoint::getBox2DWorld() const
{
    if (_world.expired()) {
        throw std::logic_error("[Box2DJoint::getBox2DWorld] Can not access Box2DWorld. A joint should not exist without a world.");
    }
    return _world.lock();
}

ObjectPtr Box2DJoint::getObject() const
{
    WorldPtr world = getWorld();
    return world->getObject(_object_name, false);
}

ObjectConstPtr Box2DJoint::getConstObject() const
{
    return getObject();
}

Box2DObjectPtr Box2DJoint::getBox2DObject()
{
    auto world = getBox2DWorld();
    auto object_ptr = world->getBox2DObject(_object_name);
    if (not object_ptr) {
        auto robot_ptr = world->getBox2DRobot(_object_name);
        if (not robot_ptr) {
            throw std::logic_error("[Box2DJoint::getBox2DObject] Could not retrieve object.");
        }
        return robot_ptr->getBox2DObject();
    }
    return object_ptr;
}

void Box2DJoint::setControlTorque(float value)
{
    WorldPtr world = getWorld();
    Box2DWorldLock lock(world->getMutex());
    float scale = getBox2DWorld()->getScale();
    switch (_joint_type) {
    case JointType::Prismatic: {
        // TODO
        throw std::logic_error("setControlTorque for prismatic joints is not implemented yet");
        break;
    }
    case JointType::Revolute: {
        b2RevoluteJoint* revolute_joint = static_cast<b2RevoluteJoint*>(_joint);
        revolute_joint->EnableMotor(true);
        // TODO There is not max torque check here. We probably should have a maximum possible torque
        // TODO at least that sth we would have on real robots. At the moment this is limited in Box2DVelocityController
        // TODO using the user-specified acceleration limits.
        revolute_joint->SetMaxMotorTorque(scale * scale * std::abs(value)); // need to scale the torque to box2d world
        float sgn = value > 0.0f ? 1.0f : -1.0f;
        // This is a hack to force the motor to apply the given torque. It seems to work
        revolute_joint->SetMotorSpeed(sgn * std::numeric_limits<float>::max());
        break;
    }
    }
}

void Box2DJoint::getDOFInformation(DOFInformation& info) const
{
    info.velocity_limits = getVelocityLimits();
    info.position_limits = getPositionLimits();
    info.acceleration_limits = getAccelerationLimits();
    info.dof_index = getDOFIndex();
}

DOFInformation Box2DJoint::getDOFInformation() const
{
    DOFInformation info;
    getDOFInformation(info);
    return info;
}

Eigen::Array2f Box2DJoint::getAccelerationLimits() const
{
    return _acceleration_limits;
}

void Box2DJoint::getAccelerationLimits(Eigen::Array2f& limits) const
{
    limits = _acceleration_limits;
}

Eigen::Vector2f Box2DJoint::getAxisPosition() const
{
    WorldPtr world = getWorld();
    Box2DWorldLock lock(world->getMutex());
    Box2DLinkPtr child_link = getChildBox2DLink();
    Eigen::Vector3f child_pose;
    child_link->getPose(child_pose);
    return Eigen::Vector2f(child_pose[0], child_pose[1]);
}

b2Vec2 Box2DJoint::getLocalAxisPosition() const
{
    switch (_joint_type) {
    case JointType::Revolute: {
        b2RevoluteJoint* joint = static_cast<b2RevoluteJoint*>(_joint);
        return joint->GetLocalAnchorA();
        break;
    }
    case JointType::Prismatic: {
        //TODO
        throw std::logic_error("getLocalAxisPosition for prismatic joints is not implemented yet");
    }
    }
}

b2Vec2 Box2DJoint::getGlobalAxisPosition() const
{
    switch (_joint_type) {
    case JointType::Revolute: {
        b2RevoluteJoint* joint = static_cast<b2RevoluteJoint*>(_joint);
        return joint->GetAnchorA();
        break;
    }
    case JointType::Prismatic: {
        //TODO
        throw std::logic_error("getLocalAxisPosition for prismatic joints is not implemented yet");
    }
    }
}
////////////////////////////////////////////////////////////////////////////////
/////////////////////* Definition of Box2DObject members *//////////////////////
////////////////////////////////////////////////////////////////////////////////

Box2DObject::Box2DObject(const Box2DObjectDescription& obj_desc, Box2DWorldPtr world)
    : _destroyed(false)
{
    _name = obj_desc.name;
    _is_static = obj_desc.is_static;
    _world = Box2DWorldWeakPtr(world);
    _mass = 0.0f;
    // first create links
    for (auto& link_desc : obj_desc.links) {
        Box2DLinkPtr link(new Box2DLink(link_desc, world, _is_static, _name));
        _links[link->getName()] = link;
        _mass += link->getMass();
        _local_bounding_box.merge(link->getLocalBoundingBox());
        _num_balls += link->getNumApproximationBalls();
    }
    _base_link = _links[obj_desc.base_link];
    Eigen::Vector3f old_center;
    _base_link->getLocalCenterOfMass(old_center);
    _base_link->setOriginToCenterOfMass();
    _local_bounding_box.min_corner -= old_center;
    _local_bounding_box.max_corner -= old_center;

    unsigned int base_dofs = _is_static ? 0 : 3;
    // next create joints
    for (auto& joint_desc : obj_desc.joints) {
        Box2DLinkPtr link_a = _links[joint_desc.link_a];
        Box2DLinkPtr link_b = _links[joint_desc.link_b];
        Box2DJointPtr joint(new Box2DJoint(joint_desc, link_a, link_b, world, _name));
        link_a->registerChildJoint(joint);
        link_b->registerParentJoint(joint);
        _joints[joint->getName()] = joint;
        _sorted_joints.push_back(joint);
        joint->setJointIndex((unsigned int)(_sorted_joints.size()) - 1);
        joint->setDOFIndex(joint->getJointIndex() + base_dofs);
    }
    _num_dofs = (unsigned int)(base_dofs + _sorted_joints.size());
    _all_dof_indices.resize(_num_dofs);
    for (unsigned int i = 0; i < _num_dofs; ++i) {
        _all_dof_indices[i] = i;
    }
    // by default set all dofs active
    _active_dof_indices = getDOFIndices();
    // finally initialize all links to be placed correctly (the initial pose at the origin does not matter)
    _base_link->setPose(Eigen::Vector3f(0.0f, 0.0f, 0.0f), true, true);
    if (!_is_static) { // set dof information for base dofs
        // by default all base dofs are unbounded
        _x_dof_info.dof_index = 0;
        _x_dof_info.position_limits[0] = std::numeric_limits<float>::lowest();
        _x_dof_info.position_limits[1] = std::numeric_limits<float>::max();
        _x_dof_info.velocity_limits[0] = std::numeric_limits<float>::lowest();
        _x_dof_info.velocity_limits[1] = std::numeric_limits<float>::max();
        _x_dof_info.acceleration_limits[0] = std::numeric_limits<float>::lowest();
        _x_dof_info.acceleration_limits[1] = std::numeric_limits<float>::max();
        _y_dof_info.dof_index = 1;
        _y_dof_info.position_limits[0] = std::numeric_limits<float>::lowest();
        _y_dof_info.position_limits[1] = std::numeric_limits<float>::max();
        _y_dof_info.velocity_limits[0] = std::numeric_limits<float>::lowest();
        _y_dof_info.velocity_limits[1] = std::numeric_limits<float>::max();
        _y_dof_info.acceleration_limits[0] = std::numeric_limits<float>::lowest();
        _y_dof_info.acceleration_limits[1] = std::numeric_limits<float>::max();
        _theta_dof_info.dof_index = 2;
        _theta_dof_info.position_limits[0] = std::numeric_limits<float>::lowest();
        _theta_dof_info.position_limits[1] = std::numeric_limits<float>::max();
        _theta_dof_info.velocity_limits[0] = std::numeric_limits<float>::lowest();
        _theta_dof_info.velocity_limits[1] = std::numeric_limits<float>::max();
        _theta_dof_info.acceleration_limits[0] = std::numeric_limits<float>::lowest();
        _theta_dof_info.acceleration_limits[1] = std::numeric_limits<float>::max();
    }
}

Box2DObject::~Box2DObject()
{
    if (!_destroyed) {
        auto logger = DefaultLogger::getInstance();
        logger->logErr("[Box2DObject::~Box2DObject] Object destructor called before destroy()");
    }
}

void Box2DObject::destroy(const std::shared_ptr<b2World>& b2world)
{
    //    sim_env::DefaultLogger::getInstance()->logDebug("Destroying Box2DObject", "[sim_env::Box2DObject::destroy]");
    // Unfortunately, this can not be called in the destructor
    for (std::pair<const std::string, Box2DJointPtr>& joint_iter : _joints) {
        joint_iter.second->destroy(b2world);
    }
    _joints.clear(); // first delete all joints
    for (std::pair<const std::string, Box2DLinkPtr>& link_iter : _links) {
        link_iter.second->destroy(b2world);
    }
    _links.clear(); // then delete all links
    _destroyed = true;
}

std::string Box2DObject::getName() const
{
    return _name;
}

EntityType Box2DObject::getType() const
{
    return EntityType::Object;
}

Eigen::Affine3f Box2DObject::getTransform() const
{
    WorldPtr world = getWorld();
    Box2DWorldLock lock(world->getMutex());
    return _base_link->getTransform();
}

Eigen::Vector3f Box2DObject::getPose() const
{
    WorldPtr world = getWorld();
    Box2DWorldLock lock(world->getMutex());
    return _base_link->getPose();
}

void Box2DObject::getPose(Eigen::Vector3f& pose) const
{
    WorldPtr world = getWorld();
    Box2DWorldLock lock(world->getMutex());
    return _base_link->getPose(pose);
}

void Box2DObject::setTransform(const Eigen::Affine3f& tf)
{
    auto world = getBox2DWorld();
    Box2DWorldLock lock(world->getMutex());
    auto rotation_matrix = tf.rotation();
    float theta = (float)acos(rotation_matrix(0, 0));
    theta = rotation_matrix(1, 0) > 0.0 ? theta : -theta;
    float x = tf.translation()(0);
    float y = tf.translation()(1);
    // save current velocities (when the pose changes, body velocities need to be updated)
    Eigen::VectorXf velocities = getDOFVelocities(_all_dof_indices);
    _base_link->setPose(Eigen::Vector3f(x, y, theta));
    // restore velocities
    updateBodyVelocities(velocities);
}

WorldPtr Box2DObject::getWorld() const
{
    return getBox2DWorld();
}

WorldConstPtr Box2DObject::getConstWorld() const
{
    return getWorld();
}

Box2DWorldPtr Box2DObject::getBox2DWorld() const
{
    Box2DWorldPtr world = _world.lock();
    if (!world) {
        throw std::logic_error("[Box2DObject::getBox2DWorld] Could not access Box2DWorld. This object should not exist anymore.");
    }
    return world;
}

bool Box2DObject::checkCollision()
{
    Box2DWorldPtr world = getBox2DWorld();
    return world->checkCollision(shared_from_this());
}

bool Box2DObject::checkCollision(std::vector<Contact>& contacts)
{
    Box2DWorldPtr world = getBox2DWorld();
    return world->checkCollision(shared_from_this(), contacts);
}

bool Box2DObject::checkCollision(ObjectPtr other_object)
{
    Box2DWorldPtr world = getBox2DWorld();
    return world->checkCollision(shared_from_this(), other_object);
}

bool Box2DObject::checkCollision(ObjectPtr other_object, std::vector<Contact>& contacts)
{
    Box2DWorldPtr world = getBox2DWorld();
    return world->checkCollision(shared_from_this(), other_object, contacts);
}

bool Box2DObject::checkCollision(const std::vector<ObjectPtr>& object_list)
{
    Box2DWorldPtr world = getBox2DWorld();
    return world->checkCollision(shared_from_this(), object_list);
}

bool Box2DObject::checkCollision(const std::vector<ObjectPtr>& other_objects, std::vector<Contact>& contacts)
{
    Box2DWorldPtr world = getBox2DWorld();
    return world->checkCollision(shared_from_this(), other_objects, contacts);
}

void Box2DObject::setActiveDOFs(const Eigen::VectorXi& indices)
{
    auto world = getBox2DWorld();
    Box2DWorldLock lock(world->getMutex());
    _active_dof_indices = indices;
}

Eigen::VectorXi Box2DObject::getActiveDOFs() const
{
    return _active_dof_indices;
}

Eigen::VectorXi Box2DObject::getDOFIndices() const
{
    return _all_dof_indices;
}

Eigen::VectorXf Box2DObject::getDOFPositions(const Eigen::VectorXi& indices) const
{
    WorldPtr world = getWorld();
    Box2DWorldLock lock(world->getMutex());
    Eigen::VectorXi dofs_to_retrieve = indices;
    if (dofs_to_retrieve.size() == 0) {
        dofs_to_retrieve = _active_dof_indices;
    }
    Eigen::VectorXf output(dofs_to_retrieve.size());
    for (unsigned int i = 0; i < dofs_to_retrieve.size(); ++i) {
        int dof = dofs_to_retrieve[i];
        if (dof < 3 && not _is_static) {
            Eigen::Vector3f pose;
            _base_link->getPose(pose);
            output[i] = pose[dof];
        } else {
            int joint_index = _is_static ? dof : dof - 3;
            output[i] = _sorted_joints[joint_index]->getPosition();
        }
    }
    return output;
}

Eigen::ArrayX2f Box2DObject::getDOFPositionLimits(const Eigen::VectorXi& indices) const
{
    WorldPtr world = getWorld();
    Box2DWorldLock lock(world->getMutex());
    Eigen::VectorXi dofs_to_retrieve = indices;
    if (dofs_to_retrieve.size() == 0) {
        dofs_to_retrieve = _active_dof_indices;
    }
    Eigen::ArrayX2f limits(dofs_to_retrieve.size(), 2);
    for (int i = 0; i < dofs_to_retrieve.size(); ++i) {
        int dof = dofs_to_retrieve[i];
        assert(dof >= 0);
        DOFInformation dof_info;
        getDOFInformation((unsigned int)dof, dof_info);
        limits(i, 0) = dof_info.position_limits[0];
        limits(i, 1) = dof_info.position_limits[1];
    }
    return limits;
}

DOFInformation Box2DObject::getDOFInformation(unsigned int dof_index) const
{
    DOFInformation dof_info;
    getDOFInformation(dof_index, dof_info);
    return dof_info;
}

void Box2DObject::getDOFInformation(unsigned int dof_index, DOFInformation& info) const
{
    WorldPtr world = getWorld();
    Box2DWorldLock lock(world->getMutex());
    if (dof_index < getNumBaseDOFs()) {
        switch (dof_index) {
        case 0: {
            info = _x_dof_info;
            break;
        }
        case 1: {
            info = _y_dof_info;
            break;
        }
        case 2: {
            info = _theta_dof_info;
            break;
        }
        default: {
            throw std::logic_error("[sim_env::Box2DObject::getDOFInformation] This code should never be reached.");
        }
        }
    } else {
        JointConstPtr joint = getConstJointFromDOFIndex(dof_index);
        joint->getDOFInformation(info);
    }
}

void Box2DObject::setDOFPositions(const Eigen::VectorXf& values, const Eigen::VectorXi& indices)
{
    auto world = getBox2DWorld();
    Box2DWorldLock lock(world->getMutex());
    // first figure out what dofs to set positions for
    Eigen::VectorXi dofs_to_set(indices);
    if (dofs_to_set.size() == 0) {
        dofs_to_set = _active_dof_indices;
    }
    // TODO do we need to be sure these are sorted? Yes, they have to, but don't wanna do this check all the time
    assert(dofs_to_set.size() == values.size());
    // if (dofs_to_set.size() != values.size()) {
    //     throw std::runtime_error("[sim_env::Box2DObject::setDOFPositions] Could not set DOF positions."
    //                                      " Number of indices to set and number of provided values"
    //                                      " do not match.");
    // }
    // we need to save the dof velocities before resetting positions
    Eigen::VectorXf velocities = getDOFVelocities(_all_dof_indices);
    // treat base pose separately
    unsigned int dof_offset = 0;
    Eigen::Vector3f current_pose;
    _base_link->getPose(current_pose);
    for (unsigned int i = 0; i < std::min(getNumBaseDOFs(), (unsigned int)dofs_to_set.size()); ++i) { // run over the first three dof indices
        unsigned int dof_idx = dofs_to_set[i];
        if (dof_idx < getNumBaseDOFs()) { // if this is a base dof, update pose
            ++dof_offset;
            current_pose[dof_idx] = values[i];
        } else { // else we can quit this loop
            break; // we assume that dofs_to_set are sorted in an ascending order
        }
    }
    if (dof_offset > 0)
        _base_link->setPose(current_pose); // only need to update the pose if we actually changed it
    // now update joint positions
    for (unsigned int i = dof_offset; i < dofs_to_set.size(); ++i) {
        int dof = dofs_to_set[i];
        int joint_index = _is_static ? dof : dof - 3;
        _sorted_joints[joint_index]->resetPosition(values[i], false);
    }
    // finally reset to previous velocities given the new positions
    updateBodyVelocities(velocities);
}

Eigen::VectorXf Box2DObject::getDOFVelocities(const Eigen::VectorXi& indices) const
{
    WorldPtr world = getWorld();
    Box2DWorldLock lock(world->getMutex());
    Eigen::VectorXi dofs_to_retrieve = indices;
    if (dofs_to_retrieve.size() == 0) {
        dofs_to_retrieve = _active_dof_indices;
    }
    Eigen::VectorXf output(dofs_to_retrieve.size());
    for (unsigned int i = 0; i < dofs_to_retrieve.size(); ++i) {
        int dof = dofs_to_retrieve[i];
        if (not _is_static and dof < 3) {
            Eigen::Vector3f current_velocity;
            _base_link->getVelocityVector(current_velocity);
            output[i] = current_velocity[dof];
        } else {
            int joint_index = _is_static ? dof : dof - 3;
            output[i] = _sorted_joints[joint_index]->getVelocity();
        }
    }
    return output;
}

Eigen::ArrayX2f Box2DObject::getDOFVelocityLimits(const Eigen::VectorXi& indices) const
{
    WorldPtr world = getWorld();
    Box2DWorldLock lock(world->getMutex());
    Eigen::VectorXi dofs_to_retrieve = indices;
    if (dofs_to_retrieve.size() == 0) {
        dofs_to_retrieve = _active_dof_indices;
    }
    Eigen::ArrayX2f limits(dofs_to_retrieve.size(), 2);
    for (int i = 0; i < dofs_to_retrieve.size(); ++i) {
        int dof = dofs_to_retrieve[i];
        assert(dof >= 0);
        DOFInformation dof_info;
        getDOFInformation((unsigned int)dof, dof_info);
        limits(i, 0) = dof_info.velocity_limits[0];
        limits(i, 1) = dof_info.velocity_limits[1];
    }
    return limits;
}

Eigen::ArrayX2f Box2DObject::getDOFAccelerationLimits(const Eigen::VectorXi& indices) const
{
    WorldPtr world = getWorld();
    Box2DWorldLock lock(world->getMutex());
    Eigen::VectorXi dofs_to_retrieve = indices;
    if (dofs_to_retrieve.size() == 0) {
        dofs_to_retrieve = _active_dof_indices;
    }
    Eigen::ArrayX2f limits(dofs_to_retrieve.size(), 2);
    for (int i = 0; i < dofs_to_retrieve.size(); ++i) {
        int dof = dofs_to_retrieve[i];
        assert(dof >= 0);
        DOFInformation dof_info;
        getDOFInformation((unsigned int)dof, dof_info);
        limits(i, 0) = dof_info.acceleration_limits[0];
        limits(i, 1) = dof_info.acceleration_limits[1];
    }
    return limits;
}

void Box2DObject::setDOFVelocities(const Eigen::VectorXf& values, const Eigen::VectorXi& indices)
{
    auto world = getBox2DWorld();
    Box2DWorldLock lock(world->getMutex());
    // first determine what dofs we have to set
    Eigen::VectorXi dofs_to_set = indices;
    if (dofs_to_set.size() == 0) {
        dofs_to_set = _active_dof_indices;
    }
    if (dofs_to_set.size() != values.size()) {
        throw std::runtime_error("[sim_env::Box2DObject::setDOFVelocities] Could not set DOF velocities."
                                 " Number of indices to set and number of provided values"
                                 " do not match.");
    }
    // velocity changes affect the whole kinematic subchain, hence we need to reset everything
    Eigen::VectorXf all_velocities = getDOFVelocities(_all_dof_indices); // get all current velocities
    Eigen::ArrayX2f limits = getDOFVelocityLimits(indices);
    // now overwrite the velocities that are requested to be changed
    for (unsigned int i = 0; i < dofs_to_set.size(); ++i) {
        int dof = dofs_to_set[i];
        all_velocities[dof] = utils::math::clamp(values[i], limits(i, 0), limits(i, 1));
    }
    // update the kinematic chain
    updateBodyVelocities(all_velocities);
}

void Box2DObject::setToRest()
{
    Eigen::VectorXf vel(_num_dofs);
    vel.setZero();
    setDOFVelocities(vel, _all_dof_indices);
}

bool Box2DObject::atRest(float threshold) const
{
    WorldPtr world = getWorld();
    Box2DWorldLock lock(world->getMutex());
    Eigen::VectorXi dof_indices = getDOFIndices();
    if (dof_indices.rows() == 0)
        return true;
    Eigen::VectorXf vel = getDOFVelocities(dof_indices);
    return vel.lpNorm<Eigen::Infinity>() <= threshold;
}

void Box2DObject::setName(const std::string& name)
{
    WorldPtr world = getWorld();
    Box2DWorldLock lock(world->getMutex());
    _name = name;
    // let all links and joints know that we have a new name
    for (auto& link : _links) {
        link.second->setObjectName(name);
    }
    for (auto& joint : _joints) {
        joint.second->setObjectName(name);
    }
}

bool Box2DObject::isStatic() const
{
    return _is_static;
}

void Box2DObject::getBallApproximation(std::vector<Ball>& balls) const
{
    balls.resize(_num_balls);
    auto balls_iter = balls.begin();
    for (const auto& link_pair : _links) {
        auto link = link_pair.second;
        auto link_balls_end = balls_iter + link->getNumApproximationBalls();
        link->updateBallApproximation(balls, balls_iter, link_balls_end);
    }
}

void Box2DObject::getLinks(std::vector<LinkPtr>& links)
{
    for (auto& iter : _links) {
        links.push_back(iter.second);
    }
}

void Box2DObject::getLinks(std::vector<LinkConstPtr>& links) const
{
    for (auto& iter : _links) {
        links.push_back(iter.second);
    }
}

LinkPtr Box2DObject::getLink(const std::string& link_name)
{
    if (_links.find(link_name) != _links.end()) {
        return _links.at(link_name);
    }
    return LinkPtr(nullptr);
}

LinkConstPtr Box2DObject::getConstLink(const std::string& link_name) const
{
    if (_links.find(link_name) != _links.end()) {
        return _links.at(link_name);
    }
    return LinkConstPtr(nullptr);
}

void Box2DObject::getJoints(std::vector<JointPtr>& joints)
{
    for (auto& iter : _joints) {
        joints.push_back(iter.second);
    }
}

void Box2DObject::getBox2DJoints(std::vector<Box2DJointPtr>& joints)
{
    for (auto& iter : _joints) {
        joints.push_back(iter.second);
    }
}

void Box2DObject::getJoints(std::vector<JointConstPtr>& joints) const
{
    for (auto& iter : _joints) {
        joints.push_back(iter.second);
    }
}

void Box2DObject::getBox2DJoints(std::vector<Box2DJointConstPtr>& joints) const
{
    for (auto& iter : _joints) {
        joints.push_back(iter.second);
    }
}

JointPtr Box2DObject::getJoint(const std::string& joint_name)
{
    if (_joints.find(joint_name) != _joints.end()) {
        return _joints.at(joint_name);
    }
    return JointPtr(nullptr);
}

JointConstPtr Box2DObject::getConstJoint(const std::string& joint_name) const
{
    if (_joints.find(joint_name) != _joints.end()) {
        return _joints.at(joint_name);
    }
    return JointConstPtr(nullptr);
}

LinkPtr Box2DObject::getBaseLink()
{
    return _base_link;
}

Box2DLinkPtr Box2DObject::getBox2DBaseLink()
{
    return _base_link;
}

unsigned int Box2DObject::getNumDOFs() const
{
    return _num_dofs;
}

unsigned int Box2DObject::getNumActiveDOFs() const
{
    WorldPtr world = getWorld();
    Box2DWorldLock lock(world->getMutex());
    Eigen::VectorXi active_dofs = getActiveDOFs();
    return (unsigned int)active_dofs.size();
}

Box2DJointPtr Box2DObject::getBox2DJoint(unsigned int idx)
{
    return _sorted_joints.at(idx);
}

unsigned int Box2DObject::getNumBaseDOFs() const
{
    return _is_static ? 0 : 3;
}

JointPtr Box2DObject::getJoint(unsigned int joint_idx)
{
    if (joint_idx >= _sorted_joints.size()) {
        return JointPtr(nullptr);
    }
    return _sorted_joints.at(joint_idx);
}

JointPtr Box2DObject::getJointFromDOFIndex(unsigned int dof_idx)
{
    if (dof_idx < getNumBaseDOFs()) {
        return JointPtr(nullptr);
    }
    return getJoint(dof_idx - getNumBaseDOFs());
}

JointConstPtr Box2DObject::getConstJoint(unsigned int joint_idx) const
{
    if (joint_idx >= _sorted_joints.size()) {
        return JointConstPtr(nullptr);
    }
    return _sorted_joints.at(joint_idx);
}

JointConstPtr Box2DObject::getConstJointFromDOFIndex(unsigned int dof_idx) const
{
    if (dof_idx < getNumBaseDOFs()) {
        return JointPtr(nullptr);
    }
    return getConstJoint(dof_idx - getNumBaseDOFs());
}

float Box2DObject::getInertia() const
{
    WorldPtr world = getWorld();
    Box2DWorldLock lock(world->getMutex());
    Eigen::Vector2f com = _base_link->getCenterOfMass();
    float inertia = 0.0f;
    for (auto& name_link_pair : _links) {
        Box2DLinkPtr link = name_link_pair.second;
        float distance = (link->getCenterOfMass() - com).norm();
        inertia += link->getInertia() + distance * distance * link->getMass();
    }
    return inertia;
}

float Box2DObject::getMass() const
{
    return _mass;
}

BoundingBox Box2DObject::getLocalAABB() const
{
    return _local_bounding_box;
}

float Box2DObject::getGroundFriction() const
{
    return _base_link->getGroundFriction();
}

void Box2DObject::getBox2DLinks(std::vector<Box2DLinkPtr>& links)
{
    for (auto& name_link_pair : _links) {
        links.push_back(name_link_pair.second);
    }
}

void Box2DObject::setPose(float x, float y, float theta)
{
    auto world = getBox2DWorld();
    Box2DWorldLock lock(world->getMutex());
    Eigen::VectorXf vel = getDOFVelocities(_all_dof_indices);
    _base_link->setPose(Eigen::Vector3f(x, y, theta), true);
    updateBodyVelocities(vel);
}

void Box2DObject::setPose(const Eigen::Vector3f& pose)
{
    setPose(pose[0], pose[1], pose[2]);
}

void Box2DObject::getBodies(std::vector<b2Body*>& bodies)
{
    for (auto& link : _links) {
        Box2DLinkPtr box2d_link = link.second;
        box2d_link->getBodies(bodies);
    }
}

void Box2DObject::getState(ObjectState& object_state) const
{
    WorldPtr world = getWorld();
    Box2DWorldLock lock(world->getMutex());
    object_state.active_dofs = _active_dof_indices;
    Eigen::VectorXi all_indices = getDOFIndices();
    object_state.dof_positions = getDOFPositions(all_indices);
    object_state.dof_velocities = getDOFVelocities(all_indices);
    object_state.pose = getTransform();
}

ObjectState Box2DObject::getState() const
{
    ObjectState object_state;
    getState(object_state);
    return object_state;
}

void Box2DObject::setState(const ObjectState& object_state)
{
    WorldPtr world = getWorld();
    Box2DWorldLock lock(world->getMutex());
    _active_dof_indices = object_state.active_dofs;
    setTransform(object_state.pose);
    Eigen::VectorXi all_indices = getDOFIndices();
    setDOFPositions(object_state.dof_positions, all_indices);
    setDOFVelocities(object_state.dof_velocities, all_indices);
}

void Box2DObject::setEnabled(bool b_enable)
{
    for (auto link : _links) {
        link.second->setEnabled(b_enable);
    }
}

bool Box2DObject::isEnabled() const
{
    bool enabled = false;
    for (auto link : _links) {
        enabled |= link.second->isEnabled();
    }
    return enabled;
}

LinkPtr Box2DObject::getLink(b2Body* body)
{
    //TODO maybe make this more efficient using a map
    for (auto link_iter : _links) {
        if (link_iter.second->getBody() == body) {
            return link_iter.second;
        }
    }
    return sim_env::LinkPtr(nullptr);
}

void Box2DObject::updateBodyVelocities(const Eigen::VectorXf& all_dof_velocities)
{
    Eigen::VectorXf velocities = all_dof_velocities;
    unsigned int index_offset = 0;
    if (_is_static) { // in case the base link is static, we need to shift the velocity vector
        velocities = Eigen::VectorXf(all_dof_velocities.size() + 3);
        // to contain fake base velocities
        velocities[0] = 0.0f;
        velocities[1] = 0.0f;
        velocities[2] = 0.0f;
        for (unsigned int i = 3; i < velocities.size(); ++i) {
            velocities[i] = all_dof_velocities[i - 3];
        }
        index_offset = 3;
    }
    // add the mobile base as a fake joint (that rotates around the base link's center)
    std::vector<std::pair<b2Vec2, unsigned int>> parent_joints;
    Eigen::Vector3f pose;
    _base_link->getPose(pose);
    float scale = getBox2DWorld()->getScale();
    auto pair = std::make_pair(b2Vec2(scale * pose[0], scale * pose[1]), 2); // this models this fake joint
    parent_joints.push_back(pair);
    _base_link->updateBodyVelocities(velocities, parent_joints, index_offset);
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////* Definition of Box2DRobot members *///////////////////////
////////////////////////////////////////////////////////////////////////////////
Box2DRobot::Box2DRobot(const Box2DRobotDescription& robot_desc, Box2DWorldPtr world)
    : _destroyed(false)
{
    // A robot is essentially an actuated object. Rather than implementing
    // the same functionalities twice (for object and robot), we use composition here.
    // Also we do not wanna have diamond inheritance
    _robot_object = Box2DObjectPtr(new Box2DObject(robot_desc.object_description, world));
    if (robot_desc.use_center_of_mass) {
        _robot_object->_base_link->setOriginToCenterOfMass();
    }
    if (not isStatic()) {
        // info on robot x dof
        _robot_object->_x_dof_info.velocity_limits = robot_desc.translational_velocity_limits;
        _robot_object->_x_dof_info.acceleration_limits = robot_desc.translational_acceleration_limits;
        // info on robot y dof
        _robot_object->_y_dof_info.velocity_limits = robot_desc.translational_velocity_limits;
        _robot_object->_y_dof_info.acceleration_limits = robot_desc.translational_acceleration_limits;
        // info on robot orientation dof
        _robot_object->_theta_dof_info.velocity_limits = robot_desc.rotational_velocity_limits;
        _robot_object->_theta_dof_info.acceleration_limits = robot_desc.rotational_acceleration_limits;
    }
}

Box2DRobot::~Box2DRobot()
{
    if (not _destroyed) {
        auto logger = DefaultLogger::getInstance();
        logger->logErr("[Box2DRobot::~Box2DRobot] Object destructor called before destroy()");
    }
}

void Box2DRobot::destroy(const std::shared_ptr<b2World>& b2world)
{
    //    sim_env::DefaultLogger::getInstance()->logDebug("Destroying Box2DRobot", "[sim_env::Box2DRobot::destroy]");
    _robot_object->destroy(b2world);
    _destroyed = true;
}

std::string Box2DRobot::getName() const
{
    return _robot_object->getName();
}

void Box2DRobot::setName(const std::string& name)
{
    _robot_object->setName(name);
}

void Box2DRobot::setTransform(const Eigen::Affine3f& tf)
{
    _robot_object->setTransform(tf);
}

void Box2DRobot::setActiveDOFs(const Eigen::VectorXi& indices)
{
    _robot_object->setActiveDOFs(indices);
}

Eigen::VectorXi Box2DRobot::getActiveDOFs() const
{
    return _robot_object->getActiveDOFs();
}

Eigen::VectorXi Box2DRobot::getDOFIndices() const
{
    return _robot_object->getDOFIndices();
}

Eigen::VectorXf Box2DRobot::getDOFPositions(const Eigen::VectorXi& indices) const
{
    return _robot_object->getDOFPositions(indices);
}

Eigen::ArrayX2f Box2DRobot::getDOFPositionLimits(const Eigen::VectorXi& indices) const
{
    return _robot_object->getDOFPositionLimits(indices);
}

void Box2DRobot::setDOFPositions(const Eigen::VectorXf& values, const Eigen::VectorXi& indices)
{
    _robot_object->setDOFPositions(values, indices);
}

Eigen::VectorXf Box2DRobot::getDOFVelocities(const Eigen::VectorXi& indices) const
{
    return _robot_object->getDOFVelocities(indices);
}

Eigen::ArrayX2f Box2DRobot::getDOFVelocityLimits(const Eigen::VectorXi& indices) const
{
    return _robot_object->getDOFVelocityLimits(indices);
}

Eigen::ArrayX2f Box2DRobot::getDOFAccelerationLimits(const Eigen::VectorXi& indices) const
{
    return _robot_object->getDOFAccelerationLimits(indices);
}

void Box2DRobot::setToRest()
{
    _robot_object->setToRest();
}

void Box2DRobot::setDOFVelocities(const Eigen::VectorXf& values, const Eigen::VectorXi& indices)
{
    _robot_object->setDOFVelocities(values, indices);
}

bool Box2DRobot::isStatic() const
{
    return _robot_object->isStatic();
}

void Box2DRobot::getBallApproximation(std::vector<Ball>& balls) const
{
    _robot_object->getBallApproximation(balls);
}

bool Box2DRobot::atRest(float threshold) const
{
    return _robot_object->atRest(threshold);
}

bool Box2DRobot::checkCollision()
{
    return _robot_object->checkCollision();
}

bool Box2DRobot::checkCollision(std::vector<Contact>& contacts)
{
    return _robot_object->checkCollision(contacts);
}

bool Box2DRobot::checkCollision(const std::vector<ObjectPtr>& others)
{
    return _robot_object->checkCollision(others);
}

bool Box2DRobot::checkCollision(ObjectPtr other_object)
{
    return _robot_object->checkCollision(other_object);
}

bool Box2DRobot::checkCollision(ObjectPtr other_object, std::vector<Contact>& contacts)
{
    return _robot_object->checkCollision(other_object, contacts);
}

bool Box2DRobot::checkCollision(const std::vector<ObjectPtr>& other_objects, std::vector<Contact>& contacts)
{
    return _robot_object->checkCollision(other_objects, contacts);
}

EntityType Box2DRobot::getType() const
{
    return EntityType::Robot;
}

Eigen::Affine3f Box2DRobot::getTransform() const
{
    return _robot_object->getTransform();
}

WorldPtr Box2DRobot::getWorld() const
{
    return _robot_object->getWorld();
}

WorldConstPtr Box2DRobot::getConstWorld() const
{
    return _robot_object->getConstWorld();
}

void Box2DRobot::getLinks(std::vector<LinkPtr>& links)
{
    _robot_object->getLinks(links);
}

void Box2DRobot::getLinks(std::vector<LinkConstPtr>& links) const
{
    _robot_object->getLinks(links);
}

void Box2DRobot::getJoints(std::vector<JointPtr>& joints)
{
    _robot_object->getJoints(joints);
}

void Box2DRobot::getBox2DJoints(std::vector<Box2DJointPtr>& joints)
{
    _robot_object->getBox2DJoints(joints);
}

void Box2DRobot::getJoints(std::vector<JointConstPtr>& joints) const
{
    _robot_object->getJoints(joints);
}

void Box2DRobot::getBox2DJoints(std::vector<Box2DJointConstPtr>& joints) const
{
    _robot_object->getBox2DJoints(joints);
}

LinkPtr Box2DRobot::getLink(const std::string& link_name)
{
    return _robot_object->getLink(link_name);
}

LinkConstPtr Box2DRobot::getConstLink(const std::string& link_name) const
{
    return _robot_object->getConstLink(link_name);
}

JointPtr Box2DRobot::getJoint(const std::string& joint_name)
{
    return _robot_object->getJoint(joint_name);
}

JointConstPtr Box2DRobot::getConstJoint(const std::string& joint_name) const
{
    return _robot_object->getConstJoint(joint_name);
}

LinkPtr Box2DRobot::getBaseLink()
{
    return _robot_object->getBaseLink();
}

unsigned int Box2DRobot::getNumDOFs() const
{
    return _robot_object->getNumDOFs();
}

void Box2DRobot::control(float timestep)
{
    // First lock the controller to ensure no other thread removes it while we do a control
    std::lock_guard<std::recursive_mutex> lock(_controller_mutex);
    // Only do sth if there is actually a controller registered
    if (not _controller_callback) {
        return;
    }
    // also lock the world
    WorldPtr world = getWorld();
    Box2DWorldLock world_lock(world->getMutex());
    // Get positions of currently active DoFs
    Eigen::VectorXf positions = getDOFPositions();
    // Get velocities of currently active DoFs
    Eigen::VectorXf velocities = getDOFVelocities();
    assert(velocities.size() == positions.size());
    // Create efforts array
    Eigen::VectorXf efforts(velocities.size());
    bool success = _controller_callback(positions, velocities, timestep, shared_from_this(), efforts);
    if (success) { // we have a control to apply
        assert(efforts.size() == positions.size());
        commandEfforts(efforts);
    } else { // else we don't do anything apart from logging
        LoggerPtr logger = getWorld()->getLogger();
        logger->logWarn("Controller indicated failure. Not applying any efforts.", "[sim_env::Box2DRobot::control]");
    }
}

void Box2DRobot::setController(ControlCallback controll_fn)
{
    // First lock the controller to ensure we do not replace the controller while it's operating
    std::lock_guard<std::recursive_mutex> lock(_controller_mutex);
    _controller_callback = controll_fn;
}

void Box2DRobot::commandEfforts(const Eigen::VectorXf& target)
{
    // we are changing the state of Box2D objects here, so we should lock the world
    Box2DWorldLock lock(_robot_object->getBox2DWorld()->getMutex());
    //    LoggerPtr logger = getWorld()->getLogger();
    Eigen::VectorXi active_dofs = _robot_object->getActiveDOFs();
    float scale = _robot_object->getBox2DWorld()->getScale();
    //    std::stringstream ss;
    //    ss << "Commanding efforts " << target.transpose();
    //    logger->logDebug(ss.str());
    for (int i = 0; i < active_dofs.size(); ++i) {
        int dof_idx = active_dofs[i];
        // check whether we have to move the base link
        if (dof_idx < getNumBaseDOFs()) {
            Box2DLinkPtr base_link = _robot_object->getBox2DBaseLink();
            b2Body* body = base_link->getBody();
            switch (dof_idx) {
            case 0: {
                // force in x
                body->ApplyForceToCenter(b2Vec2(scale * target[i], 0.0f), true); // force is N = kg * m / s^2
                break;
            }
            case 1: {
                // force in y
                body->ApplyForceToCenter(b2Vec2(0.0f, scale * target[i]), true);
                break;
            }
            case 2: {
                // torque
                body->ApplyTorque(scale * scale * target[i], true); // torque is in Nm = kg * m / s^2 * m
                break;
            }
            default: {
                // impossible case
                throw std::logic_error("[sim_env::Box2DRobot::commandEfforts] Encountered an impossible state.");
            }
            }
        } else {
            // torque/effort for joint
            int joint_idx = dof_idx - getNumBaseDOFs();
            assert(joint_idx >= 0);
            Box2DJointPtr joint = _robot_object->getBox2DJoint((unsigned int)joint_idx);
            joint->setControlTorque(target[i]);
        }
    }
}

unsigned int Box2DRobot::getNumActiveDOFs() const
{
    return _robot_object->getNumActiveDOFs();
}

unsigned int Box2DRobot::getNumBaseDOFs() const
{
    return _robot_object->getNumBaseDOFs();
}

JointPtr Box2DRobot::getJoint(unsigned int joint_idx)
{
    return _robot_object->getJoint(joint_idx);
}

JointPtr Box2DRobot::getJointFromDOFIndex(unsigned int dof_idx)
{
    return _robot_object->getJointFromDOFIndex(dof_idx);
}

JointConstPtr Box2DRobot::getConstJoint(unsigned int joint_idx) const
{
    return _robot_object->getConstJoint(joint_idx);
}

JointConstPtr Box2DRobot::getConstJointFromDOFIndex(unsigned int dof_idx) const
{
    return _robot_object->getConstJointFromDOFIndex(dof_idx);
}

DOFInformation Box2DRobot::getDOFInformation(unsigned int dof_index) const
{
    return _robot_object->getDOFInformation(dof_index);
}

void Box2DRobot::getDOFInformation(unsigned int dof_index, DOFInformation& info) const
{
    _robot_object->getDOFInformation(dof_index, info);
}

float Box2DRobot::getMass() const
{
    return _robot_object->getMass();
}

float Box2DRobot::getInertia() const
{
    return _robot_object->getInertia();
}

BoundingBox Box2DRobot::getLocalAABB() const
{
    return _robot_object->getLocalAABB();
}

float Box2DRobot::getGroundFriction() const
{
    return _robot_object->getGroundFriction();
}

void Box2DRobot::getBox2DLinks(std::vector<Box2DLinkPtr>& links)
{
    _robot_object->getBox2DLinks(links);
}

Box2DLinkPtr Box2DRobot::getBox2DBaseLink()
{
    return _robot_object->getBox2DBaseLink();
}

void Box2DRobot::getBodies(std::vector<b2Body*>& bodies)
{
    _robot_object->getBodies(bodies);
}

void Box2DRobot::getState(ObjectState& object_state) const
{
    _robot_object->getState(object_state);
}

ObjectState Box2DRobot::getState() const
{
    return _robot_object->getState();
}

void Box2DRobot::setState(const ObjectState& object_state)
{
    _robot_object->setState(object_state);
}

LinkPtr Box2DRobot::getLink(b2Body* body)
{
    return _robot_object->getLink(body);
}

Box2DObjectPtr Box2DRobot::getBox2DObject()
{
    return _robot_object;
}

Eigen::Vector3f Box2DRobot::getPose() const
{
    return _robot_object->getPose();
}

void Box2DRobot::getPose(Eigen::Vector3f& pose) const
{
    _robot_object->getPose(pose);
}

void Box2DRobot::setPose(const Eigen::Vector3f& pose)
{
    _robot_object->setPose(pose);
}

void Box2DRobot::setPose(float x, float y, float theta)
{
    _robot_object->setPose(x, y, theta);
}

void Box2DRobot::setEnabled(bool b_enable)
{
    _robot_object->setEnabled(b_enable);
}

bool Box2DRobot::isEnabled() const
{
    return _robot_object->isEnabled();
}

///////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////* Definition of Box2DContactListenerDistributor members */////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
// Box2DContactListenerDistributor::Box2DContactListenerDistributor() = default;

// Box2DContactListenerDistributor::~Box2DContactListenerDistributor() = default;

// void Box2DContactListenerDistributor::BeginContact(b2Contact *contact) {
//     for (auto listener : listeners) {
//         listener->BeginContact(contact);
//     }
// }

// void Box2DContactListenerDistributor::EndContact(b2Contact *contact) {
//     for (auto listener : listeners) {
//         listener->EndContact(contact);
//     }
// }

// void Box2DContactListenerDistributor::PreSolve(b2Contact *contact, const b2Manifold *oldManifold) {
//     for (auto listener : listeners) {
//         listener->PreSolve(contact, oldManifold);
//     }
// }

// void Box2DContactListenerDistributor::PostSolve(b2Contact *contact, const b2ContactImpulse *impulse) {
//     for (auto listener : listeners) {
//         listener->PostSolve(contact, impulse);
//     }
// }
///////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////* Definition of Box2DCollisionChecker members *///////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
Box2DCollisionChecker::Box2DCollisionChecker(Box2DWorldPtr world)
{
    _weak_world = world;
    _scale = world->getInverseScale();
    _cache_invalid = true;
    _b_record_contacts = false;
}

Box2DCollisionChecker::~Box2DCollisionChecker()
{
}

bool Box2DCollisionChecker::checkCollision(Box2DCollidablePtr collidable_a, Box2DCollidablePtr collidable_b)
{
    std::vector<Box2DCollidablePtr> collidables;
    collidables.push_back(collidable_b);
    return checkCollision(collidable_a, collidables);
}

bool Box2DCollisionChecker::checkCollision(Box2DCollidablePtr collidable_a, Box2DCollidablePtr collidable_b,
    std::vector<Contact>& contacts)
{
    std::vector<Box2DCollidablePtr> collidables;
    collidables.push_back(collidable_b);
    return checkCollision(collidable_a, collidables, contacts);
}

bool Box2DCollisionChecker::checkCollision(Box2DCollidablePtr collidable)
{
    WorldPtr world = getWorld();
    Box2DWorldLock lock(world->getMutex());
    updateContactCache();
    std::vector<b2Body*> bodies;
    collidable->getBodies(bodies);
    for (b2Body* body : bodies) {
        if (hasContacts(body)) {
            return true;
        }
    }
    return false;
}

bool Box2DCollisionChecker::checkCollision(Box2DCollidablePtr collidable, std::vector<Contact>& contacts)
{
    bool is_in_collision = false;
    WorldPtr world = getWorld();
    Box2DWorldLock lock(world->getMutex());
    // compute contacts first
    updateContactCache();
    std::vector<b2Body*> bodies;
    collidable->getBodies(bodies);
    // run over all bodies of this collidable
    for (b2Body* body : bodies) {
        LinkPtr link_a = getLink(body, collidable);
        ObjectPtr object_a = link_a->getObject();
        // get the contact maps for this body
        auto maps_iter = _body_contact_maps.find(body);
        // check whether there are any contacts for the current body
        if (maps_iter != _body_contact_maps.end() and not maps_iter->second.empty()) {
            ContactMap& contact_map = maps_iter->second;
            is_in_collision = true;
            // run over the other bodies that this body collides with
            for (auto contact_map_iter : contact_map) {
                b2Body* other_body = contact_map_iter.first;
                // run over all contacts that these two bodies have
                for (auto& contact : contact_map_iter.second) {
                    contact.link_a = link_a;
                    contact.object_a = object_a;
                    LinkPtr link = getLink(other_body, nullptr);
                    if (!link) {
                        throw std::logic_error("[sim_env::Box2DCollisionChecker::checkCollision] Could not retrieve link for a colliding Box2D body.");
                    }
                    contact.link_b = link;
                    contact.object_b = link->getObject();
                    contacts.push_back(contact);
                }
            }
        }
    }
    return is_in_collision;
}

bool Box2DCollisionChecker::checkCollision(Box2DCollidablePtr collidable_a, const std::vector<Box2DCollidablePtr>& collidables)
{
    WorldPtr world = getWorld();
    Box2DWorldLock lock(world->getMutex());
    // compute contacts
    updateContactCache();
    // we only need to get the bodies of collidable a once
    std::vector<b2Body*> bodies_a;
    collidable_a->getBodies(bodies_a);
    // now we need to run over each collidable in the list and check for collisions
    for (Box2DCollidablePtr collidable_b : collidables) {
        // now check if the specified collidables collide
        std::vector<b2Body*> bodies_b;
        collidable_b->getBodies(bodies_b);
        // now run over each body pair and check whether it is colliding
        for (b2Body* body_a : bodies_a) {
            auto iter_a = _body_contact_maps.find(body_a);
            if (iter_a != _body_contact_maps.end() and not iter_a->second.empty()) {
                // body_a has some contacts
                ContactMap& body_a_contacts = iter_a->second;
                // let's check whether any of b's bodies is in contact with this body
                for (b2Body* body_b : bodies_b) {
                    auto iter_b = body_a_contacts.find(body_b);
                    // if yes return
                    if (iter_b != body_a_contacts.end()) {
                        return true;
                    }
                }
            }
        }
    }
    // if we get here, there is no collision
    return false;
}

bool Box2DCollisionChecker::checkCollision(Box2DCollidablePtr collidable_a,
    const std::vector<Box2DCollidablePtr>& collidables,
    std::vector<Contact>& contacts)
{
    WorldPtr world = getWorld();
    Box2DWorldLock lock(world->getMutex());
    bool is_in_collision = false;
    // compute contacts
    updateContactCache();
    // we only need to get the bodies of collidable a once
    std::vector<b2Body*> bodies_a;
    collidable_a->getBodies(bodies_a);
    // now we need to run over each collidable in the list and check for collisions
    for (const Box2DCollidablePtr& collidable_b : collidables) {
        // now check if the specified collidables collide
        std::vector<b2Body*> bodies_b;
        collidable_b->getBodies(bodies_b);
        // now run over each body pair and check whether it is colliding
        for (b2Body* body_a : bodies_a) {
            LinkPtr link_a = collidable_a->getLink(body_a);
            ObjectPtr object_a = link_a->getObject();
            auto iter_a = _body_contact_maps.find(body_a);
            if (iter_a != _body_contact_maps.end() and not iter_a->second.empty()) {
                // body_a has some contacts
                ContactMap& body_a_contacts = iter_a->second;
                // let's check whether any of b's bodies is in contact with this body
                for (b2Body* body_b : bodies_b) {
                    auto iter_b = body_a_contacts.find(body_b);
                    // if yes add the contacts
                    if (iter_b != body_a_contacts.end()) {
                        is_in_collision = true;
                        for (auto& contact : iter_b->second) {
                            contact.object_a = object_a;
                            contact.link_a = link_a;
                            LinkPtr link_b = getLink(body_b, collidable_b);
                            contact.link_b = link_b;
                            contact.object_b = link_b->getObject();
                            contacts.push_back(contact);
                        }
                    }
                }
            }
        }
    }
    return is_in_collision;
}

void Box2DCollisionChecker::startRecordings()
{
    static const std::string log_prefix("[Box2DCollisionChecker::startRecordings]");
    Box2DWorldPtr world = _weak_world.lock();
    if (!world) {
        throw std::logic_error(log_prefix + "Could not acquire access to Box2DWorld. This object should not exist anymore!");
    }
    // lock the world
    Box2DWorldLock lock(world->getMutex());
    _registered_contacts.clear();
    _b_record_contacts = false;
    // initialize registered contacts with currently existing contacts
    world->saveState();
    world->setToRest();
    world->stepPhysics(1, false, false); // need to do this so that contact list gets updated
    _b_record_contacts = true;
    auto box2d_world = world->getRawBox2DWorld();
    auto contact = box2d_world->GetContactList();
    while (contact) {
        if (contact->IsTouching() && contact->IsEnabled()) {
            BeginContact(contact);
        }
        contact = contact->GetNext();
    }
    world->restoreState();
}

void Box2DCollisionChecker::getRecordedContacts(std::vector<Contact>& contacts)
{
    for (auto contact_record : _registered_contacts) {
        auto fixture_a = std::get<0>(contact_record);
        auto fixture_b = std::get<1>(contact_record);
        auto contact = std::get<2>(contact_record);
        if (!contact.link_a.lock()) {
            contact.link_a = getLink(fixture_a->GetBody());
        }
        if (!contact.link_b.lock()) {
            contact.link_b = getLink(fixture_b->GetBody());
        }
        if (!contact.object_a.lock()) {
            contact.object_a = (contact.link_a.lock())->getObject();
        }
        if (!contact.object_b.lock()) {
            contact.object_b = (contact.link_b.lock())->getObject();
        }
        contacts.push_back(contact);
    }
}

void Box2DCollisionChecker::BeginContact(b2Contact* contact)
{
    if (!_b_record_contacts)
        return;
    b2Fixture* fixture_a = contact->GetFixtureA();
    b2Fixture* fixture_b = contact->GetFixtureB();
    b2WorldManifold world_manifold;
    contact->GetWorldManifold(&world_manifold);
    Contact new_contact;
    // contact point
    new_contact.contact_point[0] = _scale * world_manifold.points[0].x;
    new_contact.contact_point[1] = _scale * world_manifold.points[0].y;
    new_contact.contact_point[2] = 0.0f;
    // contact normal
    new_contact.contact_normal[0] = _scale * world_manifold.normal.x;
    new_contact.contact_normal[1] = _scale * world_manifold.normal.y;
    new_contact.contact_normal[2] = 0.0f;
    // add the recording
    _registered_contacts.push_back(std::make_tuple(fixture_a, fixture_b, new_contact));
}

void Box2DCollisionChecker::EndContact(b2Contact* contact)
{
}

void Box2DCollisionChecker::PreSolve(b2Contact* contact, const b2Manifold* oldManifold)
{
}

void Box2DCollisionChecker::PostSolve(b2Contact* contact, const b2ContactImpulse* impulse)
{
}

void Box2DCollisionChecker::updateContactMaps(b2Body* body_a, b2Body* body_b, b2Contact* contact)
{
    // Start assumption: contact is not stored in cache yet
    // first ensure we have a contacts map for body a
    auto body_contact_map_iter = _body_contact_maps.find(body_a);
    // if we don't have a body map for body a yet, add it
    if (body_contact_map_iter == _body_contact_maps.end()) {
        auto insert_result = _body_contact_maps.insert(std::pair<b2Body*, ContactMap>(body_a, ContactMap()));
        // insert result should be (iter, bool) with iter pointing at the ContactMap for body_a
        assert(insert_result.first != _body_contact_maps.end());
        body_contact_map_iter = insert_result.first;
    }
    // get reference to contact map for body a
    ContactMap& body_a_contacts = body_contact_map_iter->second;
    // now check whether we have body_b already in the contact list
    auto body_a_contacts_iter = body_a_contacts.find(body_b);
    if (body_a_contacts_iter == body_a_contacts.end()) { // if not add contact list
        auto insert_result = body_a_contacts.insert(std::make_pair(body_b, std::vector<Contact>()));
        body_a_contacts_iter = insert_result.first;
    }
    // next we can add this contact to our cache
    b2WorldManifold world_manifold;
    contact->GetWorldManifold(&world_manifold);
    Contact new_contact;
    // contact point
    new_contact.contact_point[0] = _scale * world_manifold.points[0].x;
    new_contact.contact_point[1] = _scale * world_manifold.points[0].y;
    new_contact.contact_point[2] = 0.0f;
    // contact normal
    new_contact.contact_normal[0] = _scale * world_manifold.normal.x;
    new_contact.contact_normal[1] = _scale * world_manifold.normal.y;
    new_contact.contact_normal[2] = 0.0f;
    // we get the object ptr and link ptr later
    body_a_contacts_iter->second.push_back(new_contact);
    // since body_a_contacts is a reference, we are done
}

bool Box2DCollisionChecker::areInContact(b2Body* body_a, b2Body* body_b) const
{
    auto iter_a = _body_contact_maps.find(body_a);
    if (iter_a == _body_contact_maps.end()) {
        return false;
    }
    const ContactMap& body_a_contacts = iter_a->second;
    auto iter_b = body_a_contacts.find(body_b);
    return iter_b != body_a_contacts.end();
}

void Box2DCollisionChecker::updateContactCache()
{
    static const std::string log_prefix("[sim_env::Box2DCollisionChecker::updateContactCache]");
    if (not _cache_invalid) {
        return;
    }
    Box2DWorldPtr world = _weak_world.lock();
    if (!world) {
        throw std::logic_error(log_prefix + "Could not acquire access to Box2DWorld. This object should not exist anymore!");
    }
    // world->getLogger()->logDebug("Collision cache invalid! Forward propagating world for collision checks",
    //                              log_prefix);
    Box2DWorldLock lock(world->getMutex());
    _body_contact_maps.clear();
    world->saveState();
    world->setToRest();
    world->stepPhysics(2, false, false); // for some reason we need to propagate twice
    auto box2d_world = world->getRawBox2DWorld();
    auto contact = box2d_world->GetContactList();
    while (contact) {
        if (contact->IsTouching() && contact->IsEnabled()) {
            b2Body* body_a = contact->GetFixtureA()->GetBody();
            b2Body* body_b = contact->GetFixtureB()->GetBody();
            updateContactMaps(body_a, body_b, contact);
            updateContactMaps(body_b, body_a, contact);
        }
        contact = contact->GetNext();
    }
    world->restoreState();
    _cache_invalid = false;
}

bool Box2DCollisionChecker::hasContacts(b2Body* body) const
{
    auto iter = _body_contact_maps.find(body);
    if (iter == _body_contact_maps.end()) {
        return false;
    }
    return not iter->second.empty();
}

LinkPtr Box2DCollisionChecker::getLink(b2Body* body, Box2DCollidablePtr collidable)
{
    // we need access to the box2d world
    Box2DWorldPtr world = _weak_world.lock();
    if (!world) {
        throw std::logic_error("[sim_env::Box2DCollisionChecker::checkCollision] Could not access Box2DWorld");
    }
    LoggerPtr logger = world->getLogger();

    // first check whether we have the link already in our cache
    auto iter = _body_to_link_map.find(body);
    if (iter != _body_to_link_map.end()) {
        return iter->second.lock();
    }
    // else check whether the collidable can be used to retrieve it
    if (collidable) {
        LinkPtr link = collidable->getLink(body);
        if (link) {
            _body_to_link_map[body] = link;
            return link;
        } else {
            logger->logWarn("Collidable could not provide us with a link for the given body.",
                "[sim_env::Box2DCollisionChecker::getLink]");
        }
    }
    // as a last resort ask the world to search for a link
    LinkPtr link = world->getLink(body);
    if (link) {
        _body_to_link_map[body] = link;
        return link;
    }
    logger->logWarn("Could not retrieve link for the provided body",
        "[sim_env::Box2DCollisionChecker::getLink");
    return nullptr;
}

Box2DWorldPtr Box2DCollisionChecker::getWorld()
{
    Box2DWorldPtr world = _weak_world.lock();
    if (!world) {
        throw std::logic_error("[sim_env::Box2DCollisionChecker::checkCollision] Could not access Box2DWorld");
    }
    return world;
}

void Box2DCollisionChecker::invalidateCache()
{
    _cache_invalid = true;
}
////////////////////////////////////////////////////////////////////////////////
/////////////////////* Definition of Box2DAABBQuerier members */////////////////
////////////////////////////////////////////////////////////////////////////////
struct Box2DAABBQuery : public b2QueryCallback {
    std::unordered_set<std::string> object_hits;
    std::unordered_set<std::string> link_hits;
    b2Transform query_transform;
    b2PolygonShape query_box;
    bool ReportFixture(b2Fixture* fixture)
    {
        auto body = fixture->GetBody();
        auto user_data = static_cast<Box2DLink::Box2DBodyUserData*>(body->GetUserData());
        if (user_data->b_collision_ignore) { // bodies like ground body that are not to be considered
            return true;
        }
        if (not b2TestOverlap(&query_box, 0, fixture->GetShape(), 0, query_transform, body->GetTransform())) {
            return true;
        }
        object_hits.insert(user_data->object_name);
        link_hits.insert(user_data->link_name);
        return true;
    }
};

////////////////////////////////////////////////////////////////////////////////
/////////////////////* Definition of Box2DWorld members *///////////////////////
////////////////////////////////////////////////////////////////////////////////

Box2DWorld::Box2DWorld()
    : _world(nullptr)
    , _b2_ground_body(nullptr)
    , _collision_checker(nullptr)
    , _time_step(0.01f)
    , _velocity_steps(15)
    , _position_steps(15)
    , _logger(DefaultLogger::getInstance())
{
}

Box2DWorld::~Box2DWorld()
{
    //    _logger->logDebug("Destructor of Box2DWorld", "[sim_env::Box2DWorld::~Box2DWorld]");
    eraseWorld();
}

WorldPtr Box2DWorld::clone() const
{
    return cloneBox2D();
}

Box2DWorldPtr Box2DWorld::cloneBox2D() const
{
    Box2DWorldLock lock(getMutex());
    auto world_clone = std::make_shared<Box2DWorld>();
    world_clone->loadWorld(_env_desc);
    world_clone->setPositionSteps(_position_steps);
    world_clone->setVelocitySteps(_velocity_steps);
    world_clone->setPhysicsTimeStep(_time_step);
    WorldState world_state;
    getWorldState(world_state);
    world_clone->setWorldState(world_state);
    return world_clone;
}

void Box2DWorld::loadWorld(const std::string& path)
{
    Box2DWorldLock lock(getMutex());
    Box2DEnvironmentDescription env_desc = Box2DEnvironmentDescription();
    parseYAML(path, env_desc);
    eraseWorld();
    createWorld(env_desc);
}

void Box2DWorld::loadWorld(const Box2DEnvironmentDescription& env_desc)
{
    Box2DWorldLock lock(getMutex());
    eraseWorld();
    createWorld(env_desc);
}

RobotPtr Box2DWorld::getRobot(const std::string& name)
{
    return getBox2DRobot(name);
}

RobotConstPtr Box2DWorld::getRobotConst(const std::string& name) const
{
    return getBox2DRobotConst(name);
}

Box2DRobotPtr Box2DWorld::getBox2DRobot(const std::string& name)
{
    Box2DWorldLock lock(getMutex());
    if (_robots.count(name)) {
        return _robots.at(name);
    }
    return nullptr;
}

Box2DRobotConstPtr Box2DWorld::getBox2DRobotConst(const std::string& name) const
{
    Box2DWorldLock lock(getMutex());
    if (_robots.count(name)) {
        return _robots.at(name);
    }
    return nullptr;
}

void Box2DWorld::getRobots(std::vector<RobotPtr>& robots)
{
    Box2DWorldLock lock(getMutex());
    for (auto& robot_map_iter : _robots) {
        robots.push_back(robot_map_iter.second);
    }
}

void Box2DWorld::getRobots(std::vector<RobotConstPtr>& robots) const
{
    Box2DWorldLock lock(getMutex());
    for (auto& robot_map_iter : _robots) {
        robots.push_back(robot_map_iter.second);
    }
}

void Box2DWorld::getBox2DRobots(std::vector<Box2DRobotPtr>& robots)
{
    Box2DWorldLock lock(getMutex());
    for (auto& robot_map_iter : _robots) {
        robots.push_back(robot_map_iter.second);
    }
}

void Box2DWorld::getBox2DRobots(std::vector<Box2DRobotConstPtr>& robots) const
{
    Box2DWorldLock lock(getMutex());
    for (auto& robot_map_iter : _robots) {
        robots.push_back(robot_map_iter.second);
    }
}

bool Box2DWorld::isRobot(const std::string& name) const
{
    Box2DWorldLock lock(getMutex());
    return _robots.count(name) > 0;
}

ObjectPtr Box2DWorld::getObject(const std::string& name, bool exclude_robots)
{
    Box2DWorldLock lock(getMutex());
    ObjectPtr object = getBox2DObject(name);
    if (not object and not exclude_robots) {
        if (_robots.count(name)) {
            object = _robots.at(name);
        }
    }
    return object;
}

ObjectConstPtr Box2DWorld::getObjectConst(const std::string& name, bool exclude_robots) const
{
    Box2DWorldLock lock(getMutex());
    ObjectConstPtr object = getBox2DObjectConst(name);
    if (not object and not exclude_robots) {
        if (_robots.count(name)) {
            object = _robots.at(name);
        }
    }
    return object;
}

Box2DObjectPtr Box2DWorld::getBox2DObject(const std::string& name)
{
    Box2DWorldLock lock(getMutex());
    if (_objects.count(name)) {
        return _objects.at(name);
    }
    return nullptr;
}

Box2DObjectConstPtr Box2DWorld::getBox2DObjectConst(const std::string& name) const
{
    Box2DWorldLock lock(getMutex());
    if (_objects.count(name)) {
        return _objects.at(name);
    }
    return nullptr;
}

void Box2DWorld::getObjects(std::vector<ObjectPtr>& objects, bool exclude_robots)
{
    // TODO do not understand why we cant just call getBox2DObjects(..)
    Box2DWorldLock lock(getMutex());
    if (!exclude_robots) {
        std::vector<RobotPtr> robots;
        getRobots(robots);
        objects.insert(objects.end(), robots.begin(), robots.end());
    }
    for (auto& obj_map_iter : _objects) {
        objects.push_back(obj_map_iter.second);
    }
}

void Box2DWorld::getObjects(std::vector<ObjectConstPtr>& objects, bool exclude_robots) const
{
    // TODO do not understand why we cant just call getBox2DObjects(..)
    Box2DWorldLock lock(getMutex());
    if (!exclude_robots) {
        std::vector<RobotConstPtr> robots;
        getRobots(robots);
        objects.insert(objects.end(), robots.begin(), robots.end());
    }
    for (auto& obj_map_iter : _objects) {
        objects.push_back(obj_map_iter.second);
    }
}

void Box2DWorld::getBox2DObjects(std::vector<Box2DObjectPtr>& objects)
{
    Box2DWorldLock lock(getMutex());
    for (auto& obj_map_iter : _objects) {
        objects.push_back(obj_map_iter.second);
    }
}

void Box2DWorld::getBox2DObjects(std::vector<Box2DObjectConstPtr>& objects) const
{
    Box2DWorldLock lock(getMutex());
    for (auto& obj_map_iter : _objects) {
        objects.push_back(obj_map_iter.second);
    }
}

void Box2DWorld::getObjects(const BoundingBox& aabb, std::vector<ObjectPtr>& objects, bool exclude_robots)
{
    Box2DWorldLock lock(getMutex());
    b2AABB query_aabb;
    query_aabb.lowerBound.x = getScale() * aabb.min_corner[0];
    query_aabb.lowerBound.y = getScale() * aabb.min_corner[1];
    query_aabb.upperBound.x = getScale() * aabb.max_corner[0];
    query_aabb.upperBound.y = getScale() * aabb.max_corner[1];

    Box2DAABBQuery aabb_query;
    auto extents = query_aabb.GetExtents();
    aabb_query.query_box.SetAsBox(extents.x, extents.y);
    aabb_query.query_transform.Set(query_aabb.GetCenter(), 0.0f);
    _world->QueryAABB(&aabb_query, query_aabb);
    for (auto& name : aabb_query.object_hits) {
        if (isRobot(name) and exclude_robots) {
            continue;
        }
        auto object = getObject(name, exclude_robots);
        assert(object);
        objects.push_back(object);
    }
}

void Box2DWorld::getObjects(const BoundingBox& aabb, std::vector<ObjectConstPtr>& objects, bool exclude_robots) const
{

    Box2DWorldLock lock(getMutex());
    b2AABB query_aabb;
    query_aabb.lowerBound.x = getScale() * aabb.min_corner[0];
    query_aabb.lowerBound.y = getScale() * aabb.min_corner[1];
    query_aabb.upperBound.x = getScale() * aabb.max_corner[0];
    query_aabb.upperBound.y = getScale() * aabb.max_corner[1];

    Box2DAABBQuery aabb_query;
    auto extents = query_aabb.GetExtents();
    aabb_query.query_box.SetAsBox(extents.x, extents.y);
    aabb_query.query_transform.Set(query_aabb.GetCenter(), 0.0f);
    _world->QueryAABB(&aabb_query, query_aabb);
    for (auto& name : aabb_query.object_hits) {
        if (isRobot(name) and exclude_robots) {
            continue;
        }
        auto object = getObjectConst(name, exclude_robots);
        assert(object);
        objects.push_back(object);
    }
}

void Box2DWorld::stepPhysics(int steps)
{
    stepPhysics(steps, true, true);
}

void Box2DWorld::stepPhysics(int steps, bool execute_controllers, bool allow_sleeping)
{
    Box2DWorldLock lock(getMutex());
    invalidateCollisionCache();
    for (int i = 0; i < steps; ++i) {
        if (execute_controllers) {
            // call control functions of all robots in the scene
            for (auto& robot_map_iter : _robots) {
                robot_map_iter.second->control(_time_step);
            }
        }
        // simulate physics
        _world->SetAllowSleeping(allow_sleeping);
        _world->Step(_time_step, _velocity_steps, _position_steps);
        _world->ClearForces();
        _world->SetAllowSleeping(true);
    }
}

void Box2DWorld::stepPhysics(std::vector<Contact>& contacts, int steps)
{
    stepPhysics(contacts, steps, true, true);
}

void Box2DWorld::stepPhysics(std::vector<Contact>& contacts, int steps,
    bool execute_controllers, bool allow_sleeping)
{
    Box2DWorldLock lock(getMutex());
    invalidateCollisionCache();
    _collision_checker->startRecordings();
    for (int i = 0; i < steps; ++i) {
        if (execute_controllers) {
            // call control functions of all robots in the scene
            for (auto& robot_map_iter : _robots) {
                robot_map_iter.second->control(_time_step);
            }
        }
        // simulate physics
        _world->SetAllowSleeping(allow_sleeping);
        _world->Step(_time_step, _velocity_steps, _position_steps);
        _world->ClearForces();
        _world->SetAllowSleeping(true);
    }
    _collision_checker->getRecordedContacts(contacts);
}

bool Box2DWorld::supportsPhysics() const
{
    return true;
}

void Box2DWorld::setPhysicsTimeStep(float physics_step)
{
    _time_step = physics_step;
}

void Box2DWorld::setVelocitySteps(int velocity_steps)
{
    _velocity_steps = velocity_steps;
}

void Box2DWorld::setPositionSteps(int position_steps)
{
    _position_steps = position_steps;
}

float Box2DWorld::getPhysicsTimeStep() const
{
    return _time_step;
}

int Box2DWorld::getVelocitySteps() const
{
    return _velocity_steps;
}

int Box2DWorld::getPositionSteps() const
{
    return _position_steps;
}

bool Box2DWorld::isPhysicallyFeasible()
{
    // auto logger = getLogger();
    std::vector<Contact> contacts;
    checkCollision(contacts);
    for (auto& contact : contacts) {
        auto link_a = std::dynamic_pointer_cast<Box2DLink>(contact.link_a.lock());
        auto link_b = std::dynamic_pointer_cast<Box2DLink>(contact.link_b.lock());
        std::vector<bg::model::polygon<bg::model::d2::point_xy<float>, false>> polygons_a;
        std::vector<bg::model::polygon<bg::model::d2::point_xy<float>, false>> polygons_b;
        link_a->getWorldBoostGeometry(polygons_a);
        link_b->getWorldBoostGeometry(polygons_b);
        for (auto& polygon_a : polygons_a) {
            for (auto& polygon_b : polygons_b) {
                std::vector<bg::model::polygon<bg::model::d2::point_xy<float>, false>> intersections;
                bg::correct(polygon_a);
                bg::correct(polygon_b);
                bg::intersection(polygon_a, polygon_b, intersections);
                for (auto& intersection : intersections) {
                    float area = bg::area(intersection);
                    //                    logger->logDebug(boost::format("Area is %f") % area, "[Box2DWorld::isPhysicallyFeasible]");
                    if (area > MAX_ALLOWED_INTERSECTION_AREA) {
                        // logger->logDebug("Current state is physically infeasible", "[Box2DWorld::isPhysicallyFeasible]");
                        return false;
                    }
                }
            }
        }
    }
    return true;
}

WorldViewer::ImageRendererPtr Box2DWorld::getImageRenderer()
{
    return std::make_shared<sim_env::Box2DImageRenderer>(shared_from_this());
}

WorldViewerPtr Box2DWorld::getViewer()
{
    if (!_world_viewer) {
        _world_viewer = std::make_shared<sim_env::Box2DWorldViewer>(shared_from_this());
    }
    return _world_viewer;
}

LoggerPtr Box2DWorld::getLogger()
{
    return _logger;
}

LoggerConstPtr Box2DWorld::getConstLogger() const
{
    return _logger;
}

Eigen::Vector4f Box2DWorld::getWorldBounds() const
{
    return _world_bounds;
}

std::recursive_mutex& Box2DWorld::getMutex() const
{
    return _world_mutex;
}

bool Box2DWorld::atRest(float threshold) const
{
    Box2DWorldLock lock(getMutex());
    bool at_rest = true;
    for (auto& object : _objects) {
        at_rest = at_rest && object.second->atRest(threshold);
        if (not at_rest) {
            return false;
        }
    }
    for (auto& robot : _robots) {
        at_rest = at_rest && robot.second->atRest(threshold);
        if (not at_rest) {
            return false;
        }
    }
    return at_rest;
}

WorldState Box2DWorld::getWorldState() const
{
    Box2DWorldLock lock(_world_mutex);
    WorldState state;
    getWorldState(state);
    return state;
}

void Box2DWorld::getWorldState(WorldState& state) const
{
    Box2DWorldLock lock(_world_mutex);
    ObjectState object_state;
    for (auto& object_item : _objects) {
        object_item.second->getState(object_state);
        state[object_item.first] = object_state;
    }
    for (auto& robot_item : _robots) {
        robot_item.second->getState(object_state);
        state[robot_item.first] = object_state;
    }
}

void Box2DWorld::printWorldState(Logger::LogLevel level) const
{
    const static std::string prefix("[sim_env::Box2DWorld::printWorldState]");
    WorldState state = getWorldState();
    _logger->log("World state: ", level, prefix);
    for (auto& state_item : state) {
        std::stringstream ss;
        ss << state_item.first;
        ObjectState& obj_state = state_item.second;
        ss << " positions: " << obj_state.dof_positions.transpose();
        ss << " velocities: " << obj_state.dof_velocities.transpose();
        ss << " active_dofs: " << obj_state.active_dofs.transpose();
        _logger->log(ss.str(), level, prefix);
    }
}

bool Box2DWorld::setWorldState(WorldState& state)
{
    Box2DWorldLock lock(_world_mutex);
    bool b_success = true;
    for (auto& state_item : state) {
        auto object_iter = _objects.find(state_item.first);
        if (object_iter != _objects.end()) {
            object_iter->second->setState(state_item.second);
        } else {
            auto robot_iter = _robots.find(state_item.first);
            if (robot_iter != _robots.end()) {
                robot_iter->second->setState(state_item.second);
            } else {
                std::stringstream ss;
                ss << "Unknown object encountered! Could not set state of object " << state_item.first;
                _logger->logWarn(ss.str(), "[sim_env::Box2DWorld::setWorldState]");
                b_success = false;
            }
        }
    }
    return b_success;
}

void Box2DWorld::saveState()
{
    Box2DWorldLock lock(getMutex());
    WorldState state;
    getWorldState(state);
    _state_stack.push(state);
}

bool Box2DWorld::restoreState()
{
    Box2DWorldLock lock(getMutex());
    if (_state_stack.empty()) {
        return false;
    }
    bool state_is_set = setWorldState(_state_stack.top());
    _state_stack.pop();
    return state_is_set;
}

void Box2DWorld::dropState()
{
    Box2DWorldLock lock(getMutex());
    if (_state_stack.empty()) {
        return;
    }
    _state_stack.pop();
}

void Box2DWorld::invalidateCollisionCache()
{
    if (_collision_checker) {
        _collision_checker->invalidateCache();
    }
}

void Box2DWorld::setToRest()
{
    for (auto& object : _objects) {
        object.second->setToRest();
    }
    for (auto& robot : _robots) {
        robot.second->setToRest();
    }
}
////////////////////////////// PROTECTED FUNCTIONS //////////////////////////////
std::shared_ptr<b2World> Box2DWorld::getRawBox2DWorld()
{
    return _world;
}

b2Body* Box2DWorld::getGroundBody()
{
    return _b2_ground_body;
}

LinkPtr Box2DWorld::getLink(b2Body* body)
{
    for (auto object_iter : _objects) {
        LinkPtr link = object_iter.second->getLink(body);
        if (link) {
            return link;
        }
    }
    for (auto robot_iter : _robots) {
        LinkPtr link = robot_iter.second->getLink(body);
        if (link) {
            return link;
        }
    }
    return nullptr;
}
////////////////////////////// PRIVATE FUNCTIONS //////////////////////////////
void Box2DWorld::eraseWorld()
{
    if (!_world) { // if there is no world, there shouldn't be anything else
        assert(_robots.empty());
        assert(_objects.empty());
        assert(!_b2_ground_body);
        assert(!_collision_checker);
        return;
    }
    Box2DWorldLock lock(_world_mutex);
    for (auto& robot_iter : _robots) {
        robot_iter.second->destroy(_world);
    }
    _robots.clear();
    for (auto& object_iter : _objects) {
        object_iter.second->destroy(_world);
    }
    _objects.clear();
    deleteGroundBody();
    _world.reset();
    _collision_checker.reset();
    _scale = 1.0;
}

void Box2DWorld::deleteGroundBody()
{
    assert(_world);
    if (_b2_ground_body) {
        auto user_data = static_cast<Box2DLink::Box2DBodyUserData*>(_b2_ground_body->GetUserData());
        delete user_data;
        _world->DestroyBody(_b2_ground_body);
        _b2_ground_body = nullptr;
    }
}

void Box2DWorld::createNewObject(const Box2DObjectDescription& object_desc)
{
    Box2DObjectPtr object(new Box2DObject(object_desc, shared_from_this()));
    if (_objects.count(object->getName())) {
        std::string new_name(object->getName());
        new_name = new_name + std::to_string(_objects.size());
        object->setName(new_name);
    }
    _objects[object->getName()] = object;
}

void Box2DWorld::createNewRobot(const Box2DRobotDescription& robot_desc)
{
    Box2DRobotPtr robot(new Box2DRobot(robot_desc, shared_from_this()));
    if (_robots.count(robot->getName())) {
        std::string new_name(robot->getName());
        new_name = new_name + std::to_string(_robots.size());
        robot->setName(new_name);
    }
    _robots[robot->getName()] = robot;
}

void Box2DWorld::createGround(const Eigen::Vector4f& world_bounds)
{
    if (_b2_ground_body) {
        deleteGroundBody();
    }
    // Create ground body
    b2BodyDef ground_body_def;
    ground_body_def.type = b2_staticBody;
    ground_body_def.position.Set(0.5f * _scale * (world_bounds[0] + world_bounds[2]),
        0.5f * _scale * (world_bounds[1] + world_bounds[3]));
    ground_body_def.userData = new Box2DLink::Box2DBodyUserData("GROUND_BODY", "GROUND_BODY", true);
    _b2_ground_body = _world->CreateBody(&ground_body_def);
    b2FixtureDef fd;
    b2PolygonShape ground_shape;
    ground_shape.SetAsBox(_scale * (world_bounds[2] - world_bounds[0]), _scale * (world_bounds[3] - world_bounds[1]));
    fd.shape = &ground_shape;
    fd.isSensor = true;
    _b2_ground_body->CreateFixture(&fd);
}

void Box2DWorld::createWorld(const Box2DEnvironmentDescription& env_desc)
{
    Box2DWorldLock lock(getMutex());
    _env_desc = env_desc;
    _world.reset(new b2World(b2Vec2(0.0, 0.0)));
    _scale = env_desc.scale;
    // Get the dimension of the ground plane
    _world_bounds = env_desc.world_bounds;
    if (_world_bounds.norm() == 0.0) {
        _world_bounds << float(GROUND_DEFAULT_MIN_X), float(GROUND_DEFAULT_MIN_Y),
            float(GROUND_DEFAULT_MAX_X), float(GROUND_DEFAULT_MAX_Y);
        _world_bounds = _scale * _world_bounds;
    }
    createGround(_world_bounds);
    for (auto& robot_desc : env_desc.robots) {
        createNewRobot(robot_desc);
    }
    for (auto& object_desc : env_desc.objects) {
        createNewObject(object_desc);
    }
    for (auto& state_desc : env_desc.states) {
        Box2DObjectPtr object = getBox2DObject(state_desc.first);
        if (object == nullptr) {
            Box2DRobotPtr robot = getBox2DRobot(state_desc.first);
            if (robot != nullptr) {
                object = robot->getBox2DObject();
            }
            if (object == nullptr) {
                std::string err_msg = boost::str(boost::format("Could not find object %1% in scene. Skipping this...")
                    % state_desc.first);
                _logger->logErr(err_msg,
                    "sim_env/Box2DWorld.cpp::createWorld");
                continue;
            }
        }
        if (not object->isStatic()) {
            object->setDOFPositions(state_desc.second.configuration);
            object->setDOFVelocities(state_desc.second.velocity);
        } else {
            // TODO print an error message instead of an assert and assign default
            assert(state_desc.second.configuration.size() >= 3);
            object->setPose(state_desc.second.configuration[0],
                state_desc.second.configuration[1],
                state_desc.second.configuration[2]);
            Eigen::VectorXf remaining_config(state_desc.second.configuration.size() - 3);
            Eigen::VectorXf remaining_vel(state_desc.second.configuration.size() - 3);
            for (unsigned int i = 3; i < state_desc.second.configuration.size(); ++i) {
                remaining_config[i - 3] = state_desc.second.configuration[i];
                remaining_vel[i - 3] = state_desc.second.velocity[i];
            }
            object->setDOFPositions(remaining_config);
            object->setDOFVelocities(remaining_vel);
        }
    }
    _collision_checker = std::make_shared<Box2DCollisionChecker>(shared_from_this());
    b2ContactListener* contact_listener = _collision_checker.get();
    _world->SetContactListener(contact_listener);
}

bool Box2DWorld::checkCollision(ObjectPtr object_a, ObjectPtr object_b)
{
    Box2DCollidablePtr collidable_a = castCollidable(object_a);
    Box2DCollidablePtr collidable_b = castCollidable(object_b);
    return _collision_checker->checkCollision(collidable_a, collidable_b);
}

bool Box2DWorld::checkCollision(ObjectPtr object_a, ObjectPtr object_b, std::vector<Contact>& contacts)
{
    Box2DCollidablePtr collidable_a = castCollidable(object_a);
    Box2DCollidablePtr collidable_b = castCollidable(object_b);
    return _collision_checker->checkCollision(collidable_a, collidable_b, contacts);
}

bool Box2DWorld::checkCollision(ObjectPtr object_a, LinkPtr link_b)
{
    Box2DCollidablePtr collidable_a = castCollidable(object_a);
    Box2DCollidablePtr collidable_b = castCollidable(link_b);
    return _collision_checker->checkCollision(collidable_a, collidable_b);
}

bool Box2DWorld::checkCollision(ObjectPtr object_a, LinkPtr link_b, std::vector<Contact>& contacts)
{
    Box2DCollidablePtr collidable_a = castCollidable(object_a);
    Box2DCollidablePtr collidable_b = castCollidable(link_b);
    return _collision_checker->checkCollision(collidable_a, collidable_b, contacts);
}

bool Box2DWorld::checkCollision(ObjectPtr object)
{
    Box2DCollidablePtr collidable_b = castCollidable(object);
    return _collision_checker->checkCollision(collidable_b);
}

bool Box2DWorld::checkCollision(ObjectPtr object, std::vector<Contact>& contacts)
{
    Box2DCollidablePtr collidable_b = castCollidable(object);
    return _collision_checker->checkCollision(collidable_b, contacts);
}

bool Box2DWorld::checkCollision(ObjectPtr object_a, const std::vector<ObjectPtr>& other_objects)
{
    Box2DCollidablePtr collidable_a = castCollidable(object_a);
    std::vector<Box2DCollidablePtr> collidables;
    for (auto object : other_objects) {
        collidables.push_back(castCollidable(object));
    }
    return _collision_checker->checkCollision(collidable_a, collidables);
}

bool Box2DWorld::checkCollision(ObjectPtr object_a, const std::vector<ObjectPtr>& other_objects,
    std::vector<Contact>& contacts)
{
    Box2DCollidablePtr collidable_a = castCollidable(object_a);
    std::vector<Box2DCollidablePtr> collidables;
    for (auto object : other_objects) {
        collidables.push_back(castCollidable(object));
    }
    return _collision_checker->checkCollision(collidable_a, collidables, contacts);
}

bool Box2DWorld::checkCollision(LinkPtr link, const std::vector<LinkPtr>& other_links)
{
    Box2DCollidablePtr collidable_a = castCollidable(link);
    std::vector<Box2DCollidablePtr> collidables;
    for (auto object : other_links) {
        collidables.push_back(castCollidable(object));
    }
    return _collision_checker->checkCollision(collidable_a, collidables);
}

bool Box2DWorld::checkCollision(LinkPtr link, const std::vector<LinkPtr>& other_links, std::vector<Contact>& contacts)
{
    Box2DCollidablePtr collidable_a = castCollidable(link);
    std::vector<Box2DCollidablePtr> collidables;
    for (auto object : other_links) {
        collidables.push_back(castCollidable(object));
    }
    return _collision_checker->checkCollision(collidable_a, collidables, contacts);
}

bool Box2DWorld::checkCollision(LinkPtr link_a, const std::vector<ObjectPtr>& other_objects)
{
    Box2DCollidablePtr collidable_a = castCollidable(link_a);
    std::vector<Box2DCollidablePtr> collidables;
    for (auto object : other_objects) {
        collidables.push_back(castCollidable(object));
    }
    return _collision_checker->checkCollision(collidable_a, collidables);
}

bool Box2DWorld::checkCollision(LinkPtr link_a, const std::vector<ObjectPtr>& other_objects,
    std::vector<Contact>& contacts)
{
    Box2DCollidablePtr collidable_a = castCollidable(link_a);
    std::vector<Box2DCollidablePtr> collidables;
    for (auto object : other_objects) {
        collidables.push_back(castCollidable(object));
    }
    return _collision_checker->checkCollision(collidable_a, collidables, contacts);
}

bool Box2DWorld::checkCollision(std::vector<Contact>& contacts)
{
    bool has_contact = false;
    for (auto& item_pair : _objects) {
        has_contact = has_contact or checkCollision(item_pair.second, contacts);
    }
    for (auto& item_pair : _robots) {
        has_contact = has_contact or checkCollision(item_pair.second, contacts);
    }
    assert((has_contact and contacts.size() > 0) or (not has_contact));
    return has_contact;
}

bool Box2DWorld::checkCollision(LinkPtr link)
{
    Box2DCollidablePtr collidable_a = castCollidable(link);
    return _collision_checker->checkCollision(collidable_a);
}

bool Box2DWorld::checkCollision(LinkPtr link, std::vector<Contact>& contacts)
{
    Box2DCollidablePtr collidable_a = castCollidable(link);
    return _collision_checker->checkCollision(collidable_a, contacts);
}

Box2DCollidablePtr Box2DWorld::castCollidable(ObjectPtr object) const
{
    Box2DObjectPtr box2d_object = std::dynamic_pointer_cast<Box2DObject>(object);
    if (not box2d_object) {
        Box2DRobotPtr box2d_robot = std::dynamic_pointer_cast<Box2DRobot>(object);
        if (not box2d_robot) {
            throw std::logic_error("[sim_env::Box2DWorld::castCollidable] Could not cast ObjectPtr to "
                                   "either Box2DObject or Box2DRobot. Whatever type it is, it is not supported.");
        }
        // else just return the robot (it is upcasted to Box2DCollidable automatically)
        return box2d_robot;
    }
    // else just return the object (it is upcasted to Box2DCollidable automatically)
    return box2d_object;
}

Box2DCollidablePtr Box2DWorld::castCollidable(LinkPtr link) const
{
    Box2DLinkPtr box2d_link = std::dynamic_pointer_cast<Box2DLink>(link);
    if (not box2d_link) {
        throw std::logic_error("[sim_env::Box2DWorld::castCollidable] Could not cast LinkPtr to "
                               "either Box2DLink. Whatever type it is, it is not supported.");
    }
    return sim_env::Box2DCollidablePtr();
}
