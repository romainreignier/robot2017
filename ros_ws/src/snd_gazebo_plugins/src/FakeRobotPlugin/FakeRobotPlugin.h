#pragma once

#include <atomic>
#include <memory>
#include <thread>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Link.hh>

class FakeRobotPlugin : public gazebo::ModelPlugin
{
public:
    FakeRobotPlugin();
    virtual ~FakeRobotPlugin();
    void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;
    void Init() override;
    void Reset() override;

protected:
    gazebo::physics::WorldPtr m_world;
    gazebo::physics::ModelPtr m_model;

    gazebo::event::ConnectionPtr m_ros_publish_connection;
};
