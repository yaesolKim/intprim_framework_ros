/* @author Joseph Campbell <jacampb1@asu.edu>, Interactive Robotics Lab, Arizona State University */
#pragma once

#include "device_interface.h"
#include "robot_interface.h"
#include <vector>
#include "ros/ros.h"

#include <sensor_msgs/JointState.h>

/*!
    hand6State is an implementation of the DeviceState virtual class for hand6 robots.
*/
class hand6State : public DeviceState
{
    public:
        static constexpr unsigned int           NUM_JOINTS = 18;
        static std::string                      JOINT_NAMES[NUM_JOINTS];
        static double                           JOINT_THRESHOLD;

        hand6State();

        void to_matrix(std::vector<float>& trajectory) const;

        const sensor_msgs::JointState::ConstPtr& get_message();

        bool valid() const;

        void set_message(const sensor_msgs::JointState::ConstPtr& message);

        bool within_threshold(const std::vector<float>& trajectory, std::size_t trajectory_idx) const;

    private:
        sensor_msgs::JointState::ConstPtr       m_message;
        bool                                    m_valid;
};

/*!
    hand6Iterface is an implementation of the RobotInterface virtual class for hand6 robots.
    This interface is designed to receive state messages from the custom hand6 controller included with this framework.
*/
class hand6Interface : public RobotInterface
{
    public:
        hand6Interface(ros::NodeHandle handle, std::string robot_type);

        // Covariant return type.
        const hand6State& get_state();

        void publish_state(const std::vector<float>& trajectory, std::size_t trajectory_idx);
    private:
        ros::Subscriber                             m_state_subscriber;
        ros::Publisher                              m_state_publisher;

        hand6State                                  m_current_state;
        sensor_msgs::JointState                     m_publish_message;

        unsigned int                                m_control_frequency;
        float                                       m_message_time;

        void state_callback(const sensor_msgs::JointState::ConstPtr& message);
};
