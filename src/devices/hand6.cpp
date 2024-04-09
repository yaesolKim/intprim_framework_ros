/* @author Joseph Campbell <jacampb1@asu.edu>, Interactive Robotics Lab, Arizona State University */
#include "devices/hand6.h"
#include <boost/bind.hpp>
#include <chrono>
#include <regex>
#include <sstream>
#include <string>
#include <thread>

std::string hand6State::JOINT_NAMES[hand6State::NUM_JOINTS]; //18?

double hand6State::JOINT_THRESHOLD = 0.0;

hand6State::hand6State() :
    m_message(),
    m_valid(false)
{

}

void hand6State::to_matrix(std::vector<float>& trajectory) const
{
    if(m_message)
    {
        for(std::size_t joint_idx = 0; joint_idx < NUM_JOINTS; ++joint_idx) //조인트 인덱스 18까지
        {
            trajectory.push_back(m_message->position[joint_idx]);
            //trajectory.push_back(m_message->position[joint_idx]);
            //trajectory.push_back(m_message->position[joint_idx]);
        }
    }
    else
    {
        for(std::size_t joint_idx = 0; joint_idx < NUM_JOINTS; ++joint_idx)
        {
            trajectory.push_back(0.0);
        }
    }
}

const sensor_msgs::JointState::ConstPtr& hand6State::get_message()
{
    return m_message;
}

void hand6State::set_message(const sensor_msgs::JointState::ConstPtr& message)
{
    m_message = message;
    m_valid = true;
}

bool hand6State::valid() const
{
    return m_valid;
}

bool hand6State::within_threshold(const std::vector<float>& trajectory, std::size_t trajectory_idx) const
{
    if(m_message)
    {
        for(std::size_t joint_idx = 0; joint_idx < NUM_JOINTS; ++joint_idx)
        {
            if(std::abs(m_message->position[joint_idx] - trajectory[joint_idx + trajectory_idx]) > JOINT_THRESHOLD)
            {
                return false;
            }
        }
        return true;
    }
    return false;
}

hand6Interface::hand6Interface(ros::NodeHandle handle, std::string robot_type) :
    m_state_subscriber(),
    m_state_publisher(),
    m_current_state(),
    m_publish_message()
{
    for(int joint = 0; joint < hand6State::NUM_JOINTS; joint++) //18번
    {
        std::string name;
        //else if(robot_type.compare("observe_sim") == 0)
        {
             name = "hand6_finger" + std::to_string(joint+1) + "_x";
             hand6State::JOINT_NAMES[joint] = name;
             m_publish_message.name.push_back(name);
        m_publish_message.position.push_back(0.0);
             name = "hand6_finger" + std::to_string(joint+1) + "_y";
             hand6State::JOINT_NAMES[joint] = name;
             m_publish_message.name.push_back(name);
        m_publish_message.position.push_back(0.0);
             name = "hand6_finger" + std::to_string(joint+1) + "_z";
             hand6State::JOINT_NAMES[joint] = name;
             m_publish_message.name.push_back(name);
        m_publish_message.position.push_back(0.0);
             joint++;
             joint++;
        }

    }

    if(robot_type.compare("control_sim") == 0)
    {
        handle.getParam("control/hand6/joint_distance_threshold", hand6State::JOINT_THRESHOLD);
        m_state_publisher = handle.advertise<sensor_msgs::JointState>("/robot/hand6/control", 1);
        m_state_subscriber = handle.subscribe("/robot/hand6/state", 1, &hand6Interface::state_callback, this);
    }
    else if(robot_type.compare("observe_sim") == 0)
    {
        handle.getParam("control/hand6/joint_distance_threshold", hand6State::JOINT_THRESHOLD);
        m_state_publisher = handle.advertise<sensor_msgs::JointState>("/robot/hand6/control", 1);
        m_state_subscriber = handle.subscribe("/robot/hand6/state", 1, &hand6Interface::state_callback, this);
    }
    else{
        handle.getParam("control/hand6/joint_distance_threshold", hand6State::JOINT_THRESHOLD);
        m_state_publisher = handle.advertise<sensor_msgs::JointState>("/robot/hand6/control", 1);
        m_state_subscriber = handle.subscribe("/robot/hand6/state", 1, &hand6Interface::state_callback, this);
    }
}

const hand6State& hand6Interface::get_state()
{
    return m_current_state;
}

void hand6Interface::publish_state(const std::vector<float>& trajectory, std::size_t trajectory_idx)
{
    for(std::size_t joint_idx = 0; joint_idx < hand6State::NUM_JOINTS; ++joint_idx) //18
    {
        m_publish_message.position[joint_idx] = trajectory[trajectory_idx + joint_idx];
    }

    m_state_publisher.publish(m_publish_message);
}

void hand6Interface::state_callback(const sensor_msgs::JointState::ConstPtr& message)
{
    m_current_state.set_message(message);
}
