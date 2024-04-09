#include "devices/robot1.h"
#include <boost/bind.hpp>
#include <chrono>
#include <regex>
#include <sstream>
#include <string>
#include <thread>

//std::string Robot1State::JOINT_NAMES[Robot1State::NUM_JOINTS];
double Robot1State::JOINT_THRESHOLD = 0.0;

double Robot1Interface::CONTROL_TIME_BUFFER = 0.0;
int    Robot1Interface::CONTROL_FREQUENCY = 0;
double Robot1Interface::MAX_ACCELERATION = 0.0;

Robot1State::Robot1State() :
    m_message(),
    m_valid(false)
{

}

void Robot1State::to_matrix(std::vector<float>& trajectory) const
{
    if(m_message)
    {
        for(std::size_t joint_idx = 0; joint_idx < NUM_JOINTS; ++joint_idx)
        {
            trajectory.push_back(m_message->position[joint_idx]);
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

//const your_robots::robot1Joints::ConstPtr& Robot1State::get_message()
const sensor_msgs::JointState::ConstPtr& Robot1State::get_message()
{
    return m_message;
}

//void Robot1State::set_message(const your_robots::robot1Joints::ConstPtr& message)
void Robot1State::set_message(const sensor_msgs::JointState::ConstPtr& message)
{
    m_message = message;
    m_valid = true;
}

bool Robot1State::valid() const
{
    return m_valid;
}

bool Robot1State::within_threshold(const std::vector<float>& trajectory, std::size_t trajectory_idx) const
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


Robot1Interface::Robot1Interface(ros::NodeHandle handle) :
  m_stateSubscriber(),
  m_statePublisher(),
  m_currentState(),
  m_publish_message()

{
    handle.getParam("control/robot1/control_time_buffer", CONTROL_TIME_BUFFER);
    handle.getParam("control/robot1/control_frequency", CONTROL_FREQUENCY);
    handle.getParam("control/robot1/max_acceleration", MAX_ACCELERATION);
    handle.getParam("control/robot1/joint_distance_threshold", Robot1State::JOINT_THRESHOLD);
/*
    std::cout << "control time buffer: " << CONTROL_TIME_BUFFER << ", control frequency: " << CONTROL_FREQUENCY << ", max accel: " << MAX_ACCELERATION << ", joint thresh: " << Robot1State::JOINT_THRESHOLD << std::endl;

    m_messageTime = (1.0 / CONTROL_FREQUENCY) * CONTROL_TIME_BUFFER;

    m_publish_message.acceleration = MAX_ACCELERATION;
    m_publish_message.blend = 0;
    m_publish_message.command = "speed";
    m_publish_message.gain = 0;
    m_publish_message.jointcontrol = true;
    m_publish_message.lookahead = 0;
    m_publish_message.time = m_messageTime;
    m_publish_message.velocity = 0;
*/
//    m_statePublisher = handle.advertise<your_robots::robot1Control>("/robot1/control", 1);
//    m_stateSubscriber = handle.subscribe("/robot1/joints", 1, &Robot1Interface::state_callback, this);

    m_statePublisher = handle.advertise<sensor_msgs::JointState>("/robot/robot1/control", 1);
    m_stateSubscriber = handle.subscribe("/robot/robot1/state", 1, &Robot1Interface::state_callback, this);

}

const Robot1State& Robot1Interface::get_state()
{
    return m_currentState;
}

void Robot1Interface::publish_state(const std::vector<float>& trajectory, std::size_t trajectory_idx)
{
    for(std::size_t joint_idx = 0; joint_idx < Robot1State::NUM_JOINTS; ++joint_idx)
    {
        m_publish_message.position[joint_idx] = trajectory[trajectory_idx + joint_idx];
    }

    m_statePublisher.publish(m_publish_message);
}

//void Robot1Interface::state_callback(const your_robots::robot1Joints::ConstPtr& message)
void Robot1Interface::state_callback(const sensor_msgs::JointState::ConstPtr& message)
{
    m_currentState.set_message(message);
}
