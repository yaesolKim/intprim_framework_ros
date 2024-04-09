#include "devices/iiwa7.h"
#include <boost/bind.hpp>
#include <chrono>
#include <regex>
#include <sstream>
#include <string>
#include <thread>

std::string IIWA7State::JOINT_NAMES[IIWA7State::NUM_JOINTS]= {
    "IIWA7_joint1",
    "IIWA7_joint2",
    "IIWA7_joint3",
    "IIWA7_joint4",
    "IIWA7_joint5",
    "IIWA7_joint6",
    "IIWA7_joint7"
};

double IIWA7State::JOINT_THRESHOLD = 0.0;

//double IIWA7Interface::CONTROL_TIME_BUFFER = 0.0;
//int    IIWA7Interface::CONTROL_FREQUENCY = 0;
//double IIWA7Interface::MAX_ACCELERATION = 0.0;

IIWA7State::IIWA7State() :
    m_message(),
    m_valid(false)
{

}

void IIWA7State::to_matrix(std::vector<float>& trajectory) const
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
const sensor_msgs::JointState::ConstPtr& IIWA7State::get_message()
{
    return m_message;
}

//void Robot1State::set_message(const your_robots::robot1Joints::ConstPtr& message)
void IIWA7State::set_message(const sensor_msgs::JointState::ConstPtr& message)
{
    m_message = message;
    m_valid = true;
}

bool IIWA7State::valid() const
{
    return m_valid;
}

bool IIWA7State::within_threshold(const std::vector<float>& trajectory, std::size_t trajectory_idx) const
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


IIWA7Interface::IIWA7Interface(ros::NodeHandle handle) :
  m_stateSubscriber(),
  m_statePublisher(),
  m_currentState(),
  m_publish_message()

{

/*
  for(int joint = 0; joint < IIWA7State::NUM_JOINTS; joint++)
  {
      std::string name;

      name = "iiwa7_joint" + std::to_string(joint+1);
      IIWA7State::JOINT_NAMES[joint] = name;

      m_publish_message.name.push_back(name);
      m_publish_message.position.push_back(0.0);
  }
*/

  handle.getParam("control/iiwa7/joint_distance_threshold", IIWA7State::JOINT_THRESHOLD);

  for(const auto& name : IIWA7State::JOINT_NAMES)
  {
      m_publish_message.name.push_back(name);

  }

  m_statePublisher = handle.advertise<sensor_msgs::JointState>("/robot/iiwa7/control", 1);
  m_stateSubscriber = handle.subscribe("/robot/iiwa7/state", 1, &IIWA7Interface::state_callback, this);

/*

    handle.getParam("control/iiwa7/control_time_buffer", CONTROL_TIME_BUFFER);
    handle.getParam("control/iiwa7/control_frequency", CONTROL_FREQUENCY);
    handle.getParam("control/iiwa7/max_acceleration", MAX_ACCELERATION);
    handle.getParam("control/iiwa7/joint_distance_threshold", IIWA7State::JOINT_THRESHOLD);

    */
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


}

const IIWA7State& IIWA7Interface::get_state()
{
    return m_currentState;
}

void IIWA7Interface::publish_state(const std::vector<float>& trajectory, std::size_t trajectory_idx)
{
    for(std::size_t joint_idx = 0; joint_idx < IIWA7State::NUM_JOINTS; ++joint_idx)
    {
        m_publish_message.position[joint_idx] = trajectory[trajectory_idx + joint_idx];
    }

    m_statePublisher.publish(m_publish_message);
}

//void Robot1Interface::state_callback(const your_robots::robot1Joints::ConstPtr& message)
void IIWA7Interface::state_callback(const sensor_msgs::JointState::ConstPtr& message)
{
    m_currentState.set_message(message);
}
