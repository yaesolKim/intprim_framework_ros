#pragma once

#include "device_interface.h"
#include "robot_interface.h"

#include <vector>

#include "ros/ros.h"

//#include <your_robots/robot1Control.h>
//#include <your_robots/robot1Joints.h>
#include <sensor_msgs/JointState.h>

/*
로봇과 intprim ros framework 사이의 통신에 두 ros message가 사용된다.
1. Robot1Joints: 로봇 드라이버에서 프레임워크로 퍼블리시 된다. 로봇의 조인트 포지션과 벨로시티다.
2. Robot1Control: 프레임워크에서 로봇 드라이버로 퍼블리시 된다. 프레임워크에서 생성된 추론 결과와 다른 useful data이다.
*/

class Robot1State : public DeviceState
{
    public:
        static constexpr unsigned int           NUM_JOINTS = 7;
        static const std::string                JOINT_NAMES[NUM_JOINTS]; //added
        static double                           JOINT_THRESHOLD;

        Robot1State();

        void to_matrix(std::vector<float>& trajectory) const;

        const sensor_msgs::JointState::ConstPtr& get_message(); //added for JOintState.h

        bool valid() const;

        //void set_message(const your_robots::robot1Joints::ConstPtr& message);
        void set_message(const sensor_msgs::JointState::ConstPtr& message);

        bool within_threshold(const std::vector<float>& trajectory, std::size_t trajectory_idx) const;

    private:
        //your_robots::robot1Joints::ConstPtr         m_message;
        sensor_msgs::JointState::ConstPtr       m_message;
        bool                                        m_valid;
};

class Robot1Interface : public RobotInterface
{
    public:
        //Robot1Interface(ros::NodeHandle handle);
        Robot1Interface(ros::NodeHandle handle);//, std::string robot_type);

        // Covariant return type.
        const Robot1State& get_state();

        void publish_state(const std::vector<float>& trajectory, std::size_t trajectory_idx);
    private:
        static double                               CONTROL_TIME_BUFFER;
        static int                                  CONTROL_FREQUENCY;
        static double                               MAX_ACCELERATION;

        ros::Subscriber                             m_stateSubscriber;
        ros::Publisher                              m_statePublisher;

        Robot1State                                 m_currentState;
        //your_robots::Robot1Control                  m_publishMessage;
        sensor_msgs::JointState                     m_publish_message;

        unsigned int                                m_control_frequency;
        float                                       m_messageTime;

        //void state_callback(const your_robots::robot1Joints::ConstPtr& message);
        void state_callback(const sensor_msgs::JointState::ConstPtr& message);
};
