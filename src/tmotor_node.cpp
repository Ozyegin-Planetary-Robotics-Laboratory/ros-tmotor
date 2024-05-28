#include <mutex>
#include <thread>
#include <ros/ros.h>
#include <functional>
#include <signal.h>
#include <tmotor.hpp>
#include <tmotor/MotorCommand.h>
#include <tmotor/MotorFeedback.h>

sig_atomic_t volatile request_shutdown = 0;

/**
 * @brief MotorNode class
 * 
*/
class MotorNode
{
public:
  MotorNode(int id) :
    motor_manager(id),
    pose_cmd(0.0), vel_cmd(5.0), acc_cmd(5.0),
    pose_cur(0.0), vel_cur(0.0), eff_cur(0.0),
    mode_id(TMotor::MotorModeID::POSITIONVELOCITY)
  {
    std::string can_interface;
    ros::param::get("~can_interface", can_interface);
    ROS_INFO("Connecting to CAN interface %s", can_interface.c_str());
    motor_manager.connect(can_interface.c_str());
    n_nh = ros::NodeHandle("~");
    n_sub = n_nh.subscribe("motor_command", 1, &MotorNode::motorCommandCallback, this);
    n_pub = n_nh.advertise<tmotor::MotorFeedback>("motor_feedback", 1);
  }

  void run() {
    motor_updater = std::thread([this] {
      while (!request_shutdown && ros::ok()) {
        tmotor::MotorFeedback msg;
        pose_cur = (msg.position = motor_manager.getPosition());
        vel_cur = (msg.velocity = motor_manager.getVelocity());
        eff_cur = (msg.current = motor_manager.getCurrent());
        msg.temperature = motor_manager.getTemperature();
        msg.motor_fault = motor_manager.getFault();
        if (msg.motor_fault != TMotor::MotorFault::NONE) {
          std::string warning_msg = "Motor fault detected for ID " + std::to_string(motor_manager.getMotorID()) + " with fault: " + fault_to_str(motor_manager.getFault());
          ROS_WARN("%s", warning_msg.c_str());
        }
        n_pub.publish(msg);
        std::lock_guard<std::mutex> lock(motor_mutex);
        switch (mode_id) {
          case TMotor::MotorModeID::POSITION:
            motor_manager.sendPosition(pose_cmd);
            break;
          case TMotor::MotorModeID::VELOCITY:
            motor_manager.sendVelocity(vel_cmd);
            break;
          case TMotor::MotorModeID::POSITIONVELOCITY:
            motor_manager.sendPositionVelocityAcceleration(pose_cmd, vel_cmd, acc_cmd);
          default:
            break;
        }
        ros::Duration(0.01).sleep();
      }

      // Move the motor close to zero position before shutting down
      float upsilon = 5.0f; // degrees
      while (pose_cur < -upsilon || pose_cur > upsilon) {
        motor_manager.sendPositionVelocityAcceleration(0.0f, 5.0f, 5.0f);
        ros::Duration(0.01).sleep();
      }
    });
    motor_updater.detach();
    ros::spin();
  }

private:
  float pose_cmd, vel_cmd, acc_cmd;
  float pose_cur, vel_cur, eff_cur;
  std::mutex motor_mutex;
  std::thread motor_updater;
  TMotor::AKManager motor_manager;
  ros::NodeHandle n_nh;
  ros::Subscriber n_sub;
  ros::Publisher n_pub;
  TMotor::MotorModeID mode_id;
  
  void motorCommandCallback(const tmotor::MotorCommand::ConstPtr& msg) {
    static const double pose_max = 36000.0;
    static const double pose_min = -36000.0;
    static const double vel_max = 45.0;
    static const double vel_min = 0.0;
    static const double acc_max = 10.0;
    static const double acc_min = 0.0;

    std::lock_guard<std::mutex> lock(motor_mutex);
    mode_id = static_cast<TMotor::MotorModeID>(msg->type);
    pose_cmd = msg->position < pose_max ? msg->position : pose_max;
    pose_cmd = msg->position > pose_min ? msg->position : pose_min;
    vel_cmd = msg->velocity < vel_max ? msg->velocity : vel_max;
    vel_cmd = msg->velocity > vel_min ? msg->velocity : vel_min;
    acc_cmd = msg->acceleration < acc_max ? msg->acceleration : acc_max;
    acc_cmd = msg->acceleration > acc_min ? msg->acceleration : acc_min;
  }

  std::string fault_to_str(TMotor::MotorFault fault) {
    static std::map<TMotor::MotorFault, std::string> dict({
      {TMotor::MotorFault::NONE,            "NONE"           },
      {TMotor::MotorFault::OVERTEMPERATURE, "OVERTEMPERATURE"},
      {TMotor::MotorFault::OVERCURRENT,     "OVERCURRENT"    },
      {TMotor::MotorFault::OVERVOLTAGE,     "OVERVOLTAGE"    },
      {TMotor::MotorFault::UNDERVOLTAGE,    "UNDERVOLTAGE"   },
      {TMotor::MotorFault::ENCODER,         "ENCODER"        },
      {TMotor::MotorFault::HARDWARE,        "HARDWARE"       }
    });
    return dict[fault];
  }

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "tmotor_node");
  int id;
  ros::param::get("~id", id);
  ROS_INFO("AK motor node started with id %d", id);
  signal(SIGINT, [](int sig) { request_shutdown = 1; });
  MotorNode node(id);
  node.run();
  return 0;
}