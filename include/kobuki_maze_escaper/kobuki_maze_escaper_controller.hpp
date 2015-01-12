// For Advanced Intelligent Robotics Course (Kyushu University in Japan)
// Original source is the kobuki_controller_tutorial by Marcus Liebhardt, Yujin Robot
//                    the random_walker_controller by Marcus Liebhardt, Yujin Robot

#ifndef RANDOM_WALKER_CONTROLLER_HPP_
#define RANDOM_WALKER_CONTROLLER_HPP_

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/Led.h>
#include <kobuki_msgs/WheelDropEvent.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <yocs_controllers/default_controller.hpp>

namespace kobuki
{

class RandomWalkerController : public yocs::Controller
{
public:
  RandomWalkerController(ros::NodeHandle& nh_priv, std::string& name) : Controller(),
                                                                           nh_priv_(nh_priv),
                                                                           name_(name),
                                                                           change_direction_(false),
                                                                           stop_(false),
                                                                           isGoal_(false),
                                                                           bumper_left_pressed_(false),
                                                                           bumper_center_pressed_(false),
                                                                           bumper_right_pressed_(false),
                                                                           cliff_left_detected_(false),
                                                                           cliff_center_detected_(false),
                                                                           cliff_right_detected_(false),
                                                                           turning_(false),
                                                                           turning_direction_(1)
                                                                           {};
  ~RandomWalkerController(){};

  bool init()
  {
    bumper_event_subscriber_ = nh_priv_.subscribe("events/bumper", 10, &RandomWalkerController::bumperEventCB, this);
    cliff_event_subscriber_ = nh_priv_.subscribe("events/cliff", 10, &RandomWalkerController::cliffEventCB, this);
    cmd_vel_publisher_ = nh_priv_.advertise<geometry_msgs::Twist>("commands/velocity", 10);

    nh_priv_.param("linear_velocity", vel_lin_, 0.5);
    nh_priv_.param("angular_velocity", vel_ang_, 0.1);
    ROS_INFO_STREAM("Velocity parameters: linear velocity = " << vel_lin_
                    << ", angular velocity = " << vel_ang_ << " [" << name_ <<"]");
    std::srand(std::time(0));
    stime_ =  ros::Time::now();
    return true;
  };

  void spin();

private:
  /// Private ROS handle
  ros::NodeHandle nh_priv_;
  /// Node(let) name
  std::string name_;
  /// Subscribers
  ros::Subscriber bumper_event_subscriber_, cliff_event_subscriber_;
  /// Publishers
  ros::Publisher cmd_vel_publisher_;
  /// Flag for changing direction
  bool change_direction_;
  /// Flag for stopping
  bool stop_;
  /// Flag for left bumper's state
  bool bumper_left_pressed_;
  /// Flag for center bumper's state
  bool bumper_center_pressed_;
  /// Flag for right bumper's state
  bool bumper_right_pressed_;
  /// Flag for left cliff sensor's state
  bool cliff_left_detected_;
  /// Flag for center cliff sensor's state
  bool cliff_center_detected_;
  /// Flag for right cliff sensor's state
  bool cliff_right_detected_;
  /// Linear velocity for moving straight
  double vel_lin_;
  /// Angular velocity for rotating
  double vel_ang_;
  /// Randomly chosen turning duration
  ros::Duration turning_duration_;
  /// Randomly chosen turning direction
  int turning_direction_;
  /// Start time of turning
  ros::Time turning_start_;
  /// Flag for turning state
  bool turning_;
  /// Start time
  ros::Time stime_;
  /// End time
  ros::Time etime_;
  /// Goal
  bool isGoal_;

  /**
   * @brief Trigger direction change and LED blink, when a bumper is pressed
   * @param msg bumper event
   */
  void bumperEventCB(const kobuki_msgs::BumperEventConstPtr msg);

  /**
   * @brief Trigger direction change and LED blink, when a cliff is detected
   * @param msg cliff event
   */
  void cliffEventCB(const kobuki_msgs::CliffEventConstPtr msg);
};

void RandomWalkerController::bumperEventCB(const kobuki_msgs::BumperEventConstPtr msg)
{
  if (msg->state == kobuki_msgs::BumperEvent::PRESSED)
  {
    switch (msg->bumper)
    {
      case kobuki_msgs::BumperEvent::LEFT:
        if (!bumper_left_pressed_)
        {
          ROS_INFO_STREAM("Bumper: LEFT");
          bumper_left_pressed_ = true;
          change_direction_ = true;
        }
        break;
      case kobuki_msgs::BumperEvent::CENTER:
        if (!bumper_center_pressed_)
        {
          ROS_INFO_STREAM("Bumper: CENTER");
          bumper_center_pressed_ = true;
          change_direction_ = true;
        }
        break;
      case kobuki_msgs::BumperEvent::RIGHT:
        if (!bumper_right_pressed_)
        {
          ROS_INFO_STREAM("Bumper: RIGHT");
          bumper_right_pressed_ = true;
          change_direction_ = true;
        }
        break;
    }
  }
  else // kobuki_msgs::BumperEvent::RELEASED
  {
    switch (msg->bumper)
    {
      case kobuki_msgs::BumperEvent::LEFT:    bumper_left_pressed_   = false; break;
      case kobuki_msgs::BumperEvent::CENTER:  bumper_center_pressed_ = false; break;
      case kobuki_msgs::BumperEvent::RIGHT:   bumper_right_pressed_  = false; break;
    }
  }
};

void RandomWalkerController::cliffEventCB(const kobuki_msgs::CliffEventConstPtr msg)
{
  if (msg->state == kobuki_msgs::CliffEvent::CLIFF)
  {
    switch (msg->sensor)
    {
      case kobuki_msgs::CliffEvent::LEFT:
        if (!cliff_left_detected_)
        {
          ROS_INFO_STREAM("Cliff: LEFT");
          cliff_left_detected_ = true;
        }
        break;
      case kobuki_msgs::CliffEvent::CENTER:
        if (!cliff_center_detected_)
        {
          ROS_INFO_STREAM("Cliff: CENTER");
          cliff_center_detected_ = true;
        }
        break;
      case kobuki_msgs::CliffEvent::RIGHT:
        if (!cliff_right_detected_)
        {
          ROS_INFO_STREAM("Cliff: RIGHT");
          cliff_right_detected_ = true;
        }
        break;
    }
  }
  else // kobuki_msgs::BumperEvent::FLOOR
  {
    switch (msg->sensor)
    {
      case kobuki_msgs::CliffEvent::LEFT:    cliff_left_detected_   = false; break;
      case kobuki_msgs::CliffEvent::CENTER:  cliff_center_detected_ = false; break;
      case kobuki_msgs::CliffEvent::RIGHT:   cliff_right_detected_  = false; break;
    }
  }

  if(cliff_left_detected_ && cliff_center_detected_ && cliff_right_detected_)
  {
    //stop_ = true;
    ROS_INFO_STREAM("Goal. Stopping!");
  }
};

void RandomWalkerController::spin()
{
  // Velocity commands
  geometry_msgs::TwistPtr cmd_vel_msg_ptr;
  cmd_vel_msg_ptr.reset(new geometry_msgs::Twist());

  if (stop_)
  {
    if(!isGoal_)
    {
      etime_ = ros::Time::now();
      double secs = (etime_ - stime_).toSec();
      ROS_INFO("elapsed time: %f",secs);
    }
    isGoal_ = true;
    cmd_vel_publisher_.publish(cmd_vel_msg_ptr); // will be all zero when initialised

    return;
  }

  if (change_direction_)
  {
    change_direction_ = false;
    // calculate a random turning angle (-180 ... +180) based on the set angular velocity
    // time for turning 180 degrees in seconds = M_PI / angular velocity
    turning_duration_ = ros::Duration(((double)std::rand() / (double)RAND_MAX) * (M_PI / vel_ang_));
    // randomly chosen turning direction
    if (((double)std::rand() / (double)RAND_MAX) >= 0.5)
    {
      turning_direction_ = 1;
    }
    else
    {
      turning_direction_ = -1;
    }
    turning_start_ = ros::Time::now();
    turning_ = true;
  }

  if (turning_)
  {
    if ((ros::Time::now() - turning_start_) < turning_duration_)
    {
      cmd_vel_msg_ptr->angular.z = turning_direction_ * vel_ang_;
      cmd_vel_publisher_.publish(cmd_vel_msg_ptr);
    }
    else
    {
      turning_ = false;
    }
  }
  else
  {
    cmd_vel_msg_ptr->linear.x = vel_lin_;
    cmd_vel_publisher_.publish(cmd_vel_msg_ptr);
  }
};

} // namespace kobuki
#endif /* RANDOM_WALKER_CONTROLLER_HPP_ */
