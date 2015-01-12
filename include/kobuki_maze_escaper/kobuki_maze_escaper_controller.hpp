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
#include <gazebo_msgs/GetModelState.h>

#define rad2deg(x) ((x)*(180.0)/M_PI)
#define deg2rad(x) ((x)*M_PI/180.0)

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
    cmd_vel_publisher_ = nh_priv_.advertise<geometry_msgs::Twist>("commands/velocity", 10);
    kobuki_property_client_  = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

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
  /// ROS handle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  /// Node(let) name
  std::string name_;
  /// Subscribers
  ros::Subscriber bumper_event_subscriber_;
  ros::ServiceClient kobuki_property_client_;
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
  /// global pos of x, y and theta
  double pos_x_, pos_y_, pos_th_;

  /**
   * @brief Trigger direction change and LED blink, when a bumper is pressed
   * @param msg bumper event
   */
  void bumperEventCB(const kobuki_msgs::BumperEventConstPtr msg);

  /**
   * @brief calculate z-axis rotation from Odom_Quaternion(z, w) -180 ~ +180
   * @param quaternion z and w
   */
  double quaternionToEuler(double z, double w);

  /**
   * @brief get model property from gazebo
   */
  void getPropertyCallback(const ros::TimerEvent& e);

};


void RandomWalkerController::bumperEventCB(const kobuki_msgs::BumperEventConstPtr msg)
{
  if (msg->state == kobuki_msgs::BumperEvent::PRESSED)
  {
    switch (msg->bumper)
    {
      case kobuki_msgs::BumperEvent::LEFT:
        ROS_INFO_STREAM("Bumper LEFT: True");
        bumper_left_pressed_ = true;
        break;
      case kobuki_msgs::BumperEvent::CENTER:
        ROS_INFO_STREAM("Bumper CENTER: True");
        bumper_center_pressed_ = true;
        change_direction_ = true;
        break;
      case kobuki_msgs::BumperEvent::RIGHT:
        ROS_INFO_STREAM("Bumper RIGHT: True");
        bumper_right_pressed_ = true;
        break;
    }
  }
  else // kobuki_msgs::BumperEvent::RELEASED
  {
    switch (msg->bumper)
    {
      case kobuki_msgs::BumperEvent::LEFT:
        ROS_INFO_STREAM("Bumper LEFT: False");
        bumper_left_pressed_ = false;
        break;
      case kobuki_msgs::BumperEvent::CENTER:
        ROS_INFO_STREAM("Bumper CENTER: False");
        bumper_center_pressed_ = false;
        break;
      case kobuki_msgs::BumperEvent::RIGHT:
        ROS_INFO_STREAM("Bumper RIGHT: False");
        bumper_right_pressed_ = false;
        break;
    }
  }
}

//------------------------------------------------------------------------------
double RandomWalkerController::quaternionToEuler(double z, double w)
{
  double sqw, sqz;
  sqw = w*w;
  sqz = z*z;
  return atan2(2.0*(z*w),(-sqz + sqw));
}

//------------------------------------------------------------------------------
void RandomWalkerController::spin()
{
  double z, w;
  double temp_th;

  gazebo_msgs::GetModelState get_model_state;
  get_model_state.request.model_name = "mobile_base";

  if(kobuki_property_client_.call(get_model_state))
  {
    pos_x_ = get_model_state.response.pose.position.x;
    pos_y_ = get_model_state.response.pose.position.y;

    z = get_model_state.response.pose.orientation.z;
    w = get_model_state.response.pose.orientation.w;
    temp_th = quaternionToEuler(z, w);
    pos_th_ = rad2deg(temp_th);
  }
  ROS_INFO("[%.2f, %.2f, %.2f] [%d,%d,%d]",pos_x_, pos_y_, pos_th_, bumper_left_pressed_, bumper_center_pressed_, bumper_right_pressed_);
}

//------------------------------------------------------------------------------
} // namespace kobuki
#endif /* RANDOM_WALKER_CONTROLLER_HPP_ */
