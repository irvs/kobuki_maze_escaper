#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <gazebo_msgs/GetModelState.h>

#define rad2deg(x) ((x)*(180.0)/M_PI)
#define deg2rad(x) ((x)*M_PI/180.0)

//------------------------------------------------------------------------------
class MazeSolution
{
//------------------------------------------------------------------------------
public:
  MazeSolution() :
    nh_priv_("~"),
    update_time_(0.01),  //sec
    change_direction_(false),
    bumper_left_pressed_(false),
    bumper_center_pressed_(false),
    bumper_right_pressed_(false),
    is_debug_(false)
  {
    // Init parameter
    nh_priv_.param("update_time", update_time_, update_time_);
    nh_priv_.param("is_debug", is_debug_, is_debug_);
    // Subscriber and publisher
    bumper_event_subscriber_ = nh_.subscribe("/mobile_base/events/bumper", 10, &MazeSolution::bumperEventCB, this);
    cmd_vel_publisher_       = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
    kobuki_property_client_  = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    //TimerEvent
    update_timer_ = nh_.createTimer(ros::Duration(update_time_), &MazeSolution::getPropertyCallback, this);
    check_timer_  = nh_.createTimer(ros::Duration(update_time_), &MazeSolution::checkCallback, this);
  }

  //----------------------------------------------------------------------------
  ~MazeSolution()
  {
  }

//------------------------------------------------------------------------------
private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  // ROS Timer
  ros::Timer update_timer_, check_timer_;
  // ROS Subscriber, Publisher, ServiceClient
  ros::Subscriber bumper_event_subscriber_;
  ros::Publisher  cmd_vel_publisher_;
  ros::ServiceClient kobuki_property_client_;
  // ROS Parameters
  double update_time_;
  bool is_debug_;
  // Flag for changing direction
  bool change_direction_;
  /// Flag for left bumper's state
  bool bumper_left_pressed_;
  /// Flag for center bumper's state
  bool bumper_center_pressed_;
  /// Flag for right bumper's state
  bool bumper_right_pressed_;
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
   * @brief calculate distance of p0 and p1
   * @param position p0 (x0,y0) and p1 (x1,y1)
   */
  double calculateDistance(double x0, double y0, double x1, double y1);

  /**
   * @brief get model property from gazebo
   */
  void getPropertyCallback(const ros::TimerEvent& e);

  /**
   * @brief check robot state and send velocity command
   */
  void checkCallback(const ros::TimerEvent& e);

  /**
   * @brief forward speed linear[m/s] rotational_speed angular[rad/s]
   * @param linear and angular value
   */
  void pubVel(double linear, double angular);
};

//------------------------------------------------------------------------------
void MazeSolution::bumperEventCB(const kobuki_msgs::BumperEventConstPtr msg)
{
  if (msg->state == kobuki_msgs::BumperEvent::PRESSED)
  {
    switch (msg->bumper)
    {
      case kobuki_msgs::BumperEvent::LEFT:
        ROS_INFO_STREAM("Bumper LEFT: True");
        bumper_left_pressed_ = true;
        change_direction_ = true;
        break;
      case kobuki_msgs::BumperEvent::CENTER:
        ROS_INFO_STREAM("Bumper CENTER: True");
        bumper_center_pressed_ = true;
        change_direction_ = true;
        break;
      case kobuki_msgs::BumperEvent::RIGHT:
        ROS_INFO_STREAM("Bumper RIGHT: True");
        bumper_right_pressed_ = true;
        change_direction_ = true;
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
        change_direction_ = false;
        break;
      case kobuki_msgs::BumperEvent::CENTER:
        ROS_INFO_STREAM("Bumper CENTER: False");
        bumper_center_pressed_ = false;
        change_direction_ = false;
        break;
      case kobuki_msgs::BumperEvent::RIGHT:
        ROS_INFO_STREAM("Bumper RIGHT: False");
        bumper_right_pressed_ = false;
        change_direction_ = false;
        break;
    }
  }
}

//------------------------------------------------------------------------------
double MazeSolution::calculateDistance(double x0, double y0, double x1, double y1)
{
  return sqrt((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0));
}

//------------------------------------------------------------------------------
double MazeSolution::quaternionToEuler(double z, double w)
{
  double sqw, sqz;
  sqw = w*w;
  sqz = z*z;
  return atan2(2.0*(z*w),(-sqz + sqw));
}

//------------------------------------------------------------------------------
void MazeSolution::getPropertyCallback(const ros::TimerEvent& e)
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

  ROS_INFO("%.2f, %.2f, %.2f, [%d,%d,%d,%d]",pos_x_, pos_y_, pos_th_, bumper_left_pressed_, bumper_center_pressed_, bumper_right_pressed_, change_direction_);
}

//------------------------------------------------------------------------------
void MazeSolution::pubVel(double linear, double angular)
{
  geometry_msgs::Twist vel;
  vel.linear.x  = linear;
  vel.angular.z = angular;

  cmd_vel_publisher_.publish(vel);
}

//------------------------------------------------------------------------------
void MazeSolution::checkCallback(const ros::TimerEvent& e)
{
  // Velocity commands
  geometry_msgs::TwistPtr cmd_vel_msg_ptr;
  cmd_vel_msg_ptr.reset(new geometry_msgs::Twist());

  if (change_direction_)
  {
  }
  else
  {
  }
}

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  //Init ROS node
  ros::init(argc, argv, "maze_solution");
  MazeSolution ms;
  ros::spin();
  return 0;
}

//------------------------------------------------------------------------------
//EOF
