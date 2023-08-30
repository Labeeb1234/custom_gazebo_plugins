#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include <diff_bot_plugin/diff_bot_plugin.hpp>

#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sdf/sdf.hh>


// #ifdef NO_ERROR
// // NO_ERROR is a macro defined in Windows that's used as an enum in tf2
// #undef NO_ERROR
// #endif

#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <utility>

namespace gazebo
{

class GazeboRosDiffBotPrivate
{
public:

  /// Indicates which wheel
  enum
  {
    /// Right wheel
    RIGHT = 0,

    /// Left wheel
    LEFT = 1,
  };

  /// Callback to be called at every simulation iteration.
  /// \param[in] _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & _info);

  /// Callback when a velocity command is received.
  /// \param[in] _msg Twist command message.
  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);

  /// Update wheel velocities according to latest target velocities.
  void UpdateWheelVelocities();

  /// Publish trasforms for the wheels
  /// \param[in] _current_time Current simulation time
  void PublishWheelsTf(const gazebo::common::Time & _current_time);


  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Subscriber to command velocities
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;


  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// Distance between the wheels, in meters.
  std::vector<double> wheel_separation_;

  /// Diameter of wheels, in meters.
  std::vector<double> wheel_diameter_;

  /// Maximum wheel torque, in Nm.
  double max_wheel_torque_;

  /// Maximum wheel acceleration
  double max_wheel_accel_;

  /// Desired wheel speed.
  std::vector<double> desired_wheel_speed_;

  /// Speed sent to wheel.
  std::vector<double> wheel_speed_instr_;

  /// Pointers to wheel joints.
  std::vector<gazebo::physics::JointPtr> joints_;

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// Linear velocity in X received on command (m/s).
  double target_x_{0.0};

  /// Angular velocity in Z received on command (rad/s).
  double target_rot_{0.0};

  /// Update period in seconds.
  double update_period_;

  /// Last update time.
  gazebo::common::Time last_update_time_;

  /// True to publish wheel-to-base transforms.
  bool publish_wheel_tf_;

  /// Store number of wheel pairs
  unsigned int num_wheel_pairs_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  std::string robot_base_frame_;



};


GazeboRosDiffBot::GazeboRosDiffBot()
: impl_(std::make_unique<GazeboRosDiffBotPrivate>()){}

GazeboRosDiffBot::~GazeboRosDiffBot(){}

void GazeboRosDiffBot::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{

    impl_->model_ = _model;

  // Initialize ROS node
    impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

    // Get QoS profiles
    const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

    if (impl_->num_wheel_pairs_ < 1) 
    {
        impl_->num_wheel_pairs_ = 1;
        RCLCPP_WARN(impl_->ros_node_->get_logger(),"Drive requires at least one pair of wheels. Setting [num_wheel_pairs] to 1");
    }

    // Get number of wheel pairs in the model
    impl_->num_wheel_pairs_ = static_cast<unsigned int>(_sdf->Get<int>("num_wheel_pairs",1).first);

  // Dynamic properties
    impl_->max_wheel_accel_ = _sdf->Get<double>("max_wheel_acceleration", 0.0).first;
    impl_->max_wheel_torque_ = _sdf->Get<double>("max_wheel_torque", 5.0).first;

    // Get joints and Kinematic properties
    std::vector<gazebo::physics::JointPtr> left_joints, right_joints;

    for (auto left_joint_elem = _sdf->GetElement("left_joint"); left_joint_elem != nullptr;left_joint_elem = left_joint_elem->GetNextElement("left_joint"))
    {
        auto left_joint_name = left_joint_elem->Get<std::string>();
        auto left_joint = _model->GetJoint(left_joint_name);

        if (!left_joint) 
        {
            RCLCPP_ERROR(impl_->ros_node_->get_logger(),"Joint [%s] not found, plugin will not work.", left_joint_name.c_str());
            impl_->ros_node_.reset();
            return;
        }
        left_joint->SetParam("fmax", 0, impl_->max_wheel_torque_);
        left_joints.push_back(left_joint);
    }

    for (auto right_joint_elem = _sdf->GetElement("right_joint"); right_joint_elem != nullptr;right_joint_elem = right_joint_elem->GetNextElement("right_joint"))
    {
        auto right_joint_name = right_joint_elem->Get<std::string>();
        auto right_joint = _model->GetJoint(right_joint_name);

        if (!right_joint) {
        RCLCPP_ERROR(impl_->ros_node_->get_logger(),"Joint [%s] not found, plugin will not work.", right_joint_name.c_str());
        impl_->ros_node_.reset();
        return;
        }
        right_joint->SetParam("fmax", 0, impl_->max_wheel_torque_);
        right_joints.push_back(right_joint);
    }

    if (left_joints.size() != right_joints.size() || left_joints.size() != impl_->num_wheel_pairs_)
    {
        RCLCPP_ERROR(impl_->ros_node_->get_logger(),"Inconsistent number of joints specified. Plugin will not work.");
        impl_->ros_node_.reset();
        return;
    }

    unsigned int index; // unsigned is optional its juts that the index is best to be kept >= 0
    // sending joints to joints_ vector storage space
    for (index = 0; index < impl_->num_wheel_pairs_; index++) 
    {
      impl_->joints_.push_back(right_joints[index]);
      impl_->joints_.push_back(left_joints[index]);
    }
    index = 0;
    impl_->wheel_separation_.assign(impl_->num_wheel_pairs_, 0.34);
    for (auto wheel_separation = _sdf->GetElement("wheel_separation"); wheel_separation != nullptr;wheel_separation = wheel_separation->GetNextElement("wheel_separation"))
    {
        if (index >= impl_->num_wheel_pairs_) 
        {
            RCLCPP_WARN(impl_->ros_node_->get_logger(), "Ignoring rest of specified <wheel_separation>");
            break;
        }
        impl_->wheel_separation_[index] = wheel_separation->Get<double>();
        RCLCPP_INFO(impl_->ros_node_->get_logger(),"Wheel pair %i separation set to [%fm]", index + 1, impl_->wheel_separation_[index]);
        index++;
    }

    index = 0;
    impl_->wheel_diameter_.assign(impl_->num_wheel_pairs_, 0.15);
    for (auto wheel_diameter = _sdf->GetElement("wheel_diameter"); wheel_diameter != nullptr;
        wheel_diameter = wheel_diameter->GetNextElement("wheel_diameter"))
    {
        if (index >= impl_->num_wheel_pairs_) 
        {
            RCLCPP_WARN(impl_->ros_node_->get_logger(), "Ignoring rest of specified <wheel_diameter>");
            break;
        }
        impl_->wheel_diameter_[index] = wheel_diameter->Get<double>();
        RCLCPP_INFO(impl_->ros_node_->get_logger(),"Wheel pair %i diameter set to [%fm]", index + 1, impl_->wheel_diameter_[index]);
        index++;
    }

    // seting desired and intial wheel velocities to zero
    impl_->wheel_speed_instr_.assign(2 * impl_->num_wheel_pairs_, 0);
    impl_->desired_wheel_speed_.assign(2 * impl_->num_wheel_pairs_, 0);

    // Update rate
    auto update_rate = _sdf->Get<double>("update_rate", 100.0).first;
    if (update_rate > 0.0) 
    {
        impl_->update_period_ = 1.0 / update_rate;
    } 
    else 
    {
        impl_->update_period_ = 0.0;
    }
    // previous simulation time 
    impl_->last_update_time_ = _model->GetWorld()->SimTime();
    // printf("previous sim time: [%f]\n", impl_->last_update_time_);

    // creating s subscription to twist msgs topic /cmd_vel
    impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
        std::bind(&GazeboRosDiffBotPrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Subscribed to [%s]",impl_->cmd_vel_sub_->get_topic_name());

    impl_->robot_base_frame_ = _sdf->Get<std::string>("robot_base_frame", "base_link").first;

    // Listen to the update event (broadcast every simulation iteration)
    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboRosDiffBotPrivate::OnUpdate, impl_.get(), std::placeholders::_1));

}

void GazeboRosDiffBotPrivate::OnUpdate(const gazebo::common::UpdateInfo &_info)
{
    #ifdef IGN_PROFILER_ENABLE
        IGN_PROFILE("GazeboRosDiffBotPrivate::OnUpdate");
    #endif
    double seconds_since_last_update = (_info.simTime - last_update_time_).Double();

    if (seconds_since_last_update < update_period_) 
    {
        return;
    }

    // #ifdef IGN_PROFILER_ENABLE
    //   IGN_PROFILE_END();
    //   IGN_PROFILE_BEGIN("PublishWheelsTf");
    // #endif
    // if (publish_wheel_tf_) 
    // {
    //   PublishWheelsTf(_info.simTime);
    // }

    #ifdef IGN_PROFILER_ENABLE
        IGN_PROFILE_END();
        IGN_PROFILE_BEGIN("UpdateWheelVelocities");
    #endif
    // Update robot in case new velocities have been requested
    UpdateWheelVelocities();
    #ifdef IGN_PROFILER_ENABLE
        IGN_PROFILE_END();
    #endif
    // current speed of wheels
    std::vector<double> current_speed(2 * num_wheel_pairs_);

    for(unsigned int i=0; i < num_wheel_pairs_; i++)
    {
        current_speed[2*i+LEFT] = joints_[2*i+LEFT]->GetVelocity(0)*(wheel_diameter_[i]/2.0);
        // current_speed[2*i+LEFT] = joints_[2*i+LEFT]->GetVelocity(0); // in [rad/s]
        current_speed[2*i+RIGHT] = joints_[2*i+RIGHT]->GetVelocity(0)*(wheel_diameter_[i]/2.0);
    } 
      // If max_accel == 0, or target speed is reached
  for (unsigned int i = 0; i < num_wheel_pairs_; ++i) 
  {
    // when max_accel == 0 or if the wheel velocity reaches its desired value
    if (max_wheel_accel_ == 0 ||
      ((fabs(desired_wheel_speed_[2 * i + LEFT] - current_speed[2 * i + LEFT]) < 0.01) &&
      (fabs(desired_wheel_speed_[2 * i + RIGHT] - current_speed[2 * i + RIGHT]) < 0.01)))
    {
      joints_[2 * i + LEFT]->SetParam("vel", 0, desired_wheel_speed_[2 * i + LEFT] / (wheel_diameter_[i] / 2.0));
      joints_[2 * i + RIGHT]->SetParam("vel", 0, desired_wheel_speed_[2 * i + RIGHT] / (wheel_diameter_[i] / 2.0));
    //   joints_[2 * i + LEFT]->SetParam("vel", 0, desired_wheel_speed_[2 * i + LEFT]; 
    //   joints_[2 * i + RIGHT]->SetParam("vel", 0, desired_wheel_speed_[2 * i + RIGHT];
    } 
    else 
    {
      if (desired_wheel_speed_[2 * i + LEFT] >= current_speed[2 * i + LEFT]) 
      {
        wheel_speed_instr_[2 * i + LEFT] += fmin(desired_wheel_speed_[2 * i + LEFT]-current_speed[2 * i + LEFT], max_wheel_accel_ * seconds_since_last_update);
      } 
      else 
      {
        wheel_speed_instr_[2 * i + LEFT] += fmax(desired_wheel_speed_[2 * i + LEFT]-current_speed[2 * i + LEFT], -max_wheel_accel_ * seconds_since_last_update);
      }

      if (desired_wheel_speed_[2 * i + RIGHT] > current_speed[2 * i + RIGHT]) 
      {
        wheel_speed_instr_[2 * i + RIGHT] += fmin(desired_wheel_speed_[2 * i + RIGHT]-current_speed[2 * i + RIGHT], max_wheel_accel_ * seconds_since_last_update);
      } 
      else 
      {
        wheel_speed_instr_[2 * i + RIGHT] += fmax(desired_wheel_speed_[2 * i + RIGHT]-current_speed[2 * i + RIGHT], -max_wheel_accel_ * seconds_since_last_update);
      }

      joints_[2 * i + LEFT]->SetParam("vel", 0, wheel_speed_instr_[2 * i + LEFT] / (wheel_diameter_[i] / 2.0));
      joints_[2 * i + RIGHT]->SetParam("vel", 0, wheel_speed_instr_[2 * i + RIGHT] / (wheel_diameter_[i] / 2.0));
    //   joints_[2 * i + LEFT]->SetParam("vel", 0, wheel_speed_instr_[2 * i + LEFT];
    //   joints_[2 * i + RIGHT]->SetParam("vel", 0, wheel_speed_instr_[2 * i + RIGHT;
    }
  }

  last_update_time_ = _info.simTime;

}

void GazeboRosDiffBotPrivate::UpdateWheelVelocities()
{
    // kinematics (with desired wheel velocities)
    std::lock_guard<std::mutex> scoped_lock(lock_);

    double u = target_x_;
    double r = target_rot_;

  for (unsigned int i = 0; i < num_wheel_pairs_; ++i) {
    desired_wheel_speed_[2 * i + LEFT] = u - (r * wheel_separation_[i] / 2.0); // in m/s
    desired_wheel_speed_[2 * i + RIGHT] = u + (r * wheel_separation_[i] / 2.0); // in m/s
  }

}

void GazeboRosDiffBotPrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
{
    std::lock_guard<std::mutex> scoped_lock(lock_);
    target_x_ = _msg->linear.x; 
    target_rot_= _msg->angular.z;
}

void GazeboRosDiffBot::Reset()
{
  impl_->last_update_time_ = impl_->joints_[GazeboRosDiffBotPrivate::LEFT]->GetWorld()->SimTime();

  for(unsigned int i=0; i<impl_->num_wheel_pairs_;i++)
  {
    if(impl_->joints_[2*i + GazeboRosDiffBotPrivate::LEFT] && impl_->joints_[2*i+GazeboRosDiffBotPrivate::RIGHT])
    {
      impl_->joints_[2*i + GazeboRosDiffBotPrivate::LEFT]->SetParam("fmax", 0, impl_->max_wheel_torque_);
      impl_->joints_[2*i + GazeboRosDiffBotPrivate::RIGHT]->SetParam("fmax", 0, impl_->max_wheel_torque_);
    }

    impl_->target_x_ = 0.0;
    impl_->target_rot_ = 0.0;
  }

}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosDiffBot)
}
