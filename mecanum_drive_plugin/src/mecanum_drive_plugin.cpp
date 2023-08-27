#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include <mecanum_drive_plugin/mecanum_drive_plugin.hpp>

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
class GazeboRosMecanumDrivePrivate
{
    public:
    // indicate where the odom is calculate from
    enum OdomSource
    {
      ENCODER = 0,
      WORLD = 1,
    };


    /// Callback to be called at every simulation iteration.
    /// \param[in] _info Updated simulation info.
    void OnUpdate(const gazebo::common::UpdateInfo & _info);

    /// Callback when a velocity command is received.
    /// \param[in] _msg Twist command message.
    void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);

    /// Update wheel velocities according to latest target velocities.
    void UpdateWheelVelocities();


    /// Odometry updation
    /// \param[in] _current_time Twist command message.
    void UpdateEncoder(const gazebo::common::Time &_current_time);

    // update odom according to world
    void UpdateOdometryWorld();

    /// to publish odom transforms
    /// \param[in] _current_time Twist command message.
    void PublishOdomteryTf(const gazebo::common::Time &_current_time);


    /// to publish wheel transforms
    /// \param[in] _current_time Twist command message.
    void PublishWheelTf(const gazebo::common::Time &_current_time);

    /// to publish odom msgs calulated in the update Encoder funtion
    /// \param[in] _current_time Twist command message.
    void PublishOdometryMsg(const gazebo::common::Time &_current_time);
    
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;


    /// Connection to event called at every world iteration.
    gazebo::event::ConnectionPtr update_connection_;

    // std::vector<double> wheel_separation_;
    // std::vector<double> wheel_sep_length_;
    // std::vector<double> wheel_diameter_;
    double wheel_separation_;
    double wheel_sep_length_;
    double wheel_diameter_;
    double max_wheel_torque_; // torque in [Nm]
    double max_wheel_accel_;

    /// Desired wheel speed.
    std::vector<double> desired_wheel_speed_;

    /// Speed sent to wheel.
    std::vector<double> wheel_speed_instr_;

    /// Pointers to wheel joints.
    std::vector<gazebo::physics::JointPtr> joints_;
    /// Pointer to model.
    gazebo::physics::ModelPtr model_;

    // pointer to brodcast transform msgs
    std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

    std::mutex lock_;
    double target_x_{0.0}; // in m/s
    double target_y_{0.0}; // in m/s
    double target_rot_{0.0}; // in rad/s

    unsigned int num_wheel_;
    unsigned int num_wheel_pairs_;

    // time params and msgs update frequency
    double update_period_;

    // previous update time (gazebo clock) 
    gazebo::common::Time last_update_time_;
    // variable to store encoder readings from position feedback 
    geometry_msgs::msg::Pose2D pose_endcoder_;

    OdomSource odom_source;
    //variable to keep track of odom msgs at each time_step
    nav_msgs::msg::Odometry odom_msg_;


    std::string robot_base_frame_;

    bool publish_odom_;
    bool publish_odom_tf_;
    bool publish_wheel_tf_;

    double covarience_[3];

};


GazeboRosMecanumDrive::GazeboRosMecanumDrive()
: impl_(std::make_unique<GazeboRosMecanumDrivePrivate>()){}

GazeboRosMecanumDrive::~GazeboRosMecanumDrive(){}

void GazeboRosMecanumDrive::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    impl_->model_ = _model;

  // Initialize ROS node
    impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

    // Get QoS profiles
    const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();
    if(impl_->num_wheel_ < 4)
    {
        impl_->num_wheel_ = 4;
        RCLCPP_WARN(impl_->ros_node_->get_logger(), "This plugin requires exactly [4 wheels] to work so setting [num_wheel]=4\n");
    }

    // loading params part-1

    impl_->num_wheel_ = static_cast<unsigned int>(_sdf->Get<int>("num_wheel"));
    impl_->max_wheel_accel_ = _sdf->Get<double>("max_wheel_acceleration");
    impl_->max_wheel_torque_ = _sdf->Get<double>("max_wheel_torque");
    
    // loading joints
    std::vector<gazebo::physics::JointPtr> front_left_joints, front_right_joints, back_left_joints, back_right_joints;
    for (auto front_left_joint_elem = _sdf->GetElement("front_left_joint"); front_left_joint_elem != nullptr;front_left_joint_elem = front_left_joint_elem->GetNextElement("front_left_joint"))
    {
            auto front_left_joint_name = front_left_joint_elem->Get<std::string>();
            auto front_left_joint = _model->GetJoint(front_left_joint_name);

            if (!front_left_joint) 
            {
                RCLCPP_ERROR(impl_->ros_node_->get_logger(),"Joint [%s] not found, plugin will not work.", front_left_joint_name.c_str());
                impl_->ros_node_.reset();
                return;
            }
            front_left_joint->SetParam("fmax", 0, impl_->max_wheel_torque_);
            front_left_joints.push_back(front_left_joint);
    }
    for (auto front_right_joint_elem = _sdf->GetElement("front_right_joint"); front_right_joint_elem != nullptr;front_right_joint_elem = front_right_joint_elem->GetNextElement("front_right_joint"))
    {
            auto front_right_joint_name = front_right_joint_elem->Get<std::string>();
            auto front_right_joint = _model->GetJoint(front_right_joint_name);

            if (!front_right_joint) 
            {
                RCLCPP_ERROR(impl_->ros_node_->get_logger(),"Joint [%s] not found, plugin will not work.", front_right_joint_name.c_str());
                impl_->ros_node_.reset();
                return;
            }
            front_right_joint->SetParam("fmax", 0, impl_->max_wheel_torque_);
            front_right_joints.push_back(front_right_joint);
    }
    for (auto back_left_joint_elem = _sdf->GetElement("back_left_joint"); back_left_joint_elem != nullptr;back_left_joint_elem = back_left_joint_elem->GetNextElement("back_left_joint"))
    {
            auto back_left_joint_name = back_left_joint_elem->Get<std::string>();
            auto back_left_joint = _model->GetJoint(back_left_joint_name);

            if (!back_left_joint) 
            {
                RCLCPP_ERROR(impl_->ros_node_->get_logger(),"Joint [%s] not found, plugin will not work.", back_left_joint_name.c_str());
                impl_->ros_node_.reset();
                return;
            }
            back_left_joint->SetParam("fmax", 0, impl_->max_wheel_torque_);
            back_left_joints.push_back(back_left_joint);
    }
    for (auto back_right_joint_elem = _sdf->GetElement("back_right_joint"); back_right_joint_elem != nullptr;back_right_joint_elem = back_right_joint_elem->GetNextElement("back_right_joint"))
    {
            auto back_right_joint_name = back_right_joint_elem->Get<std::string>();
            auto back_right_joint = _model->GetJoint(back_right_joint_name);

            if (!back_right_joint) 
            {
                RCLCPP_ERROR(impl_->ros_node_->get_logger(),"Joint [%s] not found, plugin will not work.", back_right_joint_name.c_str());
                impl_->ros_node_.reset();
                return;
            }
            back_right_joint->SetParam("fmax", 0, impl_->max_wheel_torque_);
            back_right_joints.push_back(back_right_joint);
    
    }


    unsigned int index; // unsigned is optional its juts that the index is best to be kept >= 0
    // sending joints to joints_ vector storage space
    for (index = 0; index < impl_->num_wheel_/4; index++) 
    {
      impl_->joints_.push_back(front_left_joints[index]);
      impl_->joints_.push_back(front_right_joints[index]);
      impl_->joints_.push_back(back_left_joints[index]);
      impl_->joints_.push_back(back_right_joints[index]);
    }

    impl_->wheel_separation_ = _sdf->Get<double>("wheel_separation");
    impl_->wheel_sep_length_ = _sdf->Get<double>("wheel_separation_length");
    impl_->wheel_diameter_ = _sdf->Get<double>("wheel_diameter");

    // setting desired and intial wheel velocities to zero
    impl_->wheel_speed_instr_.assign(impl_->num_wheel_, 0);
    impl_->desired_wheel_speed_.assign(impl_->num_wheel_, 0);

    // setting the Update rate
    auto update_rate = _sdf->Get<double>("update_rate");
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
    printf("previous sim time: [%f]\n", impl_->last_update_time_);

    // creating s subscription to twist msgs topic /cmd_vel
    impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
        std::bind(&GazeboRosMecanumDrivePrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Subscribed to [%s]",impl_->cmd_vel_sub_->get_topic_name());

    impl_->robot_base_frame_ = _sdf->Get<std::string>("robot_base_frame", "base_link").first;

    // Listen to the update event (broadcast every simulation iteration)
    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboRosMecanumDrivePrivate::OnUpdate, impl_.get(), std::placeholders::_1));

}

void GazeboRosMecanumDrivePrivate::OnUpdate(const gazebo::common::UpdateInfo &_info)
{
    #ifdef IGN_PROFILER_ENABLE
        IGN_PROFILE("GazeboRosMecanumDrivePrivate::OnUpdate");
    #endif
    double seconds_since_last_update = (_info.simTime - last_update_time_).Double();

    if (seconds_since_last_update < update_period_) 
    {
        return;
    }

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
    std::vector<double> current_speed(num_wheel_);

    for(unsigned int i=0; i < num_wheel_; i++)
    {
        current_speed[i] = joints_[i]->GetVelocity(0)*(wheel_diameter_/2.0);
        // current_speed[2*i+LEFT] = joints_[2*i+LEFT]->GetVelocity(0); // in [rad/s]
    }

      // If max_accel == 0, or target speed is reached
  for (unsigned int i = 0; i < num_wheel_; i++) 
  {
    // when max_accel == 0 or if the wheel velocity reaches its desired value
    if (max_wheel_accel_ == 0 || (fabs(desired_wheel_speed_[i] - current_speed[i]) < 0.01))
    {
        joints_[i]->SetParam("vel", 0, desired_wheel_speed_[i] / (wheel_diameter_ / 2.0));
    //   joints_[2 * i + LEFT]->SetParam("vel", 0, desired_wheel_speed_[2 * i + LEFT]; 
    //   joints_[2 * i + RIGHT]->SetParam("vel", 0, desired_wheel_speed_[2 * i + RIGHT];
    } 
    else 
    {
      if (desired_wheel_speed_[i] >= current_speed[i]) 
      {
        wheel_speed_instr_[i] += fmin(desired_wheel_speed_[i]-current_speed[i], max_wheel_accel_ * seconds_since_last_update);
      } 
      else 
      {
        wheel_speed_instr_[i] += fmax(desired_wheel_speed_[i]-current_speed[i], -max_wheel_accel_ * seconds_since_last_update);
      }

      joints_[i]->SetParam("vel", 0, wheel_speed_instr_[i] / (wheel_diameter_ / 2.0));
    //   joints_[2 * i + LEFT]->SetParam("vel", 0, wheel_speed_instr_[2 * i + LEFT];
    //   joints_[2 * i + RIGHT]->SetParam("vel", 0, wheel_speed_instr_[2 * i + RIGHT;
    }
  }

  last_update_time_ = _info.simTime;

}

// Robot wheel kinematics constrain implementation
void GazeboRosMecanumDrivePrivate::UpdateWheelVelocities()
{
    // kinematics (with desired wheel velocities)
    std::lock_guard<std::mutex> scoped_lock(lock_);

    double u = target_x_;
    double v = target_y_;
    double r = target_rot_;

    desired_wheel_speed_[0] = u + v + r *((wheel_sep_length_-wheel_separation_) / 2.0); // in m/s
    desired_wheel_speed_[1] = u - v + r *((wheel_sep_length_-wheel_separation_) / 2.0); // in m/s
    desired_wheel_speed_[2] = u + v - r *((wheel_sep_length_-wheel_separation_) / 2.0); // in m/s
    desired_wheel_speed_[3] = u - v - r *((wheel_sep_length_-wheel_separation_) / 2.0); // in m/s


}

// cmd_vel reciever function
void GazeboRosMecanumDrivePrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
{
    std::lock_guard<std::mutex> scoped_lock(lock_);
    target_x_ = _msg->linear.x;
    target_y_ = _msg->linear.y; 
    target_rot_= _msg->angular.z;
}


// Reset function
void GazeboRosMecanumDrive::Reset()
{
  impl_->last_update_time_ = impl_->joints_[3]->GetWorld()->SimTime();

  for(unsigned int i=0; i<impl_->num_wheel_pairs_;i++)
  {
    if(impl_->joints_[i])
    {
      impl_->joints_[i]->SetParam("fmax", 0, impl_->max_wheel_torque_);

    }

    impl_->target_x_ = 0.0;
    impl_->target_y_ = 0.0;
    impl_->target_rot_ = 0.0;
  }

}


GZ_REGISTER_MODEL_PLUGIN(GazeboRosMecanumDrive)
}
