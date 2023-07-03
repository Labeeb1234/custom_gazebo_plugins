#include <memory> 
#include <queue>
#include <string>
#include <utility>
#include <vector>
#include <cmath>

#include "diff_bot_controller/diff_bot_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace
{
    constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel_unstamped";
    constexpr auto DEFAULT_ODOM_TOPIC = "~/odom";
    constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
}  


namespace diff_bot_controller
{
    using namespace std::chrono_literals;
    using controller_interface::interface_configuration_type;
    using controller_interface::InterfaceConfiguration;
    using hardware_interface::HW_IF_POSITION;
    using hardware_interface::HW_IF_VELOCITY;
    using lifecycle_msgs::msg::State;

    DiffBotController::DiffBotController() 
        : controller_interface::ControllerInterface()
        , velocity_command_subscriber_(nullptr)
        , velocity_command_ptr_(nullptr){}

    controller_interface::CallbackReturn DiffBotController::on_init()
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    InterfaceConfiguration DiffBotController::command_interface_configuration() const
    {
        std::vector<std::string> conf_names;
        
        RCLCPP_INFO(get_node()->get_logger(), "Configure DiffBotController\n");
        for(const auto &joint_name: wheel_joint_names_)
        {
            conf_names.push_back(joint_name+ "/" + HW_IF_VELOCITY);

        }
        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};

    }
    
    InterfaceConfiguration DiffBotController::state_interface_configuration() const 
    {
        std::vector<std::string> conf_names;

        for (const auto & joint_name : wheel_joint_names_) 
        {
        
            conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
        }
        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }
    
    controller_interface::CallbackReturn DiffBotController::on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Configure DiffBotController\n");

        wheel_joint_names_ = get_node()->get_parameter("wheel_joint_names").as_string_array();

        if(wheel_joint_names_.size() != 2)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "number wheels in params not matching the number of wheels on the model\n");
            return controller_interface::CallbackReturn::ERROR;
        }

        if(wheel_joint_names_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "no wheels!\n");
            return controller_interface::CallbackReturn::ERROR;
        }
        
        wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
        wheel_separation_ = get_node()->get_parameter("wheel_separation").as_double();

        odom_params_.odom_frame_id = get_node()->get_parameter("odom_frame_id").as_string();
        odom_params_.base_frame_id = get_node()->get_parameter("base_frame_id").as_string();
        odom_params_.open_loop = get_node()->get_parameter("open_loop").as_bool();
        odom_params_.enable_odom_tf = get_node()->get_parameter("enable_odom_tf").as_bool();

        // initializing odom publisher
        odom_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(DEFAULT_ODOM_TOPIC, rclcpp::SystemDefaultsQoS());

        publish_rate_ = get_node()->get_parameter("publish_rate").as_double();
        publish_period_ = rclcpp::Duration::from_seconds(1.0/publish_rate_);
        previous_publish_timestamp_ = get_node()->get_clock()->now();

        // creating a subcriber to subcriber to Twist(unstamped) topic
        velocity_command_subscriber_ = get_node()->create_subscription<Twist>(DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),[this](Twist::SharedPtr msg)
            {
                if(!subscriber_is_active_)
                {
                    RCLCPP_WARN(get_node()->get_logger(),"Can't accept new msgs subscriber is not active\n");
                    return;
                }
                
                velocity_command_ptr_.writeFromNonRT(msg);
                //velocity_command_ptr_.get(msg);
                // fake headers
                //std::shared_ptr<Twist> twist_stamped;
                //velocity_command_ptr_.get(twist_stamped);
                //twist_stamped->twist = *msg;
                //twist_stamped->header.stamp = get_node()->get_clock()->now();
                
            });

        return controller_interface::CallbackReturn::SUCCESS;

    }

    controller_interface::CallbackReturn DiffBotController::on_activate(const rclcpp_lifecycle::State &)
    {

        if(wheel_joint_names_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "wheel joint name are empty\n");
            return controller_interface::CallbackReturn::ERROR;
        }

        registered_wheel_handles_.reserve(wheel_joint_names_.size());
        {
            for(const auto &wheel_joint_name: wheel_joint_names_)
            {
                const auto interface_name = HW_IF_VELOCITY;
                const auto state_handle = std::find_if(
                    state_interfaces_.cbegin(), state_interfaces_.cend(), [&wheel_joint_name, &interface_name](const auto &interface)
                    {
                        return interface.get_prefix_name() == wheel_joint_name && interface.get_interface_name() == interface_name;

                    });

                if(state_handle == state_interfaces_.cend())
                {
                    RCLCPP_ERROR(get_node()->get_logger(), "Unable to get joint state handle for %s", wheel_joint_name.c_str());
                    return controller_interface::CallbackReturn::ERROR;
                }

                const auto command_handle = std::find_if(
                    command_interfaces_.begin(), command_interfaces_.end(), [&wheel_joint_name](const auto &interface)
                    {
                        return interface.get_prefix_name() == wheel_joint_name && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
                    });
                
                if(command_handle == command_interfaces_.end())
                {
                    RCLCPP_ERROR(get_node()->get_logger(), "Unable to get joint command handle for %s", wheel_joint_name.c_str());
                    return controller_interface::CallbackReturn::ERROR;
                }

                registered_wheel_handles_.emplace_back(WheelHandle{std::ref(*state_handle), std::ref(*command_handle)});

                RCLCPP_INFO(get_node()->get_logger(),"Got command interface: %s", command_handle->get_name().c_str());
                RCLCPP_INFO(get_node()->get_logger(),"Got state interface: %s", state_handle->get_name().c_str());
       
            }

            subscriber_is_active_ = true;
            RCLCPP_INFO(get_node()->get_logger(),"Subscriber and publisher are active now\n");
            previous_updated_timestamp_ = get_node()->get_clock()->now();
            return controller_interface::CallbackReturn::SUCCESS;
        }

    }

    controller_interface::CallbackReturn DiffBotController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_DEBUG(get_node()->get_logger(), "Called on_deactivate\n");
        subscriber_is_active_ = false;
        return controller_interface::CallbackReturn::SUCCESS;

    }

    controller_interface::CallbackReturn  DiffBotController::on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_DEBUG(get_node()->get_logger(),"Called on_cleanup\n");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    // if errors occur what to do ?
    controller_interface::CallbackReturn  DiffBotController::on_error(const rclcpp_lifecycle::State &)
    {
        RCLCPP_DEBUG(get_node()->get_logger(),"Called on_error\n");
        return controller_interface::CallbackReturn::SUCCESS; 
    }

    // shutdown phase of the controllers
    controller_interface::CallbackReturn  DiffBotController::on_shutdown(const rclcpp_lifecycle::State &)
    {
        RCLCPP_DEBUG(get_node()->get_logger(),"Called on_shutdown\n");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type DiffBotController::update(const rclcpp::Time &, const rclcpp::Duration &)
    {
        
        RCLCPP_INFO(get_node()->get_logger(), "Entered the update phase: working?\n");
        // to get previous velocity command 
        auto velocity_command = velocity_command_ptr_.readFromRT();
        if (!velocity_command || !(*velocity_command)) 
        {
            return controller_interface::return_type::OK;
        }
        //std::shared_ptr<Twist> velocity_command;
        const auto msg_linear = (*velocity_command)-> linear;
        const auto msg_angular = (*velocity_command)->angular;
        // inverse kinematics of the 4-omni-wheel-robot
        double v_x_des = msg_linear.x;
        //double v_y_des = msg_linear.y;
        double omega_des = msg_angular.z;

        std::vector<double> wheel_velocity(2);
        wheel_velocity[0] = (2*v_x_des - omega_des*wheel_separation_)/(2*wheel_radius_);
        wheel_velocity[1] = (2*v_x_des + omega_des*wheel_separation_)/(2*wheel_radius_);

        registered_wheel_handles_[0].velocity_command.get().set_value(wheel_velocity[0]);
        registered_wheel_handles_[1].velocity_command.get().set_value(wheel_velocity[1]);
        
        const auto current_time = get_node()->get_clock()->now();
        const rclcpp::Duration dt = current_time - previous_updated_timestamp_;

        struct RobotPose
        {
            double pose_x = 0;
            double pose_y = 0;
            double pose_th = 0;

        }robot_pose;
        // do odom calculations here
        if(odom_params_.open_loop == true)
        {
            // Euler integration
            robot_pose.pose_x = robot_pose.pose_x + dt.seconds()*(v_x_des*cos(robot_pose.pose_th));
            robot_pose.pose_y = robot_pose.pose_y + dt.seconds()*(v_x_des*sin(robot_pose.pose_th));
            robot_pose.pose_th = robot_pose.pose_th + dt.seconds()*(omega_des);

        }
        // put the current time as previous time as the odom calculations are done 
        previous_updated_timestamp_ = current_time; 
        



        return controller_interface::return_type::OK;
    }
    DiffBotController::~DiffBotController() {}

}

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  diff_bot_controller::DiffBotController, controller_interface::ControllerInterface)



