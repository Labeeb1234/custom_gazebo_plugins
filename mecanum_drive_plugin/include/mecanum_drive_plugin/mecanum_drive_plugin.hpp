#ifndef GAZEBO__GAZEBO_ROS_MECANUM_DRIVE_HPP_ 
#define GAZEBO__GAZEBO_ROS_MECANUM_DRIVE_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo
{
class GazeboRosMecanumDrivePrivate;

class GazeboRosMecanumDrive : public gazebo::ModelPlugin
{
    public:
    GazeboRosMecanumDrive();

    ~GazeboRosMecanumDrive();

    protected:
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    void Reset() override;

    private:
    std::unique_ptr<GazeboRosMecanumDrivePrivate> impl_;


};
}


#endif