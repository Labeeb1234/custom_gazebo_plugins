#ifndef GAZEBO__GAZEBO_ROS_DIFF_BOT_HPP_ 
#define GAZEBO__GAZEBO_ROS_DIFF_BOT_HPP_


#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo
{

class GazeboRosDiffBotPrivate;

class GazeboRosDiffBot : public gazebo::ModelPlugin
{   
    public:
        GazeboRosDiffBot();

        ~GazeboRosDiffBot();

    protected:
        void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

        void Reset() override;

    
    private:
        std::unique_ptr<GazeboRosDiffBotPrivate> impl_;


};
}
#endif