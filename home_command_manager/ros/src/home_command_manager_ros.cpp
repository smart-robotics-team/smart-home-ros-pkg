// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <home_command_manager/home_command_managerConfig.h>

// ROS message includes
#include <std_msgs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>

// other includes
#include <home_command_manager_common.cpp>


class home_command_manager_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<home_command_manager::home_command_managerConfig> server;
    dynamic_reconfigure::Server<home_command_manager::home_command_managerConfig>::CallbackType f;

    ros::Publisher command1_;
    ros::Publisher command2_;
    ros::Publisher command3_;
    ros::Publisher command4_;
    ros::Publisher command5_;
    ros::Publisher command6_;
    ros::Publisher command7_;
    ros::Publisher command8_;
    ros::Publisher command9_;
    ros::Publisher command10_;
    ros::Publisher command11_;
    ros::Publisher command12_;
    ros::Publisher command13_;
    ros::Publisher command14_;
    ros::Subscriber command_;

    home_command_manager_data component_data_;
    home_command_manager_config component_config_;
    home_command_manager_impl component_implementation_;

    home_command_manager_ros() : np_("~")
    {
        f = boost::bind(&home_command_manager_ros::configure_callback, this, _1, _2);
        server.setCallback(f);


        command1_ = n_.advertise<std_msgs::Empty>("command1", 1);
        command2_ = n_.advertise<std_msgs::Empty>("command2", 1);
        command3_ = n_.advertise<std_msgs::Empty>("command3", 1);
        command4_ = n_.advertise<std_msgs::Empty>("command4", 1);
        command5_ = n_.advertise<std_msgs::Empty>("command5", 1);
        command6_ = n_.advertise<std_msgs::Empty>("command6", 1);
        command7_ = n_.advertise<std_msgs::Empty>("command7", 1);
        command8_ = n_.advertise<std_msgs::Empty>("command8", 1);
        command9_ = n_.advertise<std_msgs::Empty>("command9", 1);
        command10_ = n_.advertise<std_msgs::Empty>("command10", 1);
        command11_ = n_.advertise<std_msgs::Empty>("command11", 1);
        command12_ = n_.advertise<std_msgs::Empty>("command12", 1);
        command13_ = n_.advertise<std_msgs::Empty>("command13", 1);
        command14_ = n_.advertise<std_msgs::Empty>("command14", 1);
        command_ = n_.subscribe("command", 1, &home_command_manager_impl::topicCallback_command, &component_implementation_);

    }
    void topicCallback_command(const std_msgs::Int32::ConstPtr& msg)
    {
        component_data_.in_command = *msg;
    }

    void configure_callback(home_command_manager::home_command_managerConfig &config, uint32_t level)
    {
        configure();
    }

    void configure()
    {
        component_implementation_.configure(component_config_);
    }

    void activate_all_output()
    {
        component_data_.out_command1_active = true;
        component_data_.out_command2_active = true;
        component_data_.out_command3_active = true;
        component_data_.out_command4_active = true;
        component_data_.out_command5_active = true;
        component_data_.out_command6_active = true;
        component_data_.out_command7_active = true;
        component_data_.out_command8_active = true;
        component_data_.out_command9_active = true;
        component_data_.out_command10_active = true;
        component_data_.out_command11_active = true;
        component_data_.out_command12_active = true;
        component_data_.out_command13_active = true;
        component_data_.out_command14_active = true;
    }

    void update()
    {
        activate_all_output();
        component_implementation_.update(component_data_, component_config_);
        if (component_data_.out_command1_active)
            command1_.publish(component_data_.out_command1);
        if (component_data_.out_command2_active)
            command2_.publish(component_data_.out_command2);
        if (component_data_.out_command3_active)
            command3_.publish(component_data_.out_command3);
        if (component_data_.out_command4_active)
            command4_.publish(component_data_.out_command4);
        if (component_data_.out_command5_active)
            command5_.publish(component_data_.out_command5);
        if (component_data_.out_command6_active)
            command6_.publish(component_data_.out_command6);
        if (component_data_.out_command7_active)
            command7_.publish(component_data_.out_command7);
        if (component_data_.out_command8_active)
            command8_.publish(component_data_.out_command8);
        if (component_data_.out_command9_active)
            command9_.publish(component_data_.out_command9);
        if (component_data_.out_command10_active)
            command10_.publish(component_data_.out_command10);
        if (component_data_.out_command11_active)
            command11_.publish(component_data_.out_command11);
        if (component_data_.out_command12_active)
            command12_.publish(component_data_.out_command12);
        if (component_data_.out_command13_active)
            command13_.publish(component_data_.out_command13);
        if (component_data_.out_command14_active)
            command14_.publish(component_data_.out_command14);
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "home_command_manager");

    home_command_manager_ros node;
    node.configure();

    ros::Rate loop_rate(10.0);

    while(node.n_.ok())
    {
        node.update();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
