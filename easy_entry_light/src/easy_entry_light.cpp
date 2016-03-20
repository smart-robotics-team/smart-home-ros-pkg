#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

class EasyLight {
	public:
		EasyLight();
		~EasyLight();

		ros::Subscriber on_sub_;
		ros::Publisher on_pub;
		ros::Publisher off_pub;
		
		void execute(void);

	private:
		void onCallback(const std_msgs::Bool::ConstPtr &input);

		ros::NodeHandle nh;
		ros::Time lastOff;
		std_msgs::Bool enableOff;
};

EasyLight::EasyLight()
{
	ros::NodeHandle nhp("~");

	on_sub_ = nh.subscribe < std_msgs::Bool > ("/pushed", 2, &EasyLight::onCallback, this);
	on_pub = nh.advertise < std_msgs::Empty > ("/MILIGHT/light3ON", 2);
	off_pub = nh.advertise < std_msgs::Empty > ("/MILIGHT/light3OFF", 2);

	lastOff = ros::Time::now();
	enableOff.data = false;
}

EasyLight::~EasyLight()
{
}


void EasyLight::onCallback(const std_msgs::Bool::ConstPtr &input)
{
	if(input->data == true)
	{
		std_msgs::Empty empty;
		on_pub.publish(empty);
		enableOff.data = false;
	}
	else
	{
		lastOff = ros::Time::now();
		enableOff.data = true;
	}
}

void EasyLight::execute(void)
{
	if(enableOff.data == true)
        {
                if( (ros::Time::now().toSec() - lastOff.toSec()) > 10.0)
		{
			std_msgs::Empty empty;
                	off_pub.publish(empty);
                	enableOff.data = false;
		}
        }
}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line. For programmatic
	 * remappings you can use a different version of init() which takes remappings
	 * directly, but for most command-line programs, passing argc and argv is the easiest
	 * way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "EasyLight");
	EasyLight easylight;

	// Refresh rate
        ros::Rate loop_rate(50);

        while (ros::ok()) {
                ros::spinOnce();
                loop_rate.sleep();
                easylight.execute();
        }

	ros::Duration(2.0).sleep();

	ros::shutdown();
}



