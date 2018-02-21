#include "ros/ros.h"
#include "std_msgs/Float64.h"


class SubscribeAndPublish
{
	public:
	 SubscribeAndPublish()
	{
		
		pub1=n.advertise<std_msgs::Float64>("/kuka_tool/joint_up_position_controller/command",1);
		pub2=n.advertise<std_msgs::Float64>("/kuka_tool/joint_down_position_controller/command",1);
		sub=n.subscribe("xpos", 100, &SubscribeAndPublish::chatterCallback,this);
	}
void chatterCallback(const std_msgs::Float64::ConstPtr& msg)
{
    std_msgs::Float64 value;
   float aux=msg->data;
   ROS_INFO("in callback %f",aux);
   value.data=7*aux;
	if(aux >=-1 && aux<=1){
		 ROS_INFO("in if");
		pub1.publish(value);
		pub2.publish(value);
	}
}
	private:
	
	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::Publisher pub1;
	ros::Publisher pub2;
	
  
};

int main(int argc, char **argv)
{	
	ros::init(argc, argv, "x_to_motor");
	SubscribeAndPublish myObject;
  
	ros::spin();
	return 0;
}
