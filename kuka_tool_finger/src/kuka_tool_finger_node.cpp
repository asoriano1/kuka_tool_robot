#include "ros/ros.h"
#include "math.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include <sensor_msgs/Joy.h>
#include <robotnik_msgs/set_float_value.h>
#include <robotnik_msgs/set_odometry.h>

#define DEFAULT_NUM_OF_BUTTONS		16
#define DEFAULT_AXIS_LINEAR_X		1
#define DEFAULT_AXIS_LINEAR_Y       1
#define DEFAULT_AXIS_ANGULAR		1
#define DEFAULT_AXIS_LINEAR_Z       1	
#define DEFAULT_SCALE_LINEAR		1.0
#define DEFAULT_SCALE_ANGULAR		1.0
#define DEFAULT_SCALE_LINEAR_Z      1.0 

class SubscribeAndPublish
{
private:
	
	
	//! Number of the DEADMAN button for KUKA
	int dead_man_button_;
	//! Number of the DEADMAN button for TOOL
	int button_dead_man_tool_mode_;
	//! Number of the button for increase or decrease the speed max of the joystick	
	int speed_up_button_, speed_down_button_;
	//! Flag to enable/disable the communication with the publishers topics
	bool bEnable;
	//! Flag to track the first reading without the deadman's button pressed.
	bool last_command_;
	int linear_x_, linear_y_, linear_z_, angular_;
	double l_scale_, a_scale_, l_scale_z_; 
	
	double 	current_linear_step, current_angular_step;

	ros::NodeHandle pnh_;	
	ros::Subscriber sub;
	ros::Subscriber sub_joint_states;
	ros::Subscriber sub_kuka_positions;
	ros::Publisher pub1;
	ros::Publisher pub2;
	//! It will be suscribed to the joystick
	ros::Subscriber pad_sub_;
	
	
	
	float position_M1; //radianes del usillo
	float position_M2;
	float position_x_from_home;
	float position_beta_from_home;
	float theta_finger;
	float c1=0.0124896/(4*M_PI);//en metros
	float c2=c1/0.048; 
	//! Number of buttons of the joystick
	int num_of_buttons_;
	float cart_position_kuka[6]; //cart position of the kuka x,y,z,a,b,c
	
	//! Pointer to a vector for controlling the event when pushing the buttons
	bool bRegisteredButtonEvent[DEFAULT_NUM_OF_BUTTONS];
	
	//! Pointer to a vector for controlling the event when pushing directional arrows (UNDER AXES ON PX4!)
    bool bRegisteredDirectionalArrows[4];
    
  
	
public:
	 SubscribeAndPublish():  
	 pnh_("~")
	{
		
		pub1=pnh_.advertise<std_msgs::Float64>("/kuka_tool/joint_up_position_controller/command",1);
		pub2=pnh_.advertise<std_msgs::Float64>("/kuka_tool/joint_down_position_controller/command",1);		
		sub_joint_states=pnh_.subscribe("/kuka_tool/joint_states",1,&SubscribeAndPublish::callback_joints,this);
		pad_sub_ = pnh_.subscribe<sensor_msgs::Joy>("/joy", 10, &SubscribeAndPublish::padCallback, this);
		sub_kuka_positions=pnh_.subscribe<sensor_msgs::JointState>("/cartesian_pos_kuka",1,&SubscribeAndPublish::kukaPosCallback,this);
		
		// MOTION CONF
		pnh_.param("num_of_buttons", num_of_buttons_, DEFAULT_NUM_OF_BUTTONS);
		pnh_.param("axis_linear_x", linear_x_, DEFAULT_AXIS_LINEAR_X);
		pnh_.param("axis_linear_y", linear_y_, DEFAULT_AXIS_LINEAR_Y);
		pnh_.param("axis_linear_z", linear_z_, DEFAULT_AXIS_LINEAR_Z);
		pnh_.param("axis_angular", angular_, DEFAULT_AXIS_ANGULAR);
		pnh_.param("scale_angular", a_scale_, DEFAULT_SCALE_ANGULAR);
		pnh_.param("scale_linear", l_scale_, DEFAULT_SCALE_LINEAR);
		pnh_.param("scale_linear_z", l_scale_z_, DEFAULT_SCALE_LINEAR_Z);
		pnh_.param("button_dead_man", dead_man_button_, dead_man_button_);
		pnh_.param("button_speed_up", speed_up_button_, speed_up_button_);  //4 Thrustmaster
		pnh_.param("button_speed_down", speed_down_button_, speed_down_button_); //5 Thrustmaster
		pnh_.param("button_dead_man_tool_mode", button_dead_man_tool_mode_, button_dead_man_tool_mode_); //euler or cartesian
	
		ROS_INFO("KukaPad num_of_buttons_ = %d", num_of_buttons_);	
		for(int i = 0; i < num_of_buttons_; i++){
			bRegisteredButtonEvent[i] = false;
			ROS_INFO("bREG %d", i);
		}
		for(int i = 0; i < 3; i++){
			bRegisteredDirectionalArrows[i] = false;
		}
		bEnable = false;	// Communication flag disabled by default
		last_command_ = true;
		
		current_linear_step = 0.0005;
		current_angular_step = 0.005;
	}
void kukaPosCallback(const sensor_msgs::JointState::ConstPtr& pos)
{
	for(int i=0; i<6;i++){
		cart_position_kuka[i]=pos->position[i];
	}
	
	
}
void padCallback(const sensor_msgs::Joy::ConstPtr& joy)

{
    std_msgs::Float64 set_pos_M1;
    std_msgs::Float64 set_pos_M2;
    bEnable = false;
    
    if(joy->buttons[dead_man_button_] == 0 && joy->buttons[button_dead_man_tool_mode_] == 1){
		bEnable = true;
	}
    // Actions dependant on dead-man button
 	if (bEnable) {
		//ROS_ERROR("SummitXLPad::padCallback: DEADMAN button %d", dead_man_button_);
		//Set the current velocity level
		if ( joy->buttons[speed_down_button_] == 1 ){

			if(!bRegisteredButtonEvent[speed_down_button_]) 
				if(current_linear_step >0.0001){
		  			current_linear_step = current_linear_step - 0.00005;
					current_angular_step = current_angular_step - 0.00005;
					bRegisteredButtonEvent[speed_down_button_] = true;
					ROS_INFO("Step: %f%%", current_linear_step/0.0009*100.0);
					char buf[50]="\0";
 					int percent = (int) (current_linear_step/0.0009*100.0);
					sprintf(buf," %d percent", percent);
                    // sc.say(buf);
				}
		}else{
			bRegisteredButtonEvent[speed_down_button_] = false;
		 }
		//ROS_ERROR("SummitXLPad::padCallback: Passed SPEED DOWN button %d", speed_down_button_);
		if (joy->buttons[speed_up_button_] == 1){
			if(!bRegisteredButtonEvent[speed_up_button_])
				if(current_linear_step <= 0.0009){
					current_linear_step = current_linear_step + 0.00005;
					current_angular_step = current_angular_step + 0.00005;
					bRegisteredButtonEvent[speed_up_button_] = true;
			 	 	ROS_INFO("Step: %f%%", current_linear_step/0.0009*100.0);
  					char buf[50]="\0";
					int percent = (int) (current_linear_step/0.0009*100.0);
					sprintf(buf," %d percent", percent);
                    // sc.say(buf);
				}
		  
		}else{
			bRegisteredButtonEvent[speed_up_button_] = false;
		}
		float incr_x = current_linear_step * l_scale_*joy->axes[linear_x_];
		float incr_beta = current_angular_step * l_scale_*joy->axes[linear_z_]; //cambio de signo para que hacia arriba sea apretar
    //	ROS_INFO("incr x: %f",incr_x);
		//inverse kin		
		float delta_M2=0.5*((incr_x/c1)-incr_beta/c2);
		//float delta_M11=(incr_x/c1)-delta_M2;
		float delta_M1=(incr_beta/c2)+delta_M2;
	//	ROS_INFO("incrM1: %f",delta_M1);


		set_pos_M2.data=delta_M2+position_M2;		
		set_pos_M1.data=delta_M1+position_M1;

		//if (joy->buttons[dead_man_button_] == 1) {
			pub2.publish(set_pos_M2);
			pub1.publish(set_pos_M1);
		//}
		
		
	}
}
void callback_joints(const sensor_msgs::JointState::ConstPtr& states)
{
	
	position_M1=states->position[1];
	position_M2=states->position[0];
	//ROS_INFO("in callback joints position[0]:%f position[1]:%f",position_M1, position_M2);
	
	//tool positions relative to the homing position
	position_x_from_home=states->position[2]; // in meters
	position_beta_from_home=states->position[3]; //in radians
}
//Service that positions the tool to the desired value of x, relative to the latest homing made. In meters
bool srv_setTranslation(robotnik_msgs::set_float_value::Request &request,robotnik_msgs::set_float_value::Response &response ){
		std_msgs::Float64 set_pos_M1;
		std_msgs::Float64 set_pos_M2;
		if(request.value>=0 && request.value<=0.1){
		float incr_x=request.value-position_x_from_home; //para hacerlo relativo al homing, convertirlo en un incremento de distancia
		float delta_M2=0.5*((incr_x/c1));
		float delta_M1=delta_M2;
		set_pos_M2.data=delta_M2+position_M2;		
		set_pos_M1.data=delta_M1+position_M1;
		pub2.publish(set_pos_M2);
		pub1.publish(set_pos_M1);
		response.ret=true;
		}else{
		response.ret=false;
		}
		
	return true;
}
//Service that positions the tool to the desired value of x and orientation, relative to the latest homing made. In meters and radians
 bool srv_setOdometry(robotnik_msgs::set_odometry::Request &request,robotnik_msgs::set_odometry::Response &response ){
		std_msgs::Float64 set_pos_M1;
		std_msgs::Float64 set_pos_M2;
		if(request.x>=0 && request.x<=0.1 && request.orientation<=0){
		float incr_x=request.x-position_x_from_home; //para hacerlo relativo al homing, convertirlo en un incremento de distancia
		float incr_beta=-request.orientation+position_beta_from_home;
		float delta_M2=0.5*((incr_x/c1)-incr_beta/c2);
		float delta_M1=(incr_beta/c2)+delta_M2;
		set_pos_M2.data=delta_M2+position_M2;		
		set_pos_M1.data=delta_M1+position_M1;
		pub2.publish(set_pos_M2);
		pub1.publish(set_pos_M1);
		response.ret=true;
		}else{
		response.ret=false;
		}
		
	return true;
} 
};

int main(int argc, char **argv)
{	
	ros::init(argc, argv, "kuka_tool_finger_node");
	ros::NodeHandle n;
	SubscribeAndPublish myObject;
	
	ros::ServiceServer set_translation_=n.advertiseService("/kuka_tool_finger_node/set_translation",&SubscribeAndPublish::srv_setTranslation,&myObject);
	ros::ServiceServer set_odometry_=n.advertiseService("/kuka_tool_finger_node/set_odometry",&SubscribeAndPublish::srv_setOdometry,&myObject);
	
	ros::spin();
	return 0;
}
