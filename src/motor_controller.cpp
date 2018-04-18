// ROS Default Header File
#include "ros/ros.h"
// MsgTutorial Message File Header
// The header file is automatically created when building the package.

#include "dynamixel_position_control/MsgDynamixel.h"
#include "dynamixel_msgs/JointState.h"
#include "picture_taker/image_cmd.h"
#include <sstream>

#define GOAL_POS 0
#define CURRENT_POS 1
#define ERROR 2
#define LOAD 3
#define ERROR_POS 0.01
#define ERROR_NEG -0.01
#define TX 0
#define RX 1

void motor_command(ros::Publisher dynamixel_publisher, picture_taker::image_cmd service, ros::ServiceClient service_client); //Receive the current motor position and send the next position
bool motor_init(float qtd_pos); //Initialize motor variables

struct Motor{
 float motor_state[4], count, pos;
 int Estado;
 bool moving;
 dynamixel_position_control::MsgDynamixel msg;
}MX28;

//Initialize motor variables
bool motor_init(float qtd_pos)
{
  if(qtd_pos > 360 || qtd_pos <= 0){
   ROS_ERROR("Qtd_pos value should be between 0 and 360");
   return false;
  }
  MX28.Estado = TX; 
  MX28.count = 0;
  MX28.pos = 6.14/qtd_pos;
  for(int i = 0; i < 4; i++) MX28.motor_state[i] = 0xff;
  MX28.moving = false;
  MX28.msg.data = 0;   
  return true;
}

// Message callback function. This is a function is called when a topic
// message named 'tilt_controller/state' is received.
void msgCallback(const dynamixel_msgs::JointState::ConstPtr& msg)
{   
 MX28.motor_state[GOAL_POS] = msg->goal_pos;
 MX28.motor_state[CURRENT_POS] = msg->current_pos;
 MX28.motor_state[ERROR] = msg->error;
 MX28.motor_state[LOAD] = msg->load;
 MX28.moving = msg->is_moving;   
}  


int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "dynamixel_publisher");	// Initializes Node Name
  ros::init(argc, argv, "dynamixel_subscriber"); 
  ros::NodeHandle nh; // Node handle declaration for communication with ROS system

  float qtd_pos;
  nh.param("Qtd_Pos", qtd_pos, qtd_pos); //Receive qtd_pos from launch file (value should be between 0 and 360) 
   
  if(!motor_init(qtd_pos)) return 0;

  // Declare publisher, create publisher 'dynamixel_publisher' using the 'MsgDynamixel'
  // message file from the 'dynamixel_control_position' package. The topic name is
  // 'tilt_controller/command' and the size of the publisher queue is set to 100
   ros::Publisher dynamixel_publisher = nh.advertise<dynamixel_position_control::MsgDynamixel>("tilt_controller/command", 100); 

  // Declares subscriber. Create subscriber 'dynamixel_subscriber' using the 'MsgDynamixel'
  // message file from the 'dynamixel_control_position' package. The topic name is
  // 'tilt_controller/state' and the size of the subscribe queue is set to 100.
  ros::Subscriber dynamixel_subscriber = nh.subscribe("tilt_controller/state", 100, msgCallback);
  ros::ServiceClient service_client = nh.serviceClient<picture_taker::image_cmd>("image_cmd");

  picture_taker::image_cmd service;
  service.request.cmd = true;
  service.request.path = "/home/everton/robot/";
  
  ros::Rate loop_rate(5); // Set the loop period (Hz)
  
  
   while (ros::ok()){	
    // Goes to sleep according to the loop rate defined above.
    loop_rate.sleep();
 	  
    // A function for calling a callback function, waiting for a message to be  
    // received, and executing a callback function when it is received
    ros::spinOnce();
    motor_command(dynamixel_publisher, service, service_client);
   }
   
   return 0;
}

//Receive the current motor position and send the next position
void motor_command(ros::Publisher dynamixel_publisher, picture_taker::image_cmd service, ros::ServiceClient service_client)
{
  switch(MX28.Estado)
  {
   case TX:
    MX28.msg.data = MX28.count; 
    dynamixel_publisher.publish(MX28.msg); // Publishes 'MX28.msg' message
    ROS_INFO("Send Position = %f", MX28.msg.data);		 
    MX28.Estado = RX;
   break;		
		
   case RX:
    if((MX28.count - MX28.motor_state[CURRENT_POS] <= ERROR_POS) && (MX28.count - MX28.motor_state[CURRENT_POS] >= ERROR_NEG))
    {
     ROS_INFO("Current Position = %f", MX28.motor_state[CURRENT_POS]); 
     std::ostringstream count_string;
 	 count_string << MX28.count;
     service.request.num_name = count_string.str();
     service_client.call(service);
     if (service.response.result == 1)
     {
       MX28.count += MX28.pos;
       if(MX28.count >= 6.14) MX28.count = 0;
       MX28.Estado = TX;
     }else MX28.Estado = RX;       	  
    }else MX28.Estado = TX; 	 
   break;		
  }		
} 
