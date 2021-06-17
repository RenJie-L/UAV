#include <ros/ros.h>
#include <std_msgs/String.h>
#include <serial/serial.h>
#include <std_msgs/Empty.h>


serial::Serial ser;

int main(int argc,char ** argv)
{
  ros::init(argc,argv,"talker");
  ros::NodeHandle nh;
  ros::Publisher read_pub=nh.advertise<std_msgs::String>("read",1000);
  
try{
ser.setPort("/dev/ttyUSB0");
ser.setBaudrate(115200);
serial::Timeout to= serial::Timeout::simpleTimeout(1000);
ser.setTimeout(to);
ser.open();
}
catch(serial::IOException& e)
{
 ROS_ERROR_STREAM("Unable to open port");
 return -1;
}

if(ser.isOpen())
{
	ROS_INFO_STREAM("Serial Port initialized");
}
else
{
	return -1;
}
ros::Rate loop_rate(20);
while(ros::ok())
{
	if(ser.available())
	{
	//ROS_INFO_STREAM("Reading from serial port\n");
	std_msgs::String result;
	result.data=ser.read(ser.available());
	ROS_INFO_STREAM("Read:-"<<result.data);
	read_pub.publish(result);
	}
	ros::spinOnce();
	loop_rate.sleep();
}

}
