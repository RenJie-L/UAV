/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>
#include <time.h>
#include <fstream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
using namespace std;
double x,y,z;                                       
void save_flight_data(std::ofstream& out_file, int timenow);   //数据存储函数
    //四元数转欧拉角[与PX4相同]
void optitrack_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{  
    //位置 -- optitrack系 到 NED系
    x = msg->pose.position.z;
    y = msg->pose.position.x;
    z = msg->pose.position.y;

}
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


int main(int argc, char **argv)
{
        
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber optitrack_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/Trackable4/pose", 1000, optitrack_cb);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x =-1.2;
    pose.pose.position.y =-1.2; 
    pose.pose.position.z = 0.25;
    pose.pose.orientation.x=0;
    pose.pose.orientation.y=0;
    pose.pose.orientation.z=0;
    pose.pose.orientation.w=1;
//
     
    for(int i = 20; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    int epock=0;
    
    time_t tt = time(NULL);
    tm* t = localtime(&tt);
    int flag_save=1;
    char iden_path[256];
    sprintf(iden_path, "/home/pi/amov_ros/data/xyz_position-%d-%02d-%02d_%02d-%02d.txt", t->tm_year+1900, t->tm_mon+1, t->tm_mday, t->tm_hour, t->tm_min);
    ofstream out_data_file(iden_path);

    if (!out_data_file.is_open())
    {
        cout << "Error: Could not write data!" << std::endl;
        return 0;
    }
    else
    {
        cout << "save data!!"<<std::endl;
        out_data_file <<" Time "<< " x " << " y " << " z " \
                                <<std::endl;
    }
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(2.0))){
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(2.0))){
                if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
	
        }
      if( epock < 400) {
            pose.pose.position.x = -1;
            pose.pose.position.y = -0.5;
            pose.pose.position.z = 0.25;
        } else if(epock >= 400 && epock < 600) {
            pose.pose.position.x = -0.8;
            pose.pose.position.y = -0.2;
            pose.pose.position.z = 0.25;
        } else if(epock >= 600 && epock < 800) {
            pose.pose.position.x = -0.7;
            pose.pose.position.y = 0.25;
            pose.pose.position.z = 0.25;
        } else if(epock >= 800 && epock < 1000) {
            pose.pose.position.x = -0.3; 
            pose.pose.position.y = 0.4;
            pose.pose.position.z = 0.25;
        } else if(epock >= 1000 && epock < 1200){
            pose.pose.position.x = 0;
            pose.pose.position.y = 0.5;
            pose.pose.position.z = 0.25;
	}else if(epock >= 1200 && epock < 1400){
            pose.pose.position.x = 0.4;
            pose.pose.position.y = 0.7;
            pose.pose.position.z = 0.25;
	}else if(epock >= 1400 && epock < 1600){
            pose.pose.position.x = 0.6;
            pose.pose.position.y = 0.8;
            pose.pose.position.z = 0.25;
	}else if(epock >= 1600 && epock < 1800){
            pose.pose.position.x = 0.8;
            pose.pose.position.y = 0.4;
            pose.pose.position.z = 0.25;
	}else if(epock >= 1800 && epock < 2000){
            pose.pose.position.x = 0.9;
            pose.pose.position.y = 0;
            pose.pose.position.z = 0.25;
        }else{
            pose.pose.position.x = 1;
            pose.pose.position.y = -0.2;
            pose.pose.position.z = 0.25;
        }
        /*if( epock < 200) {
            pose.pose.position.x = -1.5;
            pose.pose.position.y = 0.7;
            pose.pose.position.z = 0.3;
        } else if(epock >= 200 && epock < 300) {
            pose.pose.position.x = -1.2;
            pose.pose.position.y = 0.7;
            pose.pose.position.z = 0.3;
        } else if(epock >= 300 && epock < 400) {
            pose.pose.position.x = -0.9;
            pose.pose.position.y = 0.6;
            pose.pose.position.z = 0.3;
        } else if(epock >= 400 && epock < 500) {
            pose.pose.position.x = -0.6; 
            pose.pose.position.y = 0.6;
            pose.pose.position.z = 0.3;
        } else if(epock >= 500 && epock < 600) {
            pose.pose.position.x = -0.3;
            pose.pose.position.y = 0.5;
            pose.pose.position.z = 0.3;
        } else if(epock >= 600 && epock < 700) {
            pose.pose.position.x = 0;
            pose.pose.position.y = 0.5;
            pose.pose.position.z = 0.3;
        }  else if(epock >= 700 && epock < 800) {
            pose.pose.position.x = 0.4;
            pose.pose.position.y = 0.5;
            pose.pose.position.z = 0.3;
        } else if(epock >= 800 && epock < 900) {
            pose.pose.position.x = 0.8;
            pose.pose.position.y = 0.4;
            pose.pose.position.z = 0.3;
        } else if(epock >= 900 && epock < 1000) {
            pose.pose.position.x = 0.9;
            pose.pose.position.y = 0.4;
            pose.pose.position.z = 0.3;
        } else if(epock >= 1000 && epock < 1100) {
            pose.pose.position.x = 0.9;
            pose.pose.position.y = 0.4;
            pose.pose.position.z = 0.3;
        } else if(epock >= 1100 && epock < 1200) {
            pose.pose.position.x = 0.4;
            pose.pose.position.y = 0.2;
            pose.pose.position.z = 0.3;
        } else if(epock >= 1200 && epock < 1300) {
            pose.pose.position.x = 0.4;
            pose.pose.position.y = -0.3;
            pose.pose.position.z = 0.3;
        } else if(epock >= 1300 && epock < 1400) {
            pose.pose.position.x = 1.0;
            pose.pose.position.y = -0.7;
            pose.pose.position.z = 0.3;
        }*/
	/*else {
            pose.pose.position.x = 0.9;
            pose.pose.position.y = 0.4;
            pose.pose.position.z = 0.3;
        }*/
        local_pos_pub.publish(pose);
        if(flag_save == 1)
        {
            save_flight_data(out_data_file,epock+1);
             //std::cout << "Saving Data!!" << std::endl;
        }
        if (epock >= 2200)
            break;
        epock++;
        ros::spinOnce();
        rate.sleep();
    }
  out_data_file.close();
    return 0;
}
void save_flight_data(std::ofstream& out_file, int epock)
{
    out_file << epock << "  " << x <<"  "<< y <<"  "<< z <<"  "\
                              << std::endl;
}
