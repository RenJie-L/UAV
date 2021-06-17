/*************************************************************************************************************************
 * position_estimator.cpp
 *
 * Author: Qyp
 *
 * Update Time: 2018.8.17
 *
 * 说明: mavros位置估计程序
 *      1. 订阅激光SLAM (cartorgrapher_ros节点) 发布的位置信息
 *      2. 订阅Mocap设备 (vrpn-client-ros节点) 发布的位置信息
 *      3. 存储飞行数据，实验分析及作图使用
 *      4. 发布位置及偏航角(xyz+yaw)给飞控,利用飞控内部的位置及姿态模块得到更精准的无人机位置和速度 [a]
 *
 *                 [a]: 其实这一块也可以在本节点中完成，只需要订阅飞控的IMU信息，编写滤波算法程序即可，目前正在开发中； 
***************************************************************************************************************************/
//头文件
#include <ros/ros.h>
#include <fstream>
#include <math.h>
#include <string>
#include <time.h>
#include <queue>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <iomanip>
//msg 头文件
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
//
using namespace std;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define PI 3.1415926535898
//---------------------------------------相关参数-----------------------------------------------
float dt;                                                  //程序运行的频率
int flag_save;                                             //0:存储数据 1:不存储数据
int flag_use_laser_or_mocap;                               //0:使用mocap数据作为定位数据 1:使用laser数据作为定位数据
//---------------------------------------vicon定位相关------------------------------------------
geometry_msgs::Pose pos_drone_mocap;                          //无人机当前位置 (vicon)
geometry_msgs::Pose pos_drone_mocap_last;                     //无人机上一时刻位置 (vicon)
geometry_msgs::Pose vel_drone_mocap;                          //无人机当前速度 (vicon)
float Euler_mocap[3];                                         //无人机当前姿态 (vicon)

//---------------------------------------laser定位相关-------------------------------------------
geometry_msgs::Pose pos_drone_laser;                          //无人机当前位置 (laser)
geometry_msgs::Pose pos_drone_laser_last;                     //无人机上一时刻位置 (laser)
geometry_msgs::Pose vel_drone_laser;                          //无人机当前速度 (laser)
float Euler_laser[3];                                         //无人机当前姿态(laser)

geometry_msgs::TransformStamped laser;                          //当前时刻cartorgrapher发布的数据
geometry_msgs::TransformStamped laser_last;                     //上一时刻cartorgrapher发布的数据
//---------------------------------------无人机位置及速度--------------------------------------------
geometry_msgs::Pose pos_drone_fcu;                           //无人机当前位置 (来自fcu)
geometry_msgs::Pose vel_drone_fcu;                           //无人机上一时刻位置 (来自fcu)
float Euler_fcu[3];                                          //无人机当前欧拉角(来自fcu)
//---------------------------------------发布相关变量--------------------------------------------
geometry_msgs::PoseStamped mocap;                              //发送给飞控的mocap(来源：mocap或laser)
geometry_msgs::PoseStamped vision;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void quaternion_2_euler(float quat[4], float angle[3]);                              //四元数转欧拉角[与PX4相同]
void euler_2_quaternion(float angle[3], float quat[4]);                              //欧拉角转四元数
float get_dt(ros::Time last);                                                        //获取时间间隔
void printf();                                                                       //打印函数
void save_flight_data(std::ofstream& out_file, float timenow);                       //储存数据函数
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void laser_cb(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    //确定是cartographer发出来的/tf信息
    //有的时候/tf这个消息的发布者不止一个
    if (msg->transforms[0].header.frame_id == "map")
    {
        laser = msg->transforms[0];

        float dt_laser;

        dt_laser = (laser.header.stamp.sec - laser_last.header.stamp.sec) + (laser.header.stamp.nsec - laser_last.header.stamp.nsec)/10e9;


        if (dt_laser != 0)
        {
            //SLAM算法发布位置的频率大概为10Hz ， 也可以用上面的dt_laser，但不如这样效果好
            dt_laser = 0.1;

            //位置 xy
            pos_drone_laser.position.x = laser.transform.translation.x;
            pos_drone_laser.position.y = - laser.transform.translation.y;

            //速度 vx vy [平滑处理][目前没有处理]
            float vel_now[2];

            vel_now[0] = (pos_drone_laser.position.x - pos_drone_laser_last.position.x) / dt_laser;
            vel_now[1] = (pos_drone_laser.position.y - pos_drone_laser_last.position.y) / dt_laser;

            vel_drone_laser.position.x = 0*vel_drone_laser.position.x + 1*vel_now[0];
            vel_drone_laser.position.y = 0*vel_drone_laser.position.y + 1*vel_now[1];

            //储存为上一时刻数据
            pos_drone_laser_last.position.x = pos_drone_laser.position.x;
            pos_drone_laser_last.position.y = pos_drone_laser.position.y;

            laser_last = laser;

            //欧拉角
            float q[4];
            q[0] = laser.transform.rotation.w;
            q[1] = laser.transform.rotation.x;
            q[2] = laser.transform.rotation.y;
            q[3] = laser.transform.rotation.z;
            quaternion_2_euler(q, Euler_laser);

            //laser系 到 NED系
            Euler_laser[0] = Euler_laser[0];
            Euler_laser[1] = - Euler_laser[1];
            Euler_laser[2] = - Euler_laser[2];
        }
    }
}
void sonic_cb(const std_msgs::UInt16::ConstPtr& msg)
{
    std_msgs::UInt16 sonic;
    sonic = *msg;

    //位置
    pos_drone_laser.position.z  = - (float)sonic.data / 1000;

    //速度 [平滑处理]
    float vel_now;

    vel_now = (pos_drone_laser.position.z  - pos_drone_laser_last.position.z ) / 0.1;

    vel_drone_laser.position.z = 0*vel_drone_laser.position.z + 1*vel_now ;

    pos_drone_laser_last.position.z = pos_drone_laser.position.z;

}

void optitrack_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //位置 -- optitrack系 到 NED系
    pos_drone_mocap.position.x = msg->pose.position.z;
    pos_drone_mocap.position.y = msg->pose.position.x;
    pos_drone_mocap.position.z = msg->pose.position.y;

    //估算速度
    vel_drone_mocap.position.x = (pos_drone_mocap.position.x - pos_drone_mocap_last.position.x) / dt;
    vel_drone_mocap.position.y = (pos_drone_mocap.position.y - pos_drone_mocap_last.position.y) / dt;
    vel_drone_mocap.position.z = (pos_drone_mocap.position.z - pos_drone_mocap_last.position.z) / dt;


    pos_drone_mocap_last = pos_drone_mocap;

    //欧拉角
    float q[4];
    q[0] = msg->pose.orientation.w;
    q[1] = msg->pose.orientation.x;
    q[2] = msg->pose.orientation.z;
    q[3] = msg->pose.orientation.y;
    quaternion_2_euler(q, Euler_mocap);

    //optitrack系 到 NED系
    Euler_mocap[0] = Euler_mocap[1];
    Euler_mocap[1] =-Euler_mocap[0];

    Euler_mocap[2] =Euler_mocap[2];

}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone_fcu = msg->pose;
}

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    vel_drone_fcu.position.x = msg->twist.linear.x;
    vel_drone_fcu.position.y = msg->twist.linear.y;
    vel_drone_fcu.position.z = msg->twist.linear.z;
}

void euler_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    float q[4];
    q[0] = msg->orientation.w;
    q[1] = msg->orientation.x;
    q[2] = msg->orientation.y;
    q[3] = msg->orientation.z;
    quaternion_2_euler(q, Euler_fcu);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_estimator");
    ros::NodeHandle nh("~");

    //读取参数表中的参数
    // 是否存储数据 1 for save , 0 for not save
    nh.param<int>("flag_save", flag_save, 0);
    // 使用激光SLAM数据orVicon数据 0 for vicon， 1 for 激光SLAM
    nh.param<int>("flag_use_laser_or_mocap", flag_use_laser_or_mocap, 0);

    //【订阅】cartographer估计位置
    ros::Subscriber laser_sub = nh.subscribe<tf2_msgs::TFMessage>("/tf", 1000, laser_cb);

    // 【订阅】超声波的数据
    ros::Subscriber sonic_sub = nh.subscribe<std_msgs::UInt16>("/sonic", 100, sonic_cb);

    // 【订阅】optitrack估计位置
    ros::Subscriber optitrack_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/Trackable3/pose", 1000, optitrack_cb);

    // 【订阅】无人机当前位置 坐标系:NED系 [室外：GPS，室内：自主定位或mocap等] 这里订阅仅作比较用
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);

    // 【订阅】无人机当前速度 坐标系:NED系 [室外：GPS，室内：自主定位或mocap等] 这里订阅仅作比较用
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity", 10, vel_cb);
    // 【订阅】无人机当前欧拉角 坐标系 NED系 这里订阅仅作比较用
    ros::Subscriber euler_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, euler_cb);

    // 【发布】无人机位置和偏航角 坐标系 NED系
    //  本话题要发送飞控(通过mavros_extras/src/plugins/mocap_pose_estimate.cpp发送), 对应Mavlink消息为ATT_POS_MOCAP(#138), 对应的飞控中的uORB消息为att_pos_mocap.msg
    ros::Publisher mocap_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose", 100);

    // 频率
    ros::Rate rate(20.0);

    dt = 0.05;

    //储存数据

    time_t tt = time(NULL);
    tm* t = localtime(&tt);
    char iden_path[256];
    sprintf(iden_path, "/home/pi/amov_ros/data/position-estimator-%d-%02d-%02d_%02d-%02d.txt", t->tm_year+1900, t->tm_mon+1, t->tm_mday, t->tm_hour, t->tm_min);
    std::ofstream out_data_file(iden_path);

    if (!out_data_file)
    {
        std::cout << "Error: Could not write data!" << std::endl;
        return 0;
    }
    else
    {
        std::cout << "save data!!"<<std::endl;
        out_data_file <<" Time "<< " pos_drone_mocap.x " << " pos_drone_mocap.y " << " pos_drone_mocap.z " \
                                << " vel_drone_mocap.x " << " vel_drone_mocap.y " << " vel_drone_mocap.z " \
                                << " Euler_mocap.x " << " Euler_mocap.y " << " Euler_mocap.z " \
                                << " pos_drone_fcu.x " << " pos_drone_fcu.y " << " pos_drone_fcu.z " \
                                << " vel_drone_fcu.x " << " vel_drone_fcu.y " << " vel_drone_fcu.z " \
                                << " Euler_fcu.x " << " Euler_fcu.y " << " Euler_fcu.z " \
                                <<std::endl;
    }



    // 记录启控时间
    ros::Time begin_time = ros::Time::now();

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        // 当前时间
        float cur_time = get_dt(begin_time);

        //回调一次 更新传感器状态
        ros::spinOnce();

        //mocap
        if(flag_use_laser_or_mocap == 0)
        {
            mocap.pose.position.x = pos_drone_mocap.position.x;
            mocap.pose.position.y = pos_drone_mocap.position.y;
            mocap.pose.position.z = pos_drone_mocap.position.z;

            float q[4];
            euler_2_quaternion(Euler_mocap, q);

            mocap.pose.orientation.x = q[1];
            mocap.pose.orientation.y = q[2];
            mocap.pose.orientation.z = q[3];
            mocap.pose.orientation.w = q[0];

        }//laser
        else if (flag_use_laser_or_mocap == 1)
        {
            mocap.pose.position.x = pos_drone_laser.position.x;
            mocap.pose.position.y = pos_drone_laser.position.y;
            mocap.pose.position.z = pos_drone_laser.position.z;

            float q[4];

            euler_2_quaternion(Euler_laser, q);

            mocap.pose.orientation.x = q[1];
            mocap.pose.orientation.y = q[2];
            mocap.pose.orientation.z = q[3];
            mocap.pose.orientation.w = q[0];

        }

        mocap_pub.publish(mocap);

        //储存数据
        if(flag_save == 1)
        {
            save_flight_data(out_data_file,cur_time);
            std::cout << "Saving Data!!" << std::endl;
        }

        //打印
        printf();
        rate.sleep();
    }

    return 0;

}
void save_flight_data(std::ofstream& out_file, float timenow)
{
    out_file << timenow <<"  "<< pos_drone_mocap.position.x <<"  "<< pos_drone_mocap.position.y <<"  "<< pos_drone_mocap.position.z <<"  "\
                              << vel_drone_mocap.position.x <<"  "<< vel_drone_mocap.position.y <<"  "<< vel_drone_mocap.position.z <<"  "\
                              << Euler_mocap[0] /PI*180 << Euler_mocap[1] /PI*180 << Euler_mocap[2] /PI*180<<" " \
                              << pos_drone_fcu.position.x <<"  "<< pos_drone_fcu.position.y <<"  "<< pos_drone_fcu.position.z <<"  "\
                              << vel_drone_fcu.position.x <<"  "<< vel_drone_fcu.position.y <<"  "<< vel_drone_fcu.position.z <<"  "\
                              << Euler_fcu[0] /PI*180<< Euler_fcu[1] /PI*180 << Euler_fcu[2] /PI*180<<" " \
                              << std::endl;
}

//获取当前时间 单位：秒
float get_dt(ros::Time last)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-last.sec;
    float currTimenSec = time_now.nsec / 1e9 - last.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

// 四元数转Euler
// q0 q1 q2 q3
// w x y z
void quaternion_2_euler(float quat[4], float angle[3])
{
    angle[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    angle[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), -1.0 + 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]));
    //angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
}


// Euler转四元数
// q0 q1 q2 q3
// w x y z
void euler_2_quaternion(float angle[3], float quat[4])
{
    double cosPhi_2 = cos(double(angle[0]) / 2.0);

    double sinPhi_2 = sin(double(angle[0]) / 2.0);

    double cosTheta_2 = cos(double(angle[1] ) / 2.0);

    double sinTheta_2 = sin(double(angle[1] ) / 2.0);

    double cosPsi_2 = cos(double(angle[2]) / 2.0);

    double sinPsi_2 = sin(double(angle[2]) / 2.0);


    quat[0] = float(cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2);

    quat[1] = float(sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2);

    quat[2] = float(cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2);

    quat[3] = float(cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2);

}

void printf()
{

    cout <<">>>>>>>>>>>>>>>>>>>>>>Position Estimator<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout.setf(ios::fixed);

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>Laser Info<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Pos_laser: [X Y Z] : " << " " << fixed <<setprecision(5)<< pos_drone_laser.position.x << " [m] "<< pos_drone_laser.position.y<<" [m] "<< pos_drone_laser.position.z<<" [m] "<<endl;
    cout << "Vel_laser: [X Y Z] : " << " " << fixed <<setprecision(5)<< vel_drone_laser.position.x << " [m/s] "<< vel_drone_laser.position.y<<" [m/s] "<< vel_drone_laser.position.z<<" [m/s] "<<endl;
    cout << "Yaw_laser:  : " << Euler_laser[2] * 180/PI<<"   "<<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>Vicon Info<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Pos_vicon: [X Y Z] : " << " " << fixed <<setprecision(5)<< pos_drone_mocap.position.x << " [m] "<< pos_drone_mocap.position.y<<" [m] "<< pos_drone_mocap.position.z<<" [m] "<<endl;
    cout << "Vel_vicon: [X Y Z] : " << " " << fixed <<setprecision(5)<< vel_drone_mocap.position.x << " [m/s] "<< vel_drone_mocap.position.y<<" [m/s] "<< vel_drone_mocap.position.z<<" [m/s] "<<endl;
    cout << "Yaw_vicon:  : " << fixed <<setprecision(5)<< Euler_mocap[2] * 180/PI<<"   "<<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>FCU Info<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Pos_fcu: [X Y Z] : " << " " << fixed <<setprecision(5)<< pos_drone_fcu.position.x << " [m] "<< pos_drone_fcu.position.y<<" [m] "<< pos_drone_fcu.position.z<<" [m] "<<endl;
    cout << "Vel_fcu: [X Y Z] : " << " " << fixed <<setprecision(5)<< vel_drone_fcu.position.x << " [m/s] "<< vel_drone_fcu.position.y<<" [m/s] "<< vel_drone_fcu.position.z<<" [m/s] "<<endl;
    cout << "Euler_fcu:  : " << fixed <<setprecision(5)<< Euler_fcu[0] * 180/PI<<" [du] "<< Euler_fcu[1] * 180/PI<<" [du] "<< Euler_fcu[2] * 180/PI<<" [du] "<<endl;
}
