#include <iostream>
#include <cmath>
#include <time.h>
#include <fstream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

using namespace std;

#define PI 3.1415926535898

double x[2],y[2],z[2];
double uxx[2],uxy[2],uxz[2];
double vx[2],vy[2],vz[2];
double ux[2],uy[2],uz[2];
double x_last,y_last,z_last;

geometry_msgs::Pose pos_drone_fcu;                           //无人机当前位置 (来自fcu)
float Euler_fcu[3];                                          //无人机当前欧拉角(来自fcu)
mavros_msgs::State current_state;

void save_flight_data(std::ofstream& out_file, int timenow);   //数据存储函数
void quaternion_2_euler(float quat[4], float angle[3]);        //四元数转欧拉角[与PX4相同]


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void optitrack_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //位置 -- optitrack系 到 NED系
    x[0] = msg->pose.position.x;
    y[0] = msg->pose.position.z;
    z[0] = msg->pose.position.y;

    //估算速度
    vx[0] = (x[0] - x_last) / 0.01;
    vy[0] = (y[0] - y_last) / 0.01;
    vz[0] = (z[0] - z_last) / 0.01;

    x_last = x[0];
    y_last = y[0];
    z_last = z[0];
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone_fcu = msg->pose;
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




int main(int argc, char **argv)
{
    /**************************************************************************************/
    ros::init(argc, argv, "demo_node");
    ros::NodeHandle nh;

    ros::Subscriber optitrack_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/Trackable1/pose", 100, optitrack_cb);

    // 【订阅】无人机当前位置 坐标系:NED系 [室外：GPS，室内：自主定位或mocap等] 这里订阅仅作比较用
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);

    // 【订阅】无人机当前欧拉角 坐标系 NED系 这里订阅仅作比较用
    ros::Subscriber euler_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, euler_cb);



    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    /**************************************************************************************/

    int n = 2;  //多智能体个数
    double beta =0.6;
    double gamma =1.4;

    //距离矩阵
    double A[2][2]={{0,0},{1,0}};

    double dx[2][2]={{0,0},{-0.4,0}};
    double dy[2][2]={{0,0},{1,0}};
    double dz[2][2]={{0,0},{0.3,0}};


    // // 初始化位置
    for(int i=0;i<n;i++){
        x[i]=0;
        y[i]=0;
        z[i]=0.11;
    }

    x[1]=0;
    y[1]=1.3;


    // // 初始化速度
    for(int i=0;i<n;i++){
        vx[i] =0;
        vy[i] =0;
        vz[i] =0;
    }

    int epock=0;                //循环次数，1500步，每步0.01秒

    /**************************************************************************************/
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
                                << " uxx " << " uyy " << " uzz " \
                                << " vx " << " vy " << " vz " \
                                << " ux " << " uy " << " uz " \
                                << " fcu.x " << " fcu.y " << " fcu.z " \
                                << " Euler_fcu.x " << " Euler_fcu.y " << " Euler_fcu.z " \
                                << " x0 " << " y0 " << " z0 " \
                                <<std::endl;
    }


    /**************************************************************************************/
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = y[1];  //zuobiao zhuanhuan
    pose.pose.position.y = x[1];
    pose.pose.position.z = z[1];

//    //send a few setpoints before starting
    for(int i = 50; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();



    //进入主循环
    while(ros::ok()){

        if (current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(2.0))) {
            if (set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(2.0))) {
                if (arming_client.call(arm_cmd) &&
                        arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
            else
            {
                // 计算控制律
                double sumsum1 = 0;
                double sumsum2 = 0;
                double sumsum3 = 0;
                double sumsum4 = 0;
                double sumsum5 = 0;
                double sumsum6 = 0;

                for(int j = 0;j<n;++j){
                    sumsum1 = sumsum1 + A[1][j] * (x[1]-x[j]-dx[1][j]);
                    sumsum2 = sumsum2 + A[1][j] * (y[1]-y[j]-dy[1][j]);
                    sumsum3 = sumsum3 + A[1][j] * (z[1]-z[j]-dz[1][j]);
                }

                for(int j = 0;j<n;++j){
                    sumsum4 = sumsum4 + A[1][j]*(vx[j]-vx[1]);
                    sumsum5 = sumsum5 + A[1][j]*(vy[j]-vy[1]);
                    sumsum6 = sumsum6 + A[1][j]*(vz[j]-vz[1]);
                }


                double sign1,sign2,sign3,sign4,sign5,sign6;
                if(sumsum1  < 0)
                    sign1 = -1;
                else
                    sign1 = 1;

                if(sumsum2 < 0)
                    sign2 = -1;
                else
                    sign2 = 1;

                if(sumsum3< 0)
                    sign3 = -1;
                else
                    sign3 = 1;

                if(sumsum4  < 0)
                    sign4 = -1;
                else
                    sign4 = 1;

                if(sumsum5  < 0)
                    sign5 = -1;
                else
                    sign5 = 1;

                if(sumsum6  < 0)
                    sign6 = -1;
                else
                    sign6 = 1;

                ux[1]= gamma * sign4 *pow(abs(sumsum4),0.3333);
                vx[1] = vx[1] +  ux[1] * 0.01;

                uy[1] = gamma * sign5 * pow(abs(sumsum5),0.3333);
                vy[1] = vy[1] +  uy[1] * 0.01;

                uz[1] = gamma * sign6 * pow(abs(sumsum6),0.3333);
                vz[1] = vz[1] +  uz[1] * 0.01;


                uxx[1] =  vx[1] - beta * sign1 * pow(abs(sumsum1),0.3333);
                x[1] = x[1] + uxx[1] * 0.01;

                uxy[1] =  vy[1] - beta * sign2 * pow(abs(sumsum2),0.3333);
                y[1] = y[1] + uxy[1] * 0.01;

                uxz[1] =  vz[1] - beta * sign3 * pow(abs(sumsum3),0.3333);
                z[1] = z[1] + uxz[1] * 0.01;

                //设置范围
                if(x[1]>0.85)
                    x[1]=0.8;
                else if(x[1]<-0.85)
                    x[1]=-0.8;

                if(y[1]>1.1)
                    y[1]=1.0;
                else if(y[1]<-1.1)
                    y[1]=-1.0;

                if(z[1]>0.8)
                    z[1]=0.8;


                pose.pose.position.x = y[1];  //此处坐标需要转换
                pose.pose.position.y = x[1];
                pose.pose.position.z = z[1];


            }

        }

        local_pos_pub.publish(pose);

        //储存数据
        if(flag_save == 1)
        {
            save_flight_data(out_data_file,epock+1);
          //   std::cout << "Saving Data!!" << std::endl;
        }
        if (epock >= 1500)
            break;
        epock++;


        ros::spinOnce();
        rate.sleep();
    }

    out_data_file.close();



    //当上一个循环因为epock等于1500跳出了循环,我们就进入降落操作,降落很简单,我们只需要直接切换模式到auto land
    offb_set_mode.request.custom_mode = "AUTO.LAND";//重新设置模式切换服务的服务参数
    while (ros::ok())
    {
        if (current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            //如果当前模式还不是auto land,并且距离上一次模式切换大于了5s,进入这里
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                //如果模式切换执行成功打印信息
                ROS_INFO("AUTO.LAND enabled");
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


//数据保存函数
void save_flight_data(std::ofstream& out_file, int epock)
{
    out_file << epock << "  " << x[1] <<"  "<< y[1] <<"  "<< z[1] <<"  "\
                              << uxx[1] <<"  "<< uxy[1] <<"  "<< uxz[1] <<"  "\
                              << vx[1] <<"  "<< vy[1] <<"  "<< vz[1] <<"  "\
                              << ux[1] <<"  "<< uy[1] <<"  "<< uz[1] <<"  "\
                              << pos_drone_fcu.position.x <<"  "<< pos_drone_fcu.position.y <<"  "<< pos_drone_fcu.position.z <<"  "\
                              << Euler_fcu[0] /PI*180<<"  "<< Euler_fcu[1] /PI*180 <<"  "<< Euler_fcu[2] /PI*180<<" " \
                              << x[0] <<"  "<< y[0] <<"  "<< z[0] <<"  "\
                              << std::endl;
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




