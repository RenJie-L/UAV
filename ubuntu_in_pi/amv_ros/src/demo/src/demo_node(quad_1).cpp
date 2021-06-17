#include <iostream>
#include <cmath>
#include <fstream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

using namespace std;

double x[2],y[2],z[2];
double uxx[2],uxy[2],uxz[2];
double vx[2],vy[2],vz[2];
double ux[2],uy[2],uz[2];

void save_flight_data(std::ofstream& out_file, int timenow);   //数据存储函数

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


int main(int argc, char **argv)
{
    /**************************************************************************************/
    ros::init(argc, argv, "demo_node");
    ros::NodeHandle nh;

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

    double dx[2][2]={{0,0},{0,0}};
    double dy[2][2]={{0,0},{0.346,0}};
    double dz[2][2]={{0,0},{0,0}};


    // // 初始化位置
    for(int i=0;i<n;i++){
        x[i]=0;
        y[i]=0;
        z[i]=0.11;
    }

    x[0]=0;
    y[0]=0.6;

    x[1]=0.8;
    y[1]=1;


    // // 初始化速度
    for(int i=0;i<n;i++){
        vx[i] =0;
        vy[i] =0;
        vz[i] =0;
    }

    //leader
    double al[3]={0,0,0};
    double vl[3]={0,0,0};

    int epock=0;                //循环次数，1500步，每步0.01秒

    /**************************************************************************************/
    int flag_save=1;
    char iden_path[256];
    sprintf(iden_path, "/home/pi/amov_ros/data/xyz_position.txt");
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
                                <<std::endl;
    }

    /**************************************************************************************/

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = y[1];
    pose.pose.position.y = x[1];
    pose.pose.position.z = z[1];

//    //send a few setpoints before starting
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
                //leader information generator
                if(epock<299){  //0-3s          0-15s
                    al[0]=0.06-vl[0];
                    al[1]=-0.2-vl[1];
                    al[2]=0.08-vl[2];
                }
                else if(epock<599){ //3-6s       15-30s
                    al[0]=0-vl[0];
                    al[1]=-0.2-vl[1];
                    al[2]=0-vl[2];
                }
                else if(epock<749){  //6-7.5s    30-37.5s
                    al[0]=-0.12-vl[0];
                    al[1]=0-vl[1];
                    al[2]=0.08-vl[2];
                }
                else if(epock<899){  //7.5-9s    37.5-45s
                    al[0]=-0.12-vl[0];
                    al[1]=0-vl[1];
                    al[2]=0.08-vl[2];
                }
                else if(epock<1049){  //9s-10.5s    45-52.5s
                    al[0]=0-vl[0];
                    al[1]=0.13-vl[1];
                    al[2]=0-vl[2];
                }
                else if(epock<1199){  //10s-12s   50-60s
                    al[0]=0-vl[0];
                    al[1]=0.13-vl[1];
                    al[2]=0-vl[2];
                }
                else{                              //12-15s        60-75s
                    al[0]=0-vl[0];
                    al[1]=0.13-vl[1];
                    al[2]=0-vl[2];
                }


                vx[0] = vx[0] +  al[0] * 0.01;
                vy[0] = vy[0] +  al[1] * 0.01;
                vz[0] = vz[0] +  al[2] * 0.01;
                vl[0]=vx[0];vl[1]=vy[0];vl[2]=vz[0];

                x[0] = x[0] + vx[0] * 0.01;
                y[0] = y[0] + vy[0] * 0.01;
                z[0] = z[0] + vz[0] * 0.01;


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

		if(x[1]>0.85)
                      x[1]=0.8;
		else if(x[1]<-0.85)
		      x[1]=-0.80;

		if(y[1]>1.1)
                      y[1]=1.0;
		else if(y[1]<-1.1)
		      y[1]=-1.0;

		if(z[1]>0.65)
                      z[1]=0.6;


                pose.pose.position.x = y[1];  //zuobiao zhuanhuan
                pose.pose.position.y = x[1];
                pose.pose.position.z = z[1];


            }

        }

        local_pos_pub.publish(pose);

        //储存数据
        if(flag_save == 1)
        {
            save_flight_data(out_data_file,epock+1);
            //std::cout << "Saving Data!!" << std::endl;
        }


        if (epock >= 1500)
                 break;
        epock++;

        ros::spinOnce();
        rate.sleep();
    }



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

void save_flight_data(std::ofstream& out_file, int epock)
{
    out_file << epock << "  " << x[1] <<"  "<< y[1] <<"  "<< z[1] <<"  "\
                              << uxx[1] <<"  "<< uxy[1] <<"  "<< uxz[1] <<"  "\
                              << vx[1] <<"  "<< vy[1] <<"  "<< vz[1] <<"  "\
                              << ux[1] <<"  "<< uy[1] <<"  "<< uz[1] <<"  "\
                              << std::endl;
}


