#include <iostream>
#include <cmath>
#include <time.h>
#include <fstream>
#include <ros/ros.h>
#include<algorithm>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <stdio.h>
#define POSE_X          0    // ×ø±ê X
#define POSE_Y          1    // ×ø±ê Y
#define YAW_ANGLE       2    // »úÆ÷ÈËºœÏòœÇ
#define V_SPD           3    // »úÆ÷ÈËËÙ¶È
#define W_ANGLE_SPD     4    // »úÆ÷ÈËœÇËÙ¶È
//¶šÒåKinematicµÄÏÂ±êº¬Òå
#define MD_MAX_V        0    // ×îžßËÙ¶Èm / s]
#define MD_MAX_W        1    // ×îžßÐý×ªËÙ¶È[rad / s]
#define MD_ACC          2    // ŒÓËÙ¶È[m / ss]
#define MD_VW           3    // Ðý×ªŒÓËÙ¶È[rad / ss]
#define MD_V_RESOLUTION 4    // ËÙ¶È·Ö±æÂÊ[m / s]
#define MD_W_RESOLUTION 5    // ×ªËÙ·Ö±æÂÊ[rad / s]]
using namespace std;
#define pi 3.1415926535898
#define PI 3.1415926535898
struct state
{
	float x;
	float y;
	float yaw;
	float velocity;
	float angular;
};
struct controlU
{
	float vt;
	float wt;
};
struct maxmotion
{
	float minvel;
	float maxvel;
	float minang;
	float maxang;
};
struct eval_db
{
	float vt;
	float wt;
	float heading;
	float dist;
	float vel;
	float feval;
};
struct Point
{
	float x;
	float y;

};
float dt = 0.01;

float DegreeToRadian(float degree)
{
	return degree / 180 * pi;
}
float RadianToDegree(float radian)
{
	return radian / pi * 180;
}

float area[4] = { -1.5, 1.5, -1.5, 1.5 };

double x[2],y[2],z[2];
double uxx[2],uxy[2],uxz[2];
double vx[2],vy[2],vz[2];
double ux[2],uy[2],uz[2];
double x_last,y_last,z_last;

state CarState(state cx, controlU u)
{
	state result;
	result.x = cx.x + dt * cos(cx.yaw)*u.vt;
	result.y = cx.y + dt * sin(cx.yaw)*u.vt;
	result.yaw = cx.yaw + dt * u.wt;
	result.velocity = u.vt;
	result.angular = u.wt;
	return result;
}


maxmotion CalcDynamicWindow(state cx, float *model)
{
	maxmotion V_r;
        V_r.minvel = max(0.0f, cx.velocity - model[MD_ACC] * dt);
	V_r.maxvel = min(model[MD_MAX_V], cx.velocity + model[MD_ACC] * dt);
	V_r.minang = max(-model[MD_MAX_W], cx.angular - model[MD_VW] * dt);
	V_r.maxang = min(model[MD_MAX_W], cx.angular + model[MD_VW] * dt);
	return V_r;
}
float CalcHeadingEval(state cx, Point goal)
{
	float theta = RadianToDegree(cx.yaw); 
	float goalTheta = RadianToDegree(atan2(goal.y - cx.y, goal.x - cx.x));   
	float targetTheta;
	if (goalTheta > theta)
		targetTheta = goalTheta - theta; //[deg]
	else
		targetTheta = theta - goalTheta; //[deg]
	return 180 - targetTheta;
}

float CalcDistEval(state cx, vector<Point> ob, float R)
{
	float dist = 100.0;
	for (int i = 0; i < ob.size(); ++i)
	{
		
		float disttmp = sqrt((ob[i].x - cx.x)*(ob[i].x - cx.x) + (ob[i].y - cx.y)*(ob[i].y - cx.y)) - R;
		if (dist > disttmp)
			dist = disttmp;
	}
	if (dist >= 2 * R)
		dist = 2 * R;
	return dist;
}

float CalcBreakingDist(float vel, float mdacc)
{
	float stopDist = 0;
	while (vel > 0)
	{
		stopDist = stopDist + vel * dt; 
		vel = vel - mdacc*dt;
	}
	return stopDist;
}

state GenerateTrajectory(state cx, vector<state> *traj, float vt, float wt, float evaldt, float *model)
{
	float time = 0.0;
	controlU u = { vt, wt };
	traj->clear();
	traj->push_back(cx);
	state px = cx;
	while ((int)time < (int)evaldt * 10)
	{
		time = time + 10 * dt;
		px = CarState(px, u);
		traj->push_back(px);
	}
	return px;
}




void Evaluation(state cx, vector<eval_db> *EvalDb, vector<state> *TrajDb, maxmotion Vr, Point goal, vector<Point> ob, float R, float *model, float evaldt)
{
	EvalDb->clear();
	TrajDb->clear();
	vector<state> traj;
	for (float vt = Vr.minvel; vt <= Vr.maxvel; vt = vt + model[4])
	{
		for (float wt = Vr.minang; wt <= Vr.maxang; wt = wt + model[5])
		{
			
			state xt = GenerateTrajectory(cx, &traj, vt, wt, evaldt, model); 
			float heading = CalcHeadingEval(xt, goal);
			float dist = CalcDistEval(xt, ob, R);
			float vel = fabs(vt);
			float stopDist = CalcBreakingDist(vel, model[MD_ACC]); 
			eval_db db = { vt,wt,heading,dist,vel };
			if (dist > stopDist)
			{
				EvalDb->push_back(db);
				
			}
		}
	}
}



void NormalizeEval(vector<eval_db> *EvalDb)
{
	
	float sum3 = 0, sum4 = 0, sum5 = 0;
	for (int i = 0; i < EvalDb->size(); ++i)
	{
		sum3 += EvalDb->at(i).heading;
		sum4 += EvalDb->at(i).dist;
		sum5 += EvalDb->at(i).vel;
	}
	if (sum3 != 0)
	{
		for (int i = 0; i < EvalDb->size(); ++i)
			EvalDb->at(i).heading = EvalDb->at(i).heading / sum3;
	}
	if (sum4 != 0)
	{
		for (int i = 0; i < EvalDb->size(); ++i)
			EvalDb->at(i).dist = EvalDb->at(i).dist / sum4;
	}
	if (sum5 != 0)
	{
		for (int i = 0; i < EvalDb->size(); ++i)
			EvalDb->at(i).vel = EvalDb->at(i).vel / sum5;
	}
}

controlU DynamicWindowApproach(state cx, vector<state> *TrajDb, float *model, Point goal, float * evalParam, vector<Point> ob, float R)
{
	controlU u;
	vector<eval_db> EvalDb;
	maxmotion cvr = CalcDynamicWindow(cx, model); 
	Evaluation(cx, &EvalDb, TrajDb, cvr, goal, ob, R, model, evalParam[3]);  
	if (EvalDb.empty())
	{
		cout << "no path to goal!!" << endl;
		u.vt = 0;
		u.wt = 0;
		return u;
	}
	NormalizeEval(&EvalDb);
						  
	for (int i = 0; i < EvalDb.size(); ++i)
		EvalDb.at(i).feval = evalParam[0] * EvalDb.at(i).heading + evalParam[1] * EvalDb.at(i).dist + evalParam[2] * EvalDb.at(i).vel;
	float maxheading = EvalDb.at(0).feval;
	float idx = 0;
	for (int i = 0; i < EvalDb.size(); ++i)
	{
		if (maxheading < EvalDb.at(i).feval)
		{
			maxheading = EvalDb.at(i).feval;
			idx = i;
		}
	}
	u.vt = EvalDb.at(idx).vt;
	u.wt = EvalDb.at(idx).wt;
	return u;
}

state toshowcoodinate(state cx, float width, float height)
{
	state sx = { 0,0,0,0,0 };
	sx.x = -cx.y + width / 2;
	sx.y = -cx.x + height / 2;
	sx.yaw = -cx.yaw - pi / 2;
	return sx;
}
Point pointtoshowcoodinate(Point cx, float width, float height)
{
	Point sx = { 0,0 };
	sx.x = -cx.y + width / 2;
	sx.y = -cx.x + height / 2;
	return sx;
}


/*
void DwaSample()
{   
	/*int fd;
	if (wiringPiSetup() < 0)
	{
	return 1;
	if ((fd = serialOpen("/dev/ttyAMA0", 115200)) < 0)
	return 1;
	}


	cout << "RHC start!!" << endl;
	state statex = { -1.1,-1.1,pi / 10,0,0 };
	 vector<state> static cartraj, realtraj; 
	float Kinematic[6] = { 0.2, DegreeToRadian(20.0), 2, DegreeToRadian(50.0), 0.01, DegreeToRadian(0.1) };
	Point end;
        end.x=1.2;
        end.y=1.2;
	
	float evalParam[4] = { 0.05, 0.2, 0.1, 3.0 };
	
	vector <Point> obstacle = { { 0, 0 },{ 0, 0.1 },{ 0, 0.2 },{ 0, 0.3},{ 0.1, 0 },{ 0.2, 0 },{ 0.3, 0 },{ 0.4, 0 }};
	//vector <Point> obstacle = { {0, 20 }, {20, 40},{20, 50},{40, 20},{50, 40},{50, 50},{50, 60},{50, 70},{50, 90},{80, 80},{80, 90},{70, 90} };
	float obstacleR = 0.1; //³åÍ»ÅÐ¶šÓÃµÄÕÏ°­Îï°ëŸ¶
	bool is_end = false;
	int count = 0;
	while (!is_end)
	{
		
		controlU cu = DynamicWindowApproach(statex, &cartraj, Kinematic, end, evalParam, obstacle, obstacleR);
		statex = CarState(statex, cu);
		realtraj.push_back(statex);
		float tdist = sqrt((statex.x - end.x)*(statex.x - end.x) + (statex.y - end.y)*(statex.y - end.y));
		if (tdist < 0.1)
		{
			cout << "Arrive Goal!" << endl;
			is_end = true;
		}
		count++;
		
	}
} */
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
    double dy[2][2]={{0,0},{0.4,0}};
    double dz[2][2]={{0,0},{0.4,0}};


    // // 初始化位置
    for(int i=0;i<n;i++){
        x[i]=0;
        y[i]=0;
        z[i]=0.11;
    }

    x[1]=-1.1;
    y[1]=-1.1;


    // // 初始化速度
    for(int i=0;i<n;i++){
        vx[i] =0;
        vy[i] =0;
        vz[i] =0;
    }
    int epock=0;                //循环次数，1500步，每步0.01秒
  cout << "RHC start!!" << endl;
	state statex = { -1.1,-1.1,pi / 10,0,0 };
	 vector<state> static cartraj, realtraj; 
	float Kinematic[6] = { 0.2, DegreeToRadian(20.0), 2,DegreeToRadian(50.0), 0.01, DegreeToRadian(0.1) };
	Point end;
        end.x=1.2;
        end.y=1.2;
	
	float evalParam[4] = { 0.05, 0.2, 0.1, 3.0 };
	
	vector <Point> obstacle; 
//{ { 0, 0 },{ 0, 0.1 },{ 0, 0.2 },{ 0, 0.3},{ 0.1, 0 },{ 0.2, 0 },{ 0.3, 0 },{ 0.4, 0 }};

          obstacle.push_back({0,0});
          obstacle.push_back({0,0.1});
          obstacle.push_back({0,0.2});
          obstacle.push_back({0,0.3});
          obstacle.push_back({0.1,0});
          obstacle.push_back({0.2,0});
          obstacle.push_back({0.3,0});
          obstacle.push_back({0.4,0});             
	//vector <Point> obstacle = { {0, 20 }, {20, 40},{20, 50},{40, 20},{50, 40},{50, 50},{50, 60},{50, 70},{50, 90},{80, 80},{80, 90},{70, 90} };
	float obstacleR = 0.1; //³åÍ»ÅÐ¶šÓÃµÄÕÏ°­Îï°ëŸ¶
	bool is_end = false;
	int count = 0;
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
    pose.pose.position.x = statex.y;  //zuobiao zhuanhuan
    pose.pose.position.y = statex.x;
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


      
	//while (!is_end)
	//{
		
		controlU cu = DynamicWindowApproach(statex, &cartraj, Kinematic, end, evalParam, obstacle, obstacleR);
		statex = CarState(statex, cu);
		realtraj.push_back(statex);
		float tdist = sqrt((statex.x - end.x)*(statex.x - end.x) + (statex.y - end.y)*(statex.y - end.y));
		if (tdist < 0.1)
		{
			cout << "Arrive Goal!" << endl;
			is_end = true;
		}
                // 计算控制律
          
                

                y[1]=statex.x;
                x[1]=statex.y;
                z[1]=0.35;    
                //设置范围
                if(x[1]>1.5)
                    x[1]=1.5;
                else if(x[1]<-1.5)
                    x[1]=-1.5;

                if(y[1]>1.5)
                    y[1]=1.5;
                else if(y[1]<-1.5)
                    y[1]=-1.5;

                if(z[1]>0.4)
                    z[1]=0.4;

                pose.pose.position.x = y[1];  //此处坐标需要转换
                pose.pose.position.y = x[1];
                pose.pose.position.z = z[1];
                local_pos_pub.publish(pose);

            }

        }

       // local_pos_pub.publish(pose);

        //储存数据
        if(flag_save == 1)
        {
            save_flight_data(out_data_file,epock+1);
          //   std::cout << "Saving Data!!" << std::endl;
        }
        if (epock >= 1600)
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




