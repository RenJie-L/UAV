#include <iostream>
#include <cmath>
#include<time.h>                                                                                                                                                                                                                                                                                                                                                                                
#include <fstream>

#include <algorithm>
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
#include <string>
#include <random>
#include <functional>
#include <assert.h>
#include <math.h>
//#include<wiringPi.h>
//#include<wiringSerial.h>
#include <python3.5/Python.h>
#include<stdio.h>
#include <serial/serial.h>
#include<unistd.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <ros/ros.h>
#define POSE_X          0   
#define POSE_Y          1    
#define YAW_ANGLE       2    
#define V_SPD           3    
#define W_ANGLE_SPD     4    

#define MD_MAX_V        0    
#define MD_MAX_W        1    
#define MD_ACC          2    
#define MD_VW           3    
#define MD_V_RESOLUTION 4    
#define MD_W_RESOLUTION 5    
using namespace std;
#define pi 3.1415926535898
#define PI 3.1415926535898



class GP
{
	public:
	
	// constructor & destructor
	GP(float g, float l, float data_noise); 
	~GP();

	float g;
	float l;
	float data_noise;

	// NOISE PARAMETER
	float sigma_omega; // 0.01 works

	// TRAINING DATA
	// Define data matrix that holds location data
	vector<vector<float>> X_train;
        vector<vector<float>> X_test;
	// Define data matrix that holds the train time data
	Eigen::VectorXd Y_train;
	vector<float> Y_train1;
        
	// TEST TIME DATA
	// Define data matrix that holds the test time data as well as
	// predicted means and covariances
	

	Eigen::MatrixXd pred_path_var;
	Eigen::MatrixXd pred_path_mean;

	 
	// Compute the convetional data covariance matrix 
	Eigen::MatrixXd compute_data_cov(Eigen::MatrixXd X);
	
	// Compute the scalar kernel value based on two vectors
	float scalar_kernel_f2vect(Eigen::VectorXd x_a, Eigen::VectorXd x_b, float g, float l);

	// Compute the scalar kernel value based on two scalars
	//double scalar_kernel_f2scalar(double a_1, double a_2, double g, double l);
		
	// compute the kernel-covariance matrix of a matrix X
	Eigen::MatrixXd kernel_cov_matrix_f1matrix(Eigen::MatrixXd X, float g, float l);

	// compute an augmented kernel covariance matrix based on a matrix X and an additional vector x_new
	Eigen::MatrixXd augmented_kernel_cov_matrix(Eigen::MatrixXd X, Eigen::VectorXd x_new, float g, float l);

	// Compute the kernel matrix based on two vectors
	//Eigen::MatrixXd kernel_matrix_f2vect(Eigen::VectorXd x_a, Eigen::VectorXd x_b, double g, double l);

	// Get the mean values at given points
     //Eigen::MatrixXd pred_mean(Eigen::MatrixXd X_train, Eigen::VectorXd Y_train, Eigen::MatrixXd X_test); 
      Eigen::MatrixXd pred_mean(vector<vector<float>> X_train, vector<float> Y_train1, vector<vector<float>> X_test);
	// Get the covariances at all pairs of given points

	//Eigen::MatrixXd pred_var(Eigen::MatrixXd X_train, Eigen::MatrixXd X_test);
        Eigen::MatrixXd pred_var(vector<vector<float>> X_train, vector<vector<float>> X_test);
 
       float scalar_kernel_f2scalar_2d(float x_1, float y_1,float x_2, float y_2,float g, float l);
   

	//Eigen::MatrixXd kernel_matrix_f2vect(Eigen::MatrixXd X_train, Eigen::MatrixXd X_test, double g, double l);
        Eigen::MatrixXd kernel_matrix_f2vect(vector<vector<float>> X_train, vector<vector<float>>  X_test, float g, float l);

  



};

typedef unsigned int uint;
float GP::scalar_kernel_f2scalar_2d(float x_1, float y_1,float x_2, float y_2,float g, float l) {
	
	float g_SE = g; // =1 works
	float l_SE = l; // =10 works
	float squared_distance = pow((x_1-x_2),2)+pow((y_1-y_2),2);
	float kernel = pow(g_SE, 2) * exp((-squared_distance)/(2*pow(l_SE,2)));
	return kernel;
}
/*
Eigen::MatrixXd GP::kernel_matrix_f2vect(Eigen::MatrixXd X_train, Eigen::MatrixXd X_test, double g, double l) {

   uint dim_a = X_train.rows();
	uint dim_b = X_test.rows();
  

	Eigen::MatrixXd kernel_matrix(dim_a,dim_b);
    for (uint i=0; i<dim_a; i++) {
		for (uint j=0; j<dim_b; j++) {
			kernel_matrix(i,j) = scalar_kernel_f2scalar_2d(X_train(i,0),X_train(i,1),X_test(j,0),X_test(j,1),g,l);
		}	
	}
	return kernel_matrix; // dim: (dim_a)x(dim_b)


}

Eigen::MatrixXd GP::pred_mean(Eigen::MatrixXd X_train, Eigen::VectorXd Y_train, Eigen::MatrixXd X_test) 
{
	uint N = Y_train.rows();

	Eigen::MatrixXd I;
	I.setIdentity(N,N);
	
	// noise=0.01 works
	double sigma_omega = data_noise; 

	Eigen::MatrixXd k_X_test_train = kernel_matrix_f2vect(X_train,X_test,g,l);
	Eigen::MatrixXd k_X_test_train_transpose = k_X_test_train.transpose().eval();
	// print out k_t_test_trai
	//cout << "pred_mean(): K_t_test_train ---------------" << endl;
	//cout << k_t_test_train << endl;
	
	Eigen::MatrixXd K_X_train = kernel_matrix_f2vect(X_train, X_train,g,l) + pow(sigma_omega,2) * I;
	Eigen::MatrixXd K_X_train_inv = K_X_train.inverse();
	// print out k_t_train_inv
	//cout << "pred_mean(): K_t_train_inv ---------------" << endl;
	//cout << K_t_train_inv << endl;

	Eigen::MatrixXd mean;
	mean = k_X_test_train * K_X_train_inv * Y_train; 

	//cout << mean.col(0) << endl;
	//(TxN)(NxN)(NxD) = (TxD)
	return mean;
}


// COMPUTE EQ 12
// Function to compute the variance of t* for new data input x*
Eigen::MatrixXd GP::pred_var(Eigen::MatrixXd X_train,  Eigen::MatrixXd X_test ) 
{
	
	uint N = X_train.rows();

	Eigen::MatrixXd I;
	I.setIdentity(N,N);
	// noise=0.01 works
	double sigma_omega = data_noise;

	
	Eigen::MatrixXd K_X_test = kernel_matrix_f2vect(X_test,X_test, g, l);
	// print out K_t_test
	//cout << "pred_var(): K_t_test ---------------" << endl;
	//cout << K_t_test << endl;
	
	
	Eigen::MatrixXd K_X_test_train = kernel_matrix_f2vect(X_test,X_train, g, l);
	// print out K_t_test_train
	//cout << "pred_var(): K_t_test_train ---------------" << endl;
	//cout << K_t_test_train << endl;

	 
	Eigen::MatrixXd K_X_train  = kernel_matrix_f2vect(X_train, X_train, g, l) + pow(sigma_omega,2) * I;
	// print out K_t_train
	//cout << "pred_var(): K_t_train ---------------" << endl;
	//cout << K_t_train << endl;
	
	
	Eigen::MatrixXd var;
	var = K_X_test - K_X_test_train * K_X_train.inverse() * K_X_test_train.transpose().eval();
	// print out predicted variance
	//cout << "var ---------------" << endl;
	//cout << var << endl;

	return var; // (TxT)-(TxN)(NxN)(NxT)
}*/
/*****************/
Eigen::MatrixXd GP::kernel_matrix_f2vect(vector<vector<float>> X_train, vector<vector<float>> X_test, float g, float l) {

        uint dim_a = X_train.size();
	uint dim_b = X_test.size();
  

	Eigen::MatrixXd kernel_matrix(dim_a,dim_b);
    for (uint i=0; i<dim_a; i++) {
		for (uint j=0; j<dim_b; j++) {
			kernel_matrix(i,j) = scalar_kernel_f2scalar_2d(X_train[i][0],X_train[i][0],X_test[j][0],X_test[j][0],g,l);
		}	
	}
	return kernel_matrix; // dim: (dim_a)x(dim_b)
}


Eigen::MatrixXd GP::pred_mean(vector<vector<float>> X_train, vector<float> Y_train1, vector<vector<float>> X_test) 
{       
        Eigen::VectorXd Y_train;
	
        for(float a:Y_train1)
          Y_train<<a;
  
	uint N =X_train.size();
        
	Eigen::MatrixXd I;
	I.setIdentity(N,N);


	// noise=0.01 works
	float sigma_omega = data_noise; 

	//Eigen::MatrixXd k_X_test_train = kernel_matrix_f2vect(X_train,X_test,g,l);
        
	//Eigen::MatrixXd k_X_test_train_transpose = k_X_test_train.transpose().eval();
        
	// print out k_t_test_trai
	//cout << "pred_mean(): K_t_test_train ---------------" << endl;
	//cout << k_t_test_train << endl;
	
	Eigen::MatrixXd K_X_train = kernel_matrix_f2vect(X_train, X_train,g,l) + pow(sigma_omega,2) * I;
       
	//Eigen::MatrixXd K_X_train_inv = K_X_train.inverse();
          
	// print out k_t_train_inv
	//cout << "pred_mean(): K_t_train_inv ---------------" << endl;
	//cout << K_t_train_inv << endl;

	Eigen::MatrixXd mean;
	//mean = k_X_test_train * K_X_train_inv * Y_train; 
	mean = K_X_train; 
      
	//cout << mean.col(0) << endl;
	//(TxN)(NxN)(NxD) = (TxD)
	return mean;
}


// COMPUTE EQ 12
// Function to compute the variance of t* for new data input x*
Eigen::MatrixXd GP::pred_var(vector<vector<float>> X_train,  vector<vector<float>> X_test) 
{
	
	uint N = X_train.size();

	Eigen::MatrixXd I;
	I.setIdentity(N,N);
	// noise=0.01 works
	float sigma_omega = data_noise;

	
	Eigen::MatrixXd K_X_test = kernel_matrix_f2vect(X_test,X_test, g, l);
	// print out K_t_test
	//cout << "pred_var(): K_t_test ---------------" << endl;
	//cout << K_t_test << endl;
	
	
	Eigen::MatrixXd K_X_test_train = kernel_matrix_f2vect(X_test,X_train, g, l);
	// print out K_t_test_train
	//cout << "pred_var(): K_t_test_train ---------------" << endl;
	//cout << K_t_test_train << endl;

	 
	Eigen::MatrixXd K_X_train  = kernel_matrix_f2vect(X_train, X_train, g, l) + pow(sigma_omega,2) * I;
	// print out K_t_train
	//cout << "pred_var(): K_t_train ---------------" << endl;
	//cout << K_t_train << endl;
	
	
	Eigen::MatrixXd var;
	var = K_X_test - K_X_test_train * K_X_train.inverse() * K_X_test_train.transpose().eval();
	// print out predicted variance
	//cout << "var ---------------" << endl;
	//cout << var << endl;

	return var; // (TxT)-(TxN)(NxN)(NxT)
}




/******************/                                                                                                         
GP::GP(float kenrel_g_se=1,float kernel_l_se=3,float sigma_noise=0.3):g(kenrel_g_se),l(kernel_l_se),data_noise(sigma_noise)
{}
GP::~GP()
{}

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

float x[2],y[2],z[2];
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
	float obstacleR = 0.1; //????????????????????????????????????????????
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
geometry_msgs::Pose pos_drone_fcu;                           //????????????????????? (??????fcu)
float Euler_fcu[3];                                          //????????????????????????(??????fcu)
mavros_msgs::State current_state;

void save_flight_data(std::ofstream& out_file, int timenow);   //??????????????????
void quaternion_2_euler(float quat[4], float angle[3]);        //?????????????????????[???PX4??????]




void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void optitrack_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //?????? -- optitrack??? ??? NED???
    x[0] = msg->pose.position.x;
    y[0] = msg->pose.position.z;
    z[0] = msg->pose.position.y;

    //????????????
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


//
serial::Serial ser;
void write_callback(const std_msgs::String::ConstPtr& msg)
{
ROS_INFO_STREAM("serial port"<<msg->data);
ser.write(msg->data);
}



int main(int argc, char **argv)
{       
      std_msgs::String result;
      static serial::Serial ser;
        vector<vector<float>> X_train;
         vector<vector<float>> X_test;
        vector<float> Y_train;
      
        vector<float> temp_test;
	//Eigen::MatrixXd X_test(900,2);
	float length_map=30.0;
	float part=3.0/length_map;


	for(int h=0;h<length_map;h++)
	{
		for(int k=0;k<length_map;k++)
		{
                   temp_test.push_back(part/2+part*(h)-1.5);
                   temp_test.push_back(part/2+part*(k)-1.5);
                   X_test.push_back(temp_test);
                   temp_test.clear();
	  
		}
	}
       
	  // Eigen::MatrixXd X_train(100,2);
	   //Eigen::VectorXd Y_train(100);

    /**************************************************************************************/
    ros::init(argc, argv, "demo_node");
    ros::NodeHandle nh;

    ros::Subscriber optitrack_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/Trackable1/pose", 100, optitrack_cb);

    // ????????????????????????????????? ?????????:NED??? [?????????GPS???????????????????????????mocap???] ???????????????????????????
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);

    // ???????????????????????????????????? ????????? NED??? ???????????????????????????
    ros::Subscriber euler_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, euler_cb);


    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber write_sub=nh.subscribe("write",1000,write_callback);

   ros::Publisher read_pub=nh.advertise<std_msgs::String>("read",1000);
   try{
         ser.setPort("/dev/ttyUSB0");
         ser.setBaudrate(115200);
         serial::Timeout to=serial::Timeout::simpleTimeout(1000);
         ser.setTimeout(to);
         ser.open();
    }
    catch(serial::IOException& e)
    {
       ROS_ERROR_STREAM("Unable to open port");
       return -1;
    }
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    int data_size=3;
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        if(ser.available())
         {
           ROS_INFO_STREAM("Reading from serial port\n");
           //ser.read(result);
           result.data=ser.read();
	   ROS_INFO_STREAM("Read: "<<result.data);
           read_pub.publish(result);
          }
        ros::spinOnce();
        rate.sleep();
    }

   
    /**************************************************************************************/
    int num_hang=0;

   
    //int fd;
    int n = 2;  //??????????????????
    double beta =0.6;
    double gamma =1.4;

    //????????????
    double A[2][2]={{0,0},{1,0}};

    double dx[2][2]={{0,0},{-0.4,0}};
    double dy[2][2]={{0,0},{0.4,0}};
    double dz[2][2]={{0,0},{0.4,0}};


    // // ???????????????
    for(int i=0;i<n;i++){
        x[i]=0.0;
        y[i]=0.0;
        z[i]=0.11;
    }

    //x[1]=-1.1;
   // y[1]=-1.1;
   

    // // ???????????????
    for(int i=0;i<n;i++){
        vx[i] =0;
        vy[i] =0;
        vz[i] =0;
    }
    int epock=0;                //???????????????1500????????????0.01???
  cout << "RHC start!!" << endl;
	state statex = { -1.1,-1.1,pi / 10,0,0 };
	 vector<state> static cartraj, realtraj; 
	float Kinematic[6] = { 0.2, DegreeToRadian(20.0), 2,DegreeToRadian(50.0), 0.01, DegreeToRadian(0.1) };
	Point end;
        end.x=1.2;
        end.y=1.2;
	
	float evalParam[4] = { 0.05, 0.2, 0.1, 3.0 };
	
	vector <Point> obstacle={ { 0, 0 },{ 0, 0.1 },{ 0, 0.2 },{ 0, 0.3},{ 0.1, 0 },{ 0.2, 0 },{ 0.3, 0 },{ 0.4, 0 }};

          /*obstacle.push_back({0,0});
          obstacle.push_back({0,0.1});
          obstacle.push_back({0,0.2});
          obstacle.push_back({0,0.3});
          obstacle.push_back({0.1,0});
          obstacle.push_back({0.2,0});
          obstacle.push_back({0.3,0});
          obstacle.push_back({0.4,0});*/             
	//vector <Point> obstacle = { {0, 20 }, {20, 40},{20, 50},{40, 20},{50, 40},{50, 50},{50, 60},{50, 70},{50, 90},{80, 80},{80, 90},{70, 90} };
	float obstacleR = 0.1; //
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
    pose.pose.position.z = 0.45;

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

    //???????????????
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
                
             

		controlU cu = DynamicWindowApproach(statex, &cartraj, Kinematic, end, evalParam, obstacle, obstacleR);
		statex = CarState(statex, cu);
		realtraj.push_back(statex);
		float tdist = sqrt((statex.x - end.x)*(statex.x - end.x) + (statex.y - end.y)*(statex.y - end.y));
		if (tdist < 0.1)
		{
			cout << "Arrive Goal!" << endl;
			is_end = true;
		}
                //Eigen::MatrixXd X_Train_temp(1,2);
                //Eigen::MatrixXd Y_Train_temp(1);
               // X_Train_temp<<statex.x,statex.y;
                //Y_Train_temp<<-res;
                //Eigen::MatrixXd X_train+=X_Train_temp;
               // Eigen::MatrixXd Y_train+=Y_train_temp;
                //X_train<<X_Train_temp;
                //Y_train<<Y_Train_temp;
                  vector<float> temp;
                  temp.push_back(statex.x);
                  temp.push_back(statex.y);
                  X_train.push_back(temp);
                  temp.clear();
                    //int int_result=atoi(result.data);
		 char *c=new char[20];
		 //strcpy(c,result.data.c_str());
                  //Y_train.push_back(result.data.c_str());
                   //Y_train.push_back((int)c);
                  Y_train.push_back(0.0);
                GP gp_debug{1.0,3.0,0.3};
               //Eigen::VectorXd
              auto mu_x = gp_debug.pred_mean(X_train, Y_train, X_test);
              //Eigen::VectorXd sigma_debug= gp_debug.pred_var(X_train, X_test);
                  if(epock<200)
                   {
                    end.x=1.2;
                    end.y=1.2;
                    }
                   else
                   {
		    float max; 
                   /* for(int i=0;i<mu_x.size();i++)
                    {  
	               max=mu_x(0);
	           	 if(mu_x(i)>max)
	           	 {
		      		max=mu_x(i);
		     	        num_hang=i;
	                 } 
                    }*/
		         //end.x=X_test.at(num_hang)[0];
		         //end.y=X_test.at(num_hang)[1];
                         //end.x=0.0;
                         //end.y=0.0;
		         cout<<end.x<<end.y; 
		    }
         
   
                y[1]=statex.x;
                x[1]=statex.y;
                z[1]=0.45;    
                //????????????
               if(x[1]>1.5)
                    x[1]=1.5;
                else if(x[1]<-1.5)
                    x[1]=-1.5;

                if(y[1]>1.5)
                    y[1]=1.5;
                else if(y[1]<-1.5)
                    y[1]=-1.5;

                if(z[1]>0.5)
                    z[1]=0.5;

               pose.pose.position.x = y[1];  //????????????????????????
               pose.pose.position.y = x[1];
               pose.pose.position.z = z[1];
               local_pos_pub.publish(pose);

            }

        }

       // local_pos_pub.publish(pose);

        //????????????
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



    //????????????????????????epock??????1500???????????????,???????????????????????????,???????????????,????????????????????????????????????auto land
    offb_set_mode.request.custom_mode = "AUTO.LAND";//?????????????????????????????????????????????
    while (ros::ok())
    {
        if (current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            //???????????????????????????auto land,??????????????????????????????????????????5s,????????????
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                //??????????????????????????????????????????
                ROS_INFO("AUTO.LAND enabled");
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


//??????????????????
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

// ????????????Euler
// q0 q1 q2 q3
// w x y z
void quaternion_2_euler(float quat[4], float angle[3])
{
    angle[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    angle[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), -1.0 + 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]));
    //angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
}




