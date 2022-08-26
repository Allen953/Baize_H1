#include <iostream>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chainiksolverpos_nr_jl.hpp"
#include "trac_ik/trac_ik.hpp"
#include "urdf/model.h"
#include <math.h>
#include <serial/serial.h>
#include "geometry_msgs/PointStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#define pi 3.141592653
#define Amx 0.02
#define Amz 0.03
#define k 0
#define zhenshu 25

int servo[18]={
4,5,6,
7,8,9,
27,28,29,
24,25,26,
21,22,23,
10,11,12
};


int mid_c[18]={
1500,1500,1500,
1471,1500,1500,
1500,1500,1559,
1471,1500,1441,
1412,1500,1500,
1441,1529,1412
};

int joint_state_pwm[18]={
1500,1500,1500,
1500,1500,1500,
1500,1500,1500,
1500,1500,1500,
1500,1500,1500,
1500,1500,1500
};

int servo_direct[18]={
-1,1,1,
-1,1,1,
-1,1,1,
-1,1,1,
-1,1,1,
-1,1,1
};

char cmd='e';   //a:qianjin b:houtui c:zuozhuan d:youzhuan e:tingzhi

int main(int argc,char* argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"liuzu");
    ros::NodeHandle nh;
    ros::Publisher joint_pub=nh.advertise<sensor_msgs::JointState>("joint_states",1);
    tf2_ros::StaticTransformBroadcaster broadcaster;

    
/****************************************************串口操作*/
//sp  send data to servo control board
    //创建一个serial对象
    serial::Serial sp;
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");
    //设置串口通信的波特率
    sp.setBaudrate(9600);
    //串口设置timeout
    sp.setTimeout(to);
 
    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }

/****************************************************/


/****************************************************串口操作*/
//sp1:read command from blueteeth device
    //创建一个serial对象
    serial::Serial sp1;

    //设置要打开的串口名称
    sp1.setPort("/dev/ttyUSB0");
    //设置串口通信的波特率
    sp1.setBaudrate(9600);
    //串口设置timeout
    sp1.setTimeout(to);
 
    try
    {
        //打开串口
        sp1.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(sp1.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyACM2 is opened.");
    }
    else
    {
        return -1;
    }

/****************************************************/

    geometry_msgs::TransformStamped ts;
    KDL::Vector v1(1,1,1);



    KDL::Tree my_tree;
    sensor_msgs::JointState joint_state;

    std::string robot_desc_string;
    nh.param("robot_description", robot_desc_string, std::string());

    if(!kdl_parser::treeFromString(robot_desc_string, my_tree))
    //if(!kdl_parser::treeFromFile("/home/zhitong/catkin_ws3/src/LIUZU/urdf/LIUZU.urdf", my_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
    }
    else
    {
        ROS_INFO("成功生成kdl树!");
    }

    std::vector<std::string> joint_name = {
"RB_DATUI_JOINT", "RB_XI_JOINT", "RB_XIAOTUI_JOINT", "RB_R_JOINT","RB_P_JOINT","RB_Y_JOINT",
"RM_DATUI_JOINT", "RM_XI_JOINT", "RM_XIAOTUI_JOINT", "RM_R_JOINT","RM_P_JOINT","RM_Y_JOINT",
"RF_DATUI_JOINT", "RF_XI_JOINT", "RF_XIAOTUI_JOINT", "RF_R_JOINT","RF_P_JOINT","RF_Y_JOINT",
"LF_DATUI_JOINT", "LF_XI_JOINT", "LF_XIAOTUI_JOINT", "LF_R_JOINT","LF_P_JOINT","LF_Y_JOINT",
"LM_DATUI_JOINT", "LM_XI_JOINT", "LM_XIAOTUI_JOINT", "LM_R_JOINT","LM_P_JOINT","LM_Y_JOINT",
"LB_DATUI_JOINT", "LB_XI_JOINT", "LB_XIAOTUI_JOINT", "LB_R_JOINT","LB_P_JOINT","LB_Y_JOINT"
};

    std::vector<double> joint_pos = {
0,0,0,0,0,0,
0,0,0,0,0,0,
0,0,0,0,0,0,
0,0,0,0,0,0,
0,0,0,0,0,0,
0,0,0,0,0,0
};
    std::string urdf_param = "/robot_description";
    double timeout = 0.005;
    double eps = 1e-5;
    std::string chain_start  = "base_link"; 
    std::string chain_rb_end = "RB_Y_LINK"; 
    std::string chain_rm_end = "RM_Y_LINK"; 
    std::string chain_rf_end = "RF_Y_LINK"; 
    std::string chain_lf_end = "LF_Y_LINK"; 
    std::string chain_lm_end = "LM_Y_LINK"; 
    std::string chain_lb_end = "LB_Y_LINK"; 

    TRAC_IK::TRAC_IK tracik_rb_solver(chain_start, chain_rb_end, urdf_param, timeout, eps);
    TRAC_IK::TRAC_IK tracik_rm_solver(chain_start, chain_rm_end, urdf_param, timeout, eps);
    TRAC_IK::TRAC_IK tracik_rf_solver(chain_start, chain_rf_end, urdf_param, timeout, eps);
    TRAC_IK::TRAC_IK tracik_lf_solver(chain_start, chain_lf_end, urdf_param, timeout, eps);
    TRAC_IK::TRAC_IK tracik_lm_solver(chain_start, chain_lm_end, urdf_param, timeout, eps);
    TRAC_IK::TRAC_IK tracik_lb_solver(chain_start, chain_lb_end, urdf_param, timeout, eps);
    
    KDL::Chain chain_rb;
    KDL::Chain chain_rm;
    KDL::Chain chain_rf;
    KDL::Chain chain_lf;
    KDL::Chain chain_lm;
    KDL::Chain chain_lb;


    KDL::JntArray ll, ul; //关节下限, 关节上限
    bool valid_rb = tracik_rb_solver.getKDLChain(chain_rb);
    bool valid_rm = tracik_rm_solver.getKDLChain(chain_rm);
    bool valid_rf = tracik_rf_solver.getKDLChain(chain_rf);
    bool valid_lf = tracik_lf_solver.getKDLChain(chain_lf);
    bool valid_lm = tracik_lm_solver.getKDLChain(chain_lm);
    bool valid_lb = tracik_lb_solver.getKDLChain(chain_lb);
    if((!valid_rb)||(!valid_rm)||(!valid_rf)||(!valid_lf)||(!valid_lm)||(!valid_lb))
    {
        ROS_ERROR("There was no valid KDL chain found");
    }
    valid_rb = tracik_rb_solver.getKDLLimits(ll, ul);
    valid_rm = tracik_rm_solver.getKDLLimits(ll, ul);
    valid_rf = tracik_rf_solver.getKDLLimits(ll, ul);
    valid_lf = tracik_lf_solver.getKDLLimits(ll, ul);
    valid_lm = tracik_lm_solver.getKDLLimits(ll, ul);
    valid_lb = tracik_lb_solver.getKDLLimits(ll, ul);
    if((!valid_rb)||(!valid_rm)||(!valid_rf)||(!valid_lf)||(!valid_lm)||(!valid_lb))
    {
        ROS_ERROR("There was no valid KDL joint limits found");
    }
   
    KDL::ChainFkSolverPos_recursive fk_rb_solver(chain_rb);
    KDL::ChainFkSolverPos_recursive fk_rm_solver(chain_rm);
    KDL::ChainFkSolverPos_recursive fk_rf_solver(chain_rf);
    KDL::ChainFkSolverPos_recursive fk_lf_solver(chain_lf);
    KDL::ChainFkSolverPos_recursive fk_lm_solver(chain_lm);
    KDL::ChainFkSolverPos_recursive fk_lb_solver(chain_lb);

    ROS_INFO("rb_关节数量: %d", chain_rb.getNrOfJoints());
    ROS_INFO("rm_关节数量: %d", chain_rm.getNrOfJoints());
    ROS_INFO("rf_关节数量: %d", chain_rf.getNrOfJoints());
    ROS_INFO("lf_关节数量: %d", chain_lf.getNrOfJoints());
    ROS_INFO("lm_关节数量: %d", chain_lm.getNrOfJoints());
    ROS_INFO("lb_关节数量: %d", chain_lb.getNrOfJoints());
    KDL::JntArray nominal(36);

    ROS_INFO("the nominal size is:%d",nominal.data.size());

    for(size_t j = 0; j < 6; j ++)
    {
        nominal(j)=0.0;
        //nominal(j) = (ll(j) + ul(j))/2.0;
    }
    KDL::JntArray q_rb(6); // 初始关节位置
    KDL::JntArray q_rm(6); // 初始关节位置
    KDL::JntArray q_rf(6); // 初始关节位置
    KDL::JntArray q_lf(6); // 初始关节位置
    KDL::JntArray q_lm(6); // 初始关节位置
    KDL::JntArray q_lb(6); // 初始关节位置

    for(size_t j = 0; j < 6; j ++)
    {
        q_rb(j)=0.0;
        q_rm(j)=0.0;
        q_rf(j)=0.0;
        q_lf(j)=0.0;
        q_lm(j)=0.0;
        q_lb(j)=0.0;
     
    }
    
     //定义初始末端4x4齐次变换矩阵
    KDL::Frame rb_end_effector_pose;
    KDL::Frame rm_end_effector_pose;
    KDL::Frame rf_end_effector_pose;
    KDL::Frame lf_end_effector_pose;
    KDL::Frame lm_end_effector_pose;
    KDL::Frame lb_end_effector_pose;
    //定义逆运动学解算结果存储数组
    KDL::JntArray result_rb;
    KDL::JntArray result_rm;
    KDL::JntArray result_rf;
    KDL::JntArray result_lf;
    KDL::JntArray result_lm;
    KDL::JntArray result_lb;

    ros::Rate r(zhenshu);

    auto print_frame_lambda = [](KDL::Frame f)
    {
        double x, y, z, roll, pitch, yaw;
        x = f.p.x();
        y = f.p.y();
        z = f.p.z();
        f.M.GetRPY(roll, pitch, yaw);
        std::cout << "x:" << x << " y:" << y << " z:" << z << " roll:" << roll << " pitch:" << pitch << " yaw:" << yaw << std::endl;
    };
    
    //正运动学
    fk_rb_solver.JntToCart(q_rb,rb_end_effector_pose);
    fk_rm_solver.JntToCart(q_rm,rm_end_effector_pose);
    fk_rf_solver.JntToCart(q_rf,rf_end_effector_pose);
    fk_lf_solver.JntToCart(q_lf,lf_end_effector_pose);
    fk_lm_solver.JntToCart(q_lm,lm_end_effector_pose);
    fk_lb_solver.JntToCart(q_lb,lb_end_effector_pose);

    //逆运动学
    int rc_rb = tracik_rb_solver.CartToJnt(q_rb, rb_end_effector_pose, result_rb);
    int rc_rm = tracik_rm_solver.CartToJnt(q_rm, rm_end_effector_pose, result_rm);
    int rc_rf = tracik_rf_solver.CartToJnt(q_rf, rf_end_effector_pose, result_rf);
    int rc_lf = tracik_lf_solver.CartToJnt(q_lf, lf_end_effector_pose, result_lf);
    int rc_lm = tracik_lm_solver.CartToJnt(q_lm, lm_end_effector_pose, result_lm);
    int rc_lb = tracik_lb_solver.CartToJnt(q_lb, lb_end_effector_pose, result_lb);

    ROS_INFO("1:%f,2:%f,3:%f,4:%f,5:%f,6:%f",result_rb(0),result_rb(1),result_rb(2),result_rb(3),result_rb(4),result_rb(5));
    ROS_INFO("1:%f,2:%f,3:%f,4:%f,5:%f,6:%f",result_rm(0),result_rm(1),result_rm(2),result_rm(3),result_rm(4),result_rm(5));
    ROS_INFO("1:%f,2:%f,3:%f,4:%f,5:%f,6:%f",result_rf(0),result_rf(1),result_rf(2),result_rf(3),result_rf(4),result_rf(5));
    ROS_INFO("1:%f,2:%f,3:%f,4:%f,5:%f,6:%f",result_lf(0),result_lf(1),result_lf(2),result_lf(3),result_lf(4),result_lf(5));
    ROS_INFO("1:%f,2:%f,3:%f,4:%f,5:%f,6:%f",result_lm(0),result_lm(1),result_lm(2),result_lm(3),result_lm(4),result_lm(5));
    ROS_INFO("1:%f,2:%f,3:%f,4:%f,5:%f,6:%f",result_lb(0),result_lb(1),result_lb(2),result_lb(3),result_lb(4),result_lb(5));
    
    print_frame_lambda(rb_end_effector_pose);
    print_frame_lambda(rm_end_effector_pose);
    print_frame_lambda(rf_end_effector_pose);
    print_frame_lambda(lf_end_effector_pose);
    print_frame_lambda(lm_end_effector_pose);
    print_frame_lambda(lb_end_effector_pose);


    ROS_INFO("更新关节状态");
    joint_state.header.stamp = ros::Time::now();
    
    joint_state.name.resize(36);
    joint_state.position.resize(36);
    
    for(size_t i = 0; i < 36; i ++)
    {
        joint_state.name[i] = joint_name[i];
        joint_state.position[i] = 0;
    }


    for(size_t i = 0; i < 6; i ++)
    {
        joint_state.position[i] = result_rb(i);
    }
    for(size_t i = 6; i < 12; i ++)
    {
        joint_state.position[i] = result_rm(i-6);
    }
    for(size_t i = 12; i < 18; i ++)
    {
        joint_state.position[i] = result_rf(i-12);
    }
    for(size_t i = 18; i < 24; i ++)
    {
        joint_state.position[i] = result_lf(i-18);
    }
    for(size_t i = 24; i < 30; i ++)
    {
        joint_state.position[i] = result_lm(i-24);
    }
    for(size_t i = 30; i < 36; i ++)
    {
        joint_state.position[i] = result_lb(i-30);
    }

    
    joint_pub.publish(joint_state);
    
    KDL::Frame rb_end_effector_pose1;//now
    KDL::Frame rm_end_effector_pose1;//now
    KDL::Frame rf_end_effector_pose1;//now
    KDL::Frame lf_end_effector_pose1;//now
    KDL::Frame lm_end_effector_pose1;//now
    KDL::Frame lb_end_effector_pose1;//now

    fk_rb_solver.JntToCart(q_rb,rb_end_effector_pose1);
    fk_rm_solver.JntToCart(q_rm,rm_end_effector_pose1);
    fk_rf_solver.JntToCart(q_rf,rf_end_effector_pose1);
    fk_lf_solver.JntToCart(q_lf,lf_end_effector_pose1);
    fk_lm_solver.JntToCart(q_lm,lm_end_effector_pose1);
    fk_lb_solver.JntToCart(q_lb,lb_end_effector_pose1);

    KDL::JntArray resu_last_rb;//last
    KDL::JntArray resu_last_rm;//last
    KDL::JntArray resu_last_rf;//last
    KDL::JntArray resu_last_lf;//last
    KDL::JntArray resu_last_lm;//last
    KDL::JntArray resu_last_lb;//last

    resu_last_rb=q_rb;
    resu_last_rm=q_rm;
    resu_last_rf=q_rf;
    resu_last_lf=q_lf;
    resu_last_lm=q_lm;
    resu_last_lb=q_lb;


    while(ros::ok())
    {
        size_t n = sp1.available();
        if(n!=0)
        {
            uint8_t buffer[1024];
            n = sp1.read(buffer, n);
            cmd=buffer[0];
        }

    //x=0.02*(t-sint);
    //y=0.02*(1-cost);
    if(cmd=='a')
    {
	    for(int i=1;i<=20;i++)
	    { 
		double t=pi*i/20;
		double x=Amx*t;
		double z=Amz*sin(t)+k;

		rb_end_effector_pose1.p.data[0]=rb_end_effector_pose.p.data[0]-Amx*pi/2+x;
		rb_end_effector_pose1.p.data[2]=rb_end_effector_pose.p.data[2]+z-0.04;
		int rc_rb = tracik_rb_solver.CartToJnt(resu_last_rb, rb_end_effector_pose1, result_rb);

		rm_end_effector_pose1.p.data[0]=rm_end_effector_pose.p.data[0]-Amx*pi/2+Amx*pi-x;
		rm_end_effector_pose1.p.data[2]=rm_end_effector_pose.p.data[2]-0.04;
		int rc_rm = tracik_rm_solver.CartToJnt(resu_last_rm, rm_end_effector_pose1, result_rm);

		rf_end_effector_pose1.p.data[0]=rf_end_effector_pose.p.data[0]-Amx*pi/2+x;
		rf_end_effector_pose1.p.data[2]=rf_end_effector_pose.p.data[2]+z-0.04;
		int rc_rf = tracik_rf_solver.CartToJnt(resu_last_rf, rf_end_effector_pose1, result_rf);

		lf_end_effector_pose1.p.data[0]=lf_end_effector_pose.p.data[0]-Amx*pi/2+Amx*pi-x;
		lf_end_effector_pose1.p.data[2]=lf_end_effector_pose.p.data[2]-0.04;
		int rc_lf = tracik_lf_solver.CartToJnt(resu_last_lf, lf_end_effector_pose1, result_lf);

		lm_end_effector_pose1.p.data[0]=lm_end_effector_pose.p.data[0]-Amx*pi/2+x;
		lm_end_effector_pose1.p.data[2]=lm_end_effector_pose.p.data[2]+z-0.04;
		int rc_lm = tracik_lm_solver.CartToJnt(resu_last_lm, lm_end_effector_pose1, result_lm);

		lb_end_effector_pose1.p.data[0]=lb_end_effector_pose.p.data[0]-Amx*pi/2+Amx*pi-x;
		lb_end_effector_pose1.p.data[2]=lb_end_effector_pose.p.data[2]-0.04;
		int rc_lb = tracik_lb_solver.CartToJnt(resu_last_lb, lb_end_effector_pose1, result_lb);

		joint_state.header.stamp = ros::Time::now();
		    for(size_t i = 0; i < 6; i ++)
		    {
			joint_state.position[i] = result_rb(i);
		    }
		    for(size_t i = 6; i < 12; i ++)
		    {
			joint_state.position[i] = result_rm(i-6);
		    }
		    for(size_t i = 12; i < 18; i ++)
		    {
			joint_state.position[i] = result_rf(i-12);
		    }
		    for(size_t i = 18; i < 24; i ++)
		    {
			joint_state.position[i] = result_lf(i-18);
		    }
		    for(size_t i = 24; i < 30; i ++)
		    {
			joint_state.position[i] = result_lm(i-24);
		    }
		    for(size_t i = 30; i < 36; i ++)
		    {
			joint_state.position[i] = result_lb(i-30);
		    }
		std::string servo_cmd="";

		joint_state_pwm[0]=mid_c[0]+servo_direct[0]*int((2000.0/180.0)*joint_state.position[0]*180.0/pi);
		joint_state_pwm[1]=mid_c[1]+servo_direct[1]*int((2000.0/180.0)*joint_state.position[1]*180.0/pi);
		joint_state_pwm[2]=mid_c[2]+servo_direct[2]*int((2000.0/180.0)*joint_state.position[2]*180.0/pi);
		joint_state_pwm[3]=mid_c[3]+servo_direct[3]*int((2000.0/180.0)*joint_state.position[6]*180.0/pi);
		joint_state_pwm[4]=mid_c[4]+servo_direct[4]*int((2000.0/180.0)*joint_state.position[7]*180.0/pi);
		joint_state_pwm[5]=mid_c[5]+servo_direct[5]*int((2000.0/180.0)*joint_state.position[8]*180.0/pi);
		joint_state_pwm[6]=mid_c[6]+servo_direct[6]*int((2000.0/180.0)*joint_state.position[12]*180.0/pi);
		joint_state_pwm[7]=mid_c[7]+servo_direct[7]*int((2000.0/180.0)*joint_state.position[13]*180.0/pi);
		joint_state_pwm[8]=mid_c[8]+servo_direct[8]*int((2000.0/180.0)*joint_state.position[14]*180.0/pi);
		joint_state_pwm[9]=mid_c[9]+servo_direct[9]*int((2000.0/180.0)*joint_state.position[18]*180.0/pi);
		joint_state_pwm[10]=mid_c[10]+servo_direct[10]*int((2000.0/180.0)*joint_state.position[19]*180.0/pi);
		joint_state_pwm[11]=mid_c[11]+servo_direct[11]*int((2000.0/180.0)*joint_state.position[20]*180.0/pi);
		joint_state_pwm[12]=mid_c[12]+servo_direct[12]*int((2000.0/180.0)*joint_state.position[24]*180.0/pi);
		joint_state_pwm[13]=mid_c[13]+servo_direct[13]*int((2000.0/180.0)*joint_state.position[25]*180.0/pi);
		joint_state_pwm[14]=mid_c[14]+servo_direct[14]*int((2000.0/180.0)*joint_state.position[26]*180.0/pi);
		joint_state_pwm[15]=mid_c[15]+servo_direct[15]*int((2000.0/180.0)*joint_state.position[30]*180.0/pi);
		joint_state_pwm[16]=mid_c[16]+servo_direct[16]*int((2000.0/180.0)*joint_state.position[31]*180.0/pi);
		joint_state_pwm[17]=mid_c[17]+servo_direct[17]*int((2000.0/180.0)*joint_state.position[32]*180.0/pi);

		for(size_t i=0;i<18;i++)
		{
		    servo_cmd=servo_cmd+"#"+std::to_string(servo[i])+"P"+std::to_string(joint_state_pwm[i]);
		}
		servo_cmd=servo_cmd+"T100";

		uint8_t buffer[servo_cmd.length()+2];
		
		for(size_t i=0;i<servo_cmd.length();i++)
			buffer[i]=servo_cmd[i];
			buffer[servo_cmd.length()]=0x0D;
			buffer[servo_cmd.length()+1]=0x0A;

		sp.write(buffer,servo_cmd.length()+2);

	      	ROS_INFO("%s",buffer);  
		//ROS_INFO("%s",servo_cmd.c_str());	

		joint_pub.publish(joint_state);
	 
		resu_last_rb=result_rb;
		resu_last_rm=result_rm;
		resu_last_rf=result_rf;
		resu_last_lf=result_lf;
		resu_last_lm=result_lm;
		resu_last_lb=result_lb;
		r.sleep();
	    }
	    
	 
	    for(int i=1;i<=20;i++)
	    { 
		double t=pi*i/20;
		double x=Amx*t;
		double z=Amz*sin(t)+k;
		rb_end_effector_pose1.p.data[0]=rb_end_effector_pose.p.data[0]-Amx*pi/2+Amx*pi-x;
		rb_end_effector_pose1.p.data[2]=rb_end_effector_pose.p.data[2]-0.04;
		int rc_rb = tracik_rb_solver.CartToJnt(resu_last_rb, rb_end_effector_pose1, result_rb);

		rm_end_effector_pose1.p.data[0]=rm_end_effector_pose.p.data[0]-Amx*pi/2+x;
		rm_end_effector_pose1.p.data[2]=rm_end_effector_pose.p.data[2]+z-0.04;
		int rc_rm = tracik_rm_solver.CartToJnt(resu_last_rm, rm_end_effector_pose1, result_rm);

		rf_end_effector_pose1.p.data[0]=rf_end_effector_pose.p.data[0]-Amx*pi/2+Amx*pi-x;
		rf_end_effector_pose1.p.data[2]=rf_end_effector_pose.p.data[2]-0.04;
		int rc_rf = tracik_rf_solver.CartToJnt(resu_last_rf, rf_end_effector_pose1, result_rf);

		lf_end_effector_pose1.p.data[0]=lf_end_effector_pose.p.data[0]-Amx*pi/2+x;
		lf_end_effector_pose1.p.data[2]=lf_end_effector_pose.p.data[2]+z-0.04;
		int rc_lf = tracik_lf_solver.CartToJnt(resu_last_lf, lf_end_effector_pose1, result_lf);

		lm_end_effector_pose1.p.data[0]=lm_end_effector_pose.p.data[0]-Amx*pi/2+Amx*pi-x;
		lm_end_effector_pose1.p.data[2]=lm_end_effector_pose.p.data[2]-0.04;
		int rc_lm = tracik_lm_solver.CartToJnt(resu_last_lm, lm_end_effector_pose1, result_lm);

		lb_end_effector_pose1.p.data[0]=lb_end_effector_pose.p.data[0]-Amx*pi+x;
		lb_end_effector_pose1.p.data[2]=lb_end_effector_pose.p.data[2]+z-0.04;
		int rc_lb = tracik_lb_solver.CartToJnt(resu_last_lb, lb_end_effector_pose1, result_lb);
	 
		joint_state.header.stamp = ros::Time::now();
		    for(size_t i = 0; i < 6; i ++)
		    {
			joint_state.position[i] = result_rb(i);
		    }
		    for(size_t i = 6; i < 12; i ++)
		    {
			joint_state.position[i] = result_rm(i-6);
		    }
		    for(size_t i = 12; i < 18; i ++)
		    {
			joint_state.position[i] = result_rf(i-12);
		    }
		    for(size_t i = 18; i < 24; i ++)
		    {
			joint_state.position[i] = result_lf(i-18);
		    }
		    for(size_t i = 24; i < 30; i ++)
		    {
			joint_state.position[i] = result_lm(i-24);
		    }
		    for(size_t i = 30; i < 36; i ++)
		    {
			joint_state.position[i] = result_lb(i-30);
		    }
		
		std::string servo_cmd="";

		joint_state_pwm[0]=mid_c[0]+servo_direct[0]*int((2000.0/180.0)*joint_state.position[0]*180.0/pi);
		joint_state_pwm[1]=mid_c[1]+servo_direct[1]*int((2000.0/180.0)*joint_state.position[1]*180.0/pi);
		joint_state_pwm[2]=mid_c[2]+servo_direct[2]*int((2000.0/180.0)*joint_state.position[2]*180.0/pi);
		joint_state_pwm[3]=mid_c[3]+servo_direct[3]*int((2000.0/180.0)*joint_state.position[6]*180.0/pi);
		joint_state_pwm[4]=mid_c[4]+servo_direct[4]*int((2000.0/180.0)*joint_state.position[7]*180.0/pi);
		joint_state_pwm[5]=mid_c[5]+servo_direct[5]*int((2000.0/180.0)*joint_state.position[8]*180.0/pi);
		joint_state_pwm[6]=mid_c[6]+servo_direct[6]*int((2000.0/180.0)*joint_state.position[12]*180.0/pi);
		joint_state_pwm[7]=mid_c[7]+servo_direct[7]*int((2000.0/180.0)*joint_state.position[13]*180.0/pi);
		joint_state_pwm[8]=mid_c[8]+servo_direct[8]*int((2000.0/180.0)*joint_state.position[14]*180.0/pi);
		joint_state_pwm[9]=mid_c[9]+servo_direct[9]*int((2000.0/180.0)*joint_state.position[18]*180.0/pi);
		joint_state_pwm[10]=mid_c[10]+servo_direct[10]*int((2000.0/180.0)*joint_state.position[19]*180.0/pi);
		joint_state_pwm[11]=mid_c[11]+servo_direct[11]*int((2000.0/180.0)*joint_state.position[20]*180.0/pi);
		joint_state_pwm[12]=mid_c[12]+servo_direct[12]*int((2000.0/180.0)*joint_state.position[24]*180.0/pi);
		joint_state_pwm[13]=mid_c[13]+servo_direct[13]*int((2000.0/180.0)*joint_state.position[25]*180.0/pi);
		joint_state_pwm[14]=mid_c[14]+servo_direct[14]*int((2000.0/180.0)*joint_state.position[26]*180.0/pi);
		joint_state_pwm[15]=mid_c[15]+servo_direct[15]*int((2000.0/180.0)*joint_state.position[30]*180.0/pi);
		joint_state_pwm[16]=mid_c[16]+servo_direct[16]*int((2000.0/180.0)*joint_state.position[31]*180.0/pi);
		joint_state_pwm[17]=mid_c[17]+servo_direct[17]*int((2000.0/180.0)*joint_state.position[32]*180.0/pi);

		for(size_t i=0;i<18;i++)
		{
		    servo_cmd=servo_cmd+"#"+std::to_string(servo[i])+"P"+std::to_string(joint_state_pwm[i]);
		}
		servo_cmd=servo_cmd+"T100";

		uint8_t buffer[servo_cmd.length()+2];
		
		for(size_t i=0;i<servo_cmd.length();i++)
			buffer[i]=servo_cmd[i];
			buffer[servo_cmd.length()]=0x0D;
			buffer[servo_cmd.length()+1]=0x0A;
			
		
		sp.write(buffer,servo_cmd.length()+2);
		
	      	ROS_INFO("%s",buffer);  
		//ROS_INFO("%s",servo_cmd.c_str());

		joint_pub.publish(joint_state);
	 
		resu_last_rb=result_rb;
		resu_last_rm=result_rm;
		resu_last_rf=result_rf;
		resu_last_lf=result_lf;
		resu_last_lm=result_lm;
		resu_last_lb=result_lb;
		r.sleep();
	    }
    }
    else if(cmd=='b')
    {
	    for(int i=1;i<=20;i++)
	    { 
		double t=pi*i/20;
		double x=Amx*t;
		double z=Amz*sin(t)+k;

		rb_end_effector_pose1.p.data[0]=rb_end_effector_pose.p.data[0]+Amx*pi/2-x;
		rb_end_effector_pose1.p.data[2]=rb_end_effector_pose.p.data[2]+z-0.04;
		int rc_rb = tracik_rb_solver.CartToJnt(resu_last_rb, rb_end_effector_pose1, result_rb);

		rm_end_effector_pose1.p.data[0]=rm_end_effector_pose.p.data[0]+Amx*pi/2-Amx*pi+x;
		rm_end_effector_pose1.p.data[2]=rm_end_effector_pose.p.data[2]-0.04;
		int rc_rm = tracik_rm_solver.CartToJnt(resu_last_rm, rm_end_effector_pose1, result_rm);

		rf_end_effector_pose1.p.data[0]=rf_end_effector_pose.p.data[0]+Amx*pi/2-x;
		rf_end_effector_pose1.p.data[2]=rf_end_effector_pose.p.data[2]+z-0.04;
		int rc_rf = tracik_rf_solver.CartToJnt(resu_last_rf, rf_end_effector_pose1, result_rf);

		lf_end_effector_pose1.p.data[0]=lf_end_effector_pose.p.data[0]+Amx*pi/2-Amx*pi+x;
		lf_end_effector_pose1.p.data[2]=lf_end_effector_pose.p.data[2]-0.04;
		int rc_lf = tracik_lf_solver.CartToJnt(resu_last_lf, lf_end_effector_pose1, result_lf);

		lm_end_effector_pose1.p.data[0]=lm_end_effector_pose.p.data[0]+Amx*pi/2-x;
		lm_end_effector_pose1.p.data[2]=lm_end_effector_pose.p.data[2]+z-0.04;
		int rc_lm = tracik_lm_solver.CartToJnt(resu_last_lm, lm_end_effector_pose1, result_lm);

		lb_end_effector_pose1.p.data[0]=lb_end_effector_pose.p.data[0]+Amx*pi/2-Amx*pi+x;
		lb_end_effector_pose1.p.data[2]=lb_end_effector_pose.p.data[2]-0.04;
		int rc_lb = tracik_lb_solver.CartToJnt(resu_last_lb, lb_end_effector_pose1, result_lb);

		joint_state.header.stamp = ros::Time::now();
		    for(size_t i = 0; i < 6; i ++)
		    {
			joint_state.position[i] = result_rb(i);
		    }
		    for(size_t i = 6; i < 12; i ++)
		    {
			joint_state.position[i] = result_rm(i-6);
		    }
		    for(size_t i = 12; i < 18; i ++)
		    {
			joint_state.position[i] = result_rf(i-12);
		    }
		    for(size_t i = 18; i < 24; i ++)
		    {
			joint_state.position[i] = result_lf(i-18);
		    }
		    for(size_t i = 24; i < 30; i ++)
		    {
			joint_state.position[i] = result_lm(i-24);
		    }
		    for(size_t i = 30; i < 36; i ++)
		    {
			joint_state.position[i] = result_lb(i-30);
		    }
		std::string servo_cmd="";

		joint_state_pwm[0]=mid_c[0]+servo_direct[0]*int((2000.0/180.0)*joint_state.position[0]*180.0/pi);
		joint_state_pwm[1]=mid_c[1]+servo_direct[1]*int((2000.0/180.0)*joint_state.position[1]*180.0/pi);
		joint_state_pwm[2]=mid_c[2]+servo_direct[2]*int((2000.0/180.0)*joint_state.position[2]*180.0/pi);
		joint_state_pwm[3]=mid_c[3]+servo_direct[3]*int((2000.0/180.0)*joint_state.position[6]*180.0/pi);
		joint_state_pwm[4]=mid_c[4]+servo_direct[4]*int((2000.0/180.0)*joint_state.position[7]*180.0/pi);
		joint_state_pwm[5]=mid_c[5]+servo_direct[5]*int((2000.0/180.0)*joint_state.position[8]*180.0/pi);
		joint_state_pwm[6]=mid_c[6]+servo_direct[6]*int((2000.0/180.0)*joint_state.position[12]*180.0/pi);
		joint_state_pwm[7]=mid_c[7]+servo_direct[7]*int((2000.0/180.0)*joint_state.position[13]*180.0/pi);
		joint_state_pwm[8]=mid_c[8]+servo_direct[8]*int((2000.0/180.0)*joint_state.position[14]*180.0/pi);
		joint_state_pwm[9]=mid_c[9]+servo_direct[9]*int((2000.0/180.0)*joint_state.position[18]*180.0/pi);
		joint_state_pwm[10]=mid_c[10]+servo_direct[10]*int((2000.0/180.0)*joint_state.position[19]*180.0/pi);
		joint_state_pwm[11]=mid_c[11]+servo_direct[11]*int((2000.0/180.0)*joint_state.position[20]*180.0/pi);
		joint_state_pwm[12]=mid_c[12]+servo_direct[12]*int((2000.0/180.0)*joint_state.position[24]*180.0/pi);
		joint_state_pwm[13]=mid_c[13]+servo_direct[13]*int((2000.0/180.0)*joint_state.position[25]*180.0/pi);
		joint_state_pwm[14]=mid_c[14]+servo_direct[14]*int((2000.0/180.0)*joint_state.position[26]*180.0/pi);
		joint_state_pwm[15]=mid_c[15]+servo_direct[15]*int((2000.0/180.0)*joint_state.position[30]*180.0/pi);
		joint_state_pwm[16]=mid_c[16]+servo_direct[16]*int((2000.0/180.0)*joint_state.position[31]*180.0/pi);
		joint_state_pwm[17]=mid_c[17]+servo_direct[17]*int((2000.0/180.0)*joint_state.position[32]*180.0/pi);

		for(size_t i=0;i<18;i++)
		{
		    servo_cmd=servo_cmd+"#"+std::to_string(servo[i])+"P"+std::to_string(joint_state_pwm[i]);
		}
		servo_cmd=servo_cmd+"T100";

		uint8_t buffer[servo_cmd.length()+2];
		
		for(size_t i=0;i<servo_cmd.length();i++)
			buffer[i]=servo_cmd[i];
			buffer[servo_cmd.length()]=0x0D;
			buffer[servo_cmd.length()+1]=0x0A;

		sp.write(buffer,servo_cmd.length()+2);

	      	ROS_INFO("%s",buffer);  
		//ROS_INFO("%s",servo_cmd.c_str());	

		joint_pub.publish(joint_state);
	 
		resu_last_rb=result_rb;
		resu_last_rm=result_rm;
		resu_last_rf=result_rf;
		resu_last_lf=result_lf;
		resu_last_lm=result_lm;
		resu_last_lb=result_lb;
		r.sleep();
	    }
	    
	 
	    for(int i=1;i<=20;i++)
	    { 
		double t=pi*i/20;
		double x=Amx*t;
		double z=Amz*sin(t)+k;
		rb_end_effector_pose1.p.data[0]=rb_end_effector_pose.p.data[0]+Amx*pi/2-Amx*pi+x;
		rb_end_effector_pose1.p.data[2]=rb_end_effector_pose.p.data[2]-0.04;
		int rc_rb = tracik_rb_solver.CartToJnt(resu_last_rb, rb_end_effector_pose1, result_rb);

		rm_end_effector_pose1.p.data[0]=rm_end_effector_pose.p.data[0]+Amx*pi/2-x;
		rm_end_effector_pose1.p.data[2]=rm_end_effector_pose.p.data[2]+z-0.04;
		int rc_rm = tracik_rm_solver.CartToJnt(resu_last_rm, rm_end_effector_pose1, result_rm);

		rf_end_effector_pose1.p.data[0]=rf_end_effector_pose.p.data[0]+Amx*pi/2-Amx*pi+x;
		rf_end_effector_pose1.p.data[2]=rf_end_effector_pose.p.data[2]-0.04;
		int rc_rf = tracik_rf_solver.CartToJnt(resu_last_rf, rf_end_effector_pose1, result_rf);

		lf_end_effector_pose1.p.data[0]=lf_end_effector_pose.p.data[0]+Amx*pi/2-x;
		lf_end_effector_pose1.p.data[2]=lf_end_effector_pose.p.data[2]+z-0.04;
		int rc_lf = tracik_lf_solver.CartToJnt(resu_last_lf, lf_end_effector_pose1, result_lf);

		lm_end_effector_pose1.p.data[0]=lm_end_effector_pose.p.data[0]+Amx*pi/2-Amx*pi+x;
		lm_end_effector_pose1.p.data[2]=lm_end_effector_pose.p.data[2]-0.04;
		int rc_lm = tracik_lm_solver.CartToJnt(resu_last_lm, lm_end_effector_pose1, result_lm);

		lb_end_effector_pose1.p.data[0]=lb_end_effector_pose.p.data[0]+Amx*pi-x;
		lb_end_effector_pose1.p.data[2]=lb_end_effector_pose.p.data[2]+z-0.04;
		int rc_lb = tracik_lb_solver.CartToJnt(resu_last_lb, lb_end_effector_pose1, result_lb);
	 
		joint_state.header.stamp = ros::Time::now();
		    for(size_t i = 0; i < 6; i ++)
		    {
			joint_state.position[i] = result_rb(i);
		    }
		    for(size_t i = 6; i < 12; i ++)
		    {
			joint_state.position[i] = result_rm(i-6);
		    }
		    for(size_t i = 12; i < 18; i ++)
		    {
			joint_state.position[i] = result_rf(i-12);
		    }
		    for(size_t i = 18; i < 24; i ++)
		    {
			joint_state.position[i] = result_lf(i-18);
		    }
		    for(size_t i = 24; i < 30; i ++)
		    {
			joint_state.position[i] = result_lm(i-24);
		    }
		    for(size_t i = 30; i < 36; i ++)
		    {
			joint_state.position[i] = result_lb(i-30);
		    }
		
		std::string servo_cmd="";

		joint_state_pwm[0]=mid_c[0]+servo_direct[0]*int((2000.0/180.0)*joint_state.position[0]*180.0/pi);
		joint_state_pwm[1]=mid_c[1]+servo_direct[1]*int((2000.0/180.0)*joint_state.position[1]*180.0/pi);
		joint_state_pwm[2]=mid_c[2]+servo_direct[2]*int((2000.0/180.0)*joint_state.position[2]*180.0/pi);
		joint_state_pwm[3]=mid_c[3]+servo_direct[3]*int((2000.0/180.0)*joint_state.position[6]*180.0/pi);
		joint_state_pwm[4]=mid_c[4]+servo_direct[4]*int((2000.0/180.0)*joint_state.position[7]*180.0/pi);
		joint_state_pwm[5]=mid_c[5]+servo_direct[5]*int((2000.0/180.0)*joint_state.position[8]*180.0/pi);
		joint_state_pwm[6]=mid_c[6]+servo_direct[6]*int((2000.0/180.0)*joint_state.position[12]*180.0/pi);
		joint_state_pwm[7]=mid_c[7]+servo_direct[7]*int((2000.0/180.0)*joint_state.position[13]*180.0/pi);
		joint_state_pwm[8]=mid_c[8]+servo_direct[8]*int((2000.0/180.0)*joint_state.position[14]*180.0/pi);
		joint_state_pwm[9]=mid_c[9]+servo_direct[9]*int((2000.0/180.0)*joint_state.position[18]*180.0/pi);
		joint_state_pwm[10]=mid_c[10]+servo_direct[10]*int((2000.0/180.0)*joint_state.position[19]*180.0/pi);
		joint_state_pwm[11]=mid_c[11]+servo_direct[11]*int((2000.0/180.0)*joint_state.position[20]*180.0/pi);
		joint_state_pwm[12]=mid_c[12]+servo_direct[12]*int((2000.0/180.0)*joint_state.position[24]*180.0/pi);
		joint_state_pwm[13]=mid_c[13]+servo_direct[13]*int((2000.0/180.0)*joint_state.position[25]*180.0/pi);
		joint_state_pwm[14]=mid_c[14]+servo_direct[14]*int((2000.0/180.0)*joint_state.position[26]*180.0/pi);
		joint_state_pwm[15]=mid_c[15]+servo_direct[15]*int((2000.0/180.0)*joint_state.position[30]*180.0/pi);
		joint_state_pwm[16]=mid_c[16]+servo_direct[16]*int((2000.0/180.0)*joint_state.position[31]*180.0/pi);
		joint_state_pwm[17]=mid_c[17]+servo_direct[17]*int((2000.0/180.0)*joint_state.position[32]*180.0/pi);

		for(size_t i=0;i<18;i++)
		{
		    servo_cmd=servo_cmd+"#"+std::to_string(servo[i])+"P"+std::to_string(joint_state_pwm[i]);
		}
		servo_cmd=servo_cmd+"T100";

		uint8_t buffer[servo_cmd.length()+2];
		
		for(size_t i=0;i<servo_cmd.length();i++)
			buffer[i]=servo_cmd[i];
			buffer[servo_cmd.length()]=0x0D;
			buffer[servo_cmd.length()+1]=0x0A;
			
		
		sp.write(buffer,servo_cmd.length()+2);
		
	      	ROS_INFO("%s",buffer);  
		//ROS_INFO("%s",servo_cmd.c_str());

		joint_pub.publish(joint_state);
	 
		resu_last_rb=result_rb;
		resu_last_rm=result_rm;
		resu_last_rf=result_rf;
		resu_last_lf=result_lf;
		resu_last_lm=result_lm;
		resu_last_lb=result_lb;
		r.sleep();
	    }
    }
    else if(cmd=='c')
    {
	    for(int i=1;i<=20;i++)
	    { 
		double t=pi*i/20;
		double x=Amx*t;
		double z=Amz*sin(t)+k;

		rb_end_effector_pose1.p.data[0]=rb_end_effector_pose.p.data[0]-Amx*pi/2+x;
		rb_end_effector_pose1.p.data[2]=rb_end_effector_pose.p.data[2]+z-0.04;
		int rc_rb = tracik_rb_solver.CartToJnt(resu_last_rb, rb_end_effector_pose1, result_rb);

		rm_end_effector_pose1.p.data[0]=rm_end_effector_pose.p.data[0]-Amx*pi/2+Amx*pi-x;
		rm_end_effector_pose1.p.data[2]=rm_end_effector_pose.p.data[2]-0.04;
		int rc_rm = tracik_rm_solver.CartToJnt(resu_last_rm, rm_end_effector_pose1, result_rm);

		rf_end_effector_pose1.p.data[0]=rf_end_effector_pose.p.data[0]-Amx*pi/2+x;
		rf_end_effector_pose1.p.data[2]=rf_end_effector_pose.p.data[2]+z-0.04;
		int rc_rf = tracik_rf_solver.CartToJnt(resu_last_rf, rf_end_effector_pose1, result_rf);

		lf_end_effector_pose1.p.data[0]=lf_end_effector_pose.p.data[0]+Amx*pi/2-Amx*pi+x;
		lf_end_effector_pose1.p.data[2]=lf_end_effector_pose.p.data[2]-0.04;
		int rc_lf = tracik_lf_solver.CartToJnt(resu_last_lf, lf_end_effector_pose1, result_lf);

		lm_end_effector_pose1.p.data[0]=lm_end_effector_pose.p.data[0]+Amx*pi/2-x;
		lm_end_effector_pose1.p.data[2]=lm_end_effector_pose.p.data[2]+z-0.04;
		int rc_lm = tracik_lm_solver.CartToJnt(resu_last_lm, lm_end_effector_pose1, result_lm);

		lb_end_effector_pose1.p.data[0]=lb_end_effector_pose.p.data[0]+Amx*pi/2-Amx*pi+x;
		lb_end_effector_pose1.p.data[2]=lb_end_effector_pose.p.data[2]-0.04;
		int rc_lb = tracik_lb_solver.CartToJnt(resu_last_lb, lb_end_effector_pose1, result_lb);

		joint_state.header.stamp = ros::Time::now();
		    for(size_t i = 0; i < 6; i ++)
		    {
			joint_state.position[i] = result_rb(i);
		    }
		    for(size_t i = 6; i < 12; i ++)
		    {
			joint_state.position[i] = result_rm(i-6);
		    }
		    for(size_t i = 12; i < 18; i ++)
		    {
			joint_state.position[i] = result_rf(i-12);
		    }
		    for(size_t i = 18; i < 24; i ++)
		    {
			joint_state.position[i] = result_lf(i-18);
		    }
		    for(size_t i = 24; i < 30; i ++)
		    {
			joint_state.position[i] = result_lm(i-24);
		    }
		    for(size_t i = 30; i < 36; i ++)
		    {
			joint_state.position[i] = result_lb(i-30);
		    }
		std::string servo_cmd="";

		joint_state_pwm[0]=mid_c[0]+servo_direct[0]*int((2000.0/180.0)*joint_state.position[0]*180.0/pi);
		joint_state_pwm[1]=mid_c[1]+servo_direct[1]*int((2000.0/180.0)*joint_state.position[1]*180.0/pi);
		joint_state_pwm[2]=mid_c[2]+servo_direct[2]*int((2000.0/180.0)*joint_state.position[2]*180.0/pi);
		joint_state_pwm[3]=mid_c[3]+servo_direct[3]*int((2000.0/180.0)*joint_state.position[6]*180.0/pi);
		joint_state_pwm[4]=mid_c[4]+servo_direct[4]*int((2000.0/180.0)*joint_state.position[7]*180.0/pi);
		joint_state_pwm[5]=mid_c[5]+servo_direct[5]*int((2000.0/180.0)*joint_state.position[8]*180.0/pi);
		joint_state_pwm[6]=mid_c[6]+servo_direct[6]*int((2000.0/180.0)*joint_state.position[12]*180.0/pi);
		joint_state_pwm[7]=mid_c[7]+servo_direct[7]*int((2000.0/180.0)*joint_state.position[13]*180.0/pi);
		joint_state_pwm[8]=mid_c[8]+servo_direct[8]*int((2000.0/180.0)*joint_state.position[14]*180.0/pi);
		joint_state_pwm[9]=mid_c[9]+servo_direct[9]*int((2000.0/180.0)*joint_state.position[18]*180.0/pi);
		joint_state_pwm[10]=mid_c[10]+servo_direct[10]*int((2000.0/180.0)*joint_state.position[19]*180.0/pi);
		joint_state_pwm[11]=mid_c[11]+servo_direct[11]*int((2000.0/180.0)*joint_state.position[20]*180.0/pi);
		joint_state_pwm[12]=mid_c[12]+servo_direct[12]*int((2000.0/180.0)*joint_state.position[24]*180.0/pi);
		joint_state_pwm[13]=mid_c[13]+servo_direct[13]*int((2000.0/180.0)*joint_state.position[25]*180.0/pi);
		joint_state_pwm[14]=mid_c[14]+servo_direct[14]*int((2000.0/180.0)*joint_state.position[26]*180.0/pi);
		joint_state_pwm[15]=mid_c[15]+servo_direct[15]*int((2000.0/180.0)*joint_state.position[30]*180.0/pi);
		joint_state_pwm[16]=mid_c[16]+servo_direct[16]*int((2000.0/180.0)*joint_state.position[31]*180.0/pi);
		joint_state_pwm[17]=mid_c[17]+servo_direct[17]*int((2000.0/180.0)*joint_state.position[32]*180.0/pi);

		for(size_t i=0;i<18;i++)
		{
		    servo_cmd=servo_cmd+"#"+std::to_string(servo[i])+"P"+std::to_string(joint_state_pwm[i]);
		}
		servo_cmd=servo_cmd+"T100";

		uint8_t buffer[servo_cmd.length()+2];
		
		for(size_t i=0;i<servo_cmd.length();i++)
			buffer[i]=servo_cmd[i];
			buffer[servo_cmd.length()]=0x0D;
			buffer[servo_cmd.length()+1]=0x0A;

		sp.write(buffer,servo_cmd.length()+2);

	      	ROS_INFO("%s",buffer);  
		//ROS_INFO("%s",servo_cmd.c_str());	

		joint_pub.publish(joint_state);
	 
		resu_last_rb=result_rb;
		resu_last_rm=result_rm;
		resu_last_rf=result_rf;
		resu_last_lf=result_lf;
		resu_last_lm=result_lm;
		resu_last_lb=result_lb;
		r.sleep();
	    }
	    
	 
	    for(int i=1;i<=20;i++)
	    { 
		double t=pi*i/20;
		double x=Amx*t;
		double z=Amz*sin(t)+k;
		rb_end_effector_pose1.p.data[0]=rb_end_effector_pose.p.data[0]-Amx*pi/2+Amx*pi-x;
		rb_end_effector_pose1.p.data[2]=rb_end_effector_pose.p.data[2]-0.04;
		int rc_rb = tracik_rb_solver.CartToJnt(resu_last_rb, rb_end_effector_pose1, result_rb);

		rm_end_effector_pose1.p.data[0]=rm_end_effector_pose.p.data[0]-Amx*pi/2+x;
		rm_end_effector_pose1.p.data[2]=rm_end_effector_pose.p.data[2]+z-0.04;
		int rc_rm = tracik_rm_solver.CartToJnt(resu_last_rm, rm_end_effector_pose1, result_rm);

		rf_end_effector_pose1.p.data[0]=rf_end_effector_pose.p.data[0]-Amx*pi/2+Amx*pi-x;
		rf_end_effector_pose1.p.data[2]=rf_end_effector_pose.p.data[2]-0.04;
		int rc_rf = tracik_rf_solver.CartToJnt(resu_last_rf, rf_end_effector_pose1, result_rf);

		lf_end_effector_pose1.p.data[0]=lf_end_effector_pose.p.data[0]+Amx*pi/2-x;
		lf_end_effector_pose1.p.data[2]=lf_end_effector_pose.p.data[2]+z-0.04;
		int rc_lf = tracik_lf_solver.CartToJnt(resu_last_lf, lf_end_effector_pose1, result_lf);

		lm_end_effector_pose1.p.data[0]=lm_end_effector_pose.p.data[0]+Amx*pi/2-Amx*pi+x;
		lm_end_effector_pose1.p.data[2]=lm_end_effector_pose.p.data[2]-0.04;
		int rc_lm = tracik_lm_solver.CartToJnt(resu_last_lm, lm_end_effector_pose1, result_lm);

		lb_end_effector_pose1.p.data[0]=lb_end_effector_pose.p.data[0]+Amx*pi-x;
		lb_end_effector_pose1.p.data[2]=lb_end_effector_pose.p.data[2]+z-0.04;
		int rc_lb = tracik_lb_solver.CartToJnt(resu_last_lb, lb_end_effector_pose1, result_lb);
	 
		joint_state.header.stamp = ros::Time::now();
		    for(size_t i = 0; i < 6; i ++)
		    {
			joint_state.position[i] = result_rb(i);
		    }
		    for(size_t i = 6; i < 12; i ++)
		    {
			joint_state.position[i] = result_rm(i-6);
		    }
		    for(size_t i = 12; i < 18; i ++)
		    {
			joint_state.position[i] = result_rf(i-12);
		    }
		    for(size_t i = 18; i < 24; i ++)
		    {
			joint_state.position[i] = result_lf(i-18);
		    }
		    for(size_t i = 24; i < 30; i ++)
		    {
			joint_state.position[i] = result_lm(i-24);
		    }
		    for(size_t i = 30; i < 36; i ++)
		    {
			joint_state.position[i] = result_lb(i-30);
		    }
		
		std::string servo_cmd="";

		joint_state_pwm[0]=mid_c[0]+servo_direct[0]*int((2000.0/180.0)*joint_state.position[0]*180.0/pi);
		joint_state_pwm[1]=mid_c[1]+servo_direct[1]*int((2000.0/180.0)*joint_state.position[1]*180.0/pi);
		joint_state_pwm[2]=mid_c[2]+servo_direct[2]*int((2000.0/180.0)*joint_state.position[2]*180.0/pi);
		joint_state_pwm[3]=mid_c[3]+servo_direct[3]*int((2000.0/180.0)*joint_state.position[6]*180.0/pi);
		joint_state_pwm[4]=mid_c[4]+servo_direct[4]*int((2000.0/180.0)*joint_state.position[7]*180.0/pi);
		joint_state_pwm[5]=mid_c[5]+servo_direct[5]*int((2000.0/180.0)*joint_state.position[8]*180.0/pi);
		joint_state_pwm[6]=mid_c[6]+servo_direct[6]*int((2000.0/180.0)*joint_state.position[12]*180.0/pi);
		joint_state_pwm[7]=mid_c[7]+servo_direct[7]*int((2000.0/180.0)*joint_state.position[13]*180.0/pi);
		joint_state_pwm[8]=mid_c[8]+servo_direct[8]*int((2000.0/180.0)*joint_state.position[14]*180.0/pi);
		joint_state_pwm[9]=mid_c[9]+servo_direct[9]*int((2000.0/180.0)*joint_state.position[18]*180.0/pi);
		joint_state_pwm[10]=mid_c[10]+servo_direct[10]*int((2000.0/180.0)*joint_state.position[19]*180.0/pi);
		joint_state_pwm[11]=mid_c[11]+servo_direct[11]*int((2000.0/180.0)*joint_state.position[20]*180.0/pi);
		joint_state_pwm[12]=mid_c[12]+servo_direct[12]*int((2000.0/180.0)*joint_state.position[24]*180.0/pi);
		joint_state_pwm[13]=mid_c[13]+servo_direct[13]*int((2000.0/180.0)*joint_state.position[25]*180.0/pi);
		joint_state_pwm[14]=mid_c[14]+servo_direct[14]*int((2000.0/180.0)*joint_state.position[26]*180.0/pi);
		joint_state_pwm[15]=mid_c[15]+servo_direct[15]*int((2000.0/180.0)*joint_state.position[30]*180.0/pi);
		joint_state_pwm[16]=mid_c[16]+servo_direct[16]*int((2000.0/180.0)*joint_state.position[31]*180.0/pi);
		joint_state_pwm[17]=mid_c[17]+servo_direct[17]*int((2000.0/180.0)*joint_state.position[32]*180.0/pi);

		for(size_t i=0;i<18;i++)
		{
		    servo_cmd=servo_cmd+"#"+std::to_string(servo[i])+"P"+std::to_string(joint_state_pwm[i]);
		}
		servo_cmd=servo_cmd+"T100";

		uint8_t buffer[servo_cmd.length()+2];
		
		for(size_t i=0;i<servo_cmd.length();i++)
			buffer[i]=servo_cmd[i];
			buffer[servo_cmd.length()]=0x0D;
			buffer[servo_cmd.length()+1]=0x0A;
			
		
		sp.write(buffer,servo_cmd.length()+2);
		
	      	ROS_INFO("%s",buffer);  
		//ROS_INFO("%s",servo_cmd.c_str());

		joint_pub.publish(joint_state);
	 
		resu_last_rb=result_rb;
		resu_last_rm=result_rm;
		resu_last_rf=result_rf;
		resu_last_lf=result_lf;
		resu_last_lm=result_lm;
		resu_last_lb=result_lb;
		r.sleep();
	    }
    }
    else if(cmd=='d')
    {
	    for(int i=1;i<=20;i++)
	    { 
		double t=pi*i/20;
		double x=Amx*t;
		double z=Amz*sin(t)+k;

		rb_end_effector_pose1.p.data[0]=rb_end_effector_pose.p.data[0]+Amx*pi/2-x;
		rb_end_effector_pose1.p.data[2]=rb_end_effector_pose.p.data[2]+z-0.04;
		int rc_rb = tracik_rb_solver.CartToJnt(resu_last_rb, rb_end_effector_pose1, result_rb);

		rm_end_effector_pose1.p.data[0]=rm_end_effector_pose.p.data[0]+Amx*pi/2-Amx*pi+x;
		rm_end_effector_pose1.p.data[2]=rm_end_effector_pose.p.data[2]-0.04;
		int rc_rm = tracik_rm_solver.CartToJnt(resu_last_rm, rm_end_effector_pose1, result_rm);

		rf_end_effector_pose1.p.data[0]=rf_end_effector_pose.p.data[0]+Amx*pi/2-x;
		rf_end_effector_pose1.p.data[2]=rf_end_effector_pose.p.data[2]+z-0.04;
		int rc_rf = tracik_rf_solver.CartToJnt(resu_last_rf, rf_end_effector_pose1, result_rf);

		lf_end_effector_pose1.p.data[0]=lf_end_effector_pose.p.data[0]-Amx*pi/2+Amx*pi-x;
		lf_end_effector_pose1.p.data[2]=lf_end_effector_pose.p.data[2]-0.04;
		int rc_lf = tracik_lf_solver.CartToJnt(resu_last_lf, lf_end_effector_pose1, result_lf);

		lm_end_effector_pose1.p.data[0]=lm_end_effector_pose.p.data[0]-Amx*pi/2+x;
		lm_end_effector_pose1.p.data[2]=lm_end_effector_pose.p.data[2]+z-0.04;
		int rc_lm = tracik_lm_solver.CartToJnt(resu_last_lm, lm_end_effector_pose1, result_lm);

		lb_end_effector_pose1.p.data[0]=lb_end_effector_pose.p.data[0]-Amx*pi/2+Amx*pi-x;
		lb_end_effector_pose1.p.data[2]=lb_end_effector_pose.p.data[2]-0.04;
		int rc_lb = tracik_lb_solver.CartToJnt(resu_last_lb, lb_end_effector_pose1, result_lb);

		joint_state.header.stamp = ros::Time::now();
		    for(size_t i = 0; i < 6; i ++)
		    {
			joint_state.position[i] = result_rb(i);
		    }
		    for(size_t i = 6; i < 12; i ++)
		    {
			joint_state.position[i] = result_rm(i-6);
		    }
		    for(size_t i = 12; i < 18; i ++)
		    {
			joint_state.position[i] = result_rf(i-12);
		    }
		    for(size_t i = 18; i < 24; i ++)
		    {
			joint_state.position[i] = result_lf(i-18);
		    }
		    for(size_t i = 24; i < 30; i ++)
		    {
			joint_state.position[i] = result_lm(i-24);
		    }
		    for(size_t i = 30; i < 36; i ++)
		    {
			joint_state.position[i] = result_lb(i-30);
		    }
		std::string servo_cmd="";

		joint_state_pwm[0]=mid_c[0]+servo_direct[0]*int((2000.0/180.0)*joint_state.position[0]*180.0/pi);
		joint_state_pwm[1]=mid_c[1]+servo_direct[1]*int((2000.0/180.0)*joint_state.position[1]*180.0/pi);
		joint_state_pwm[2]=mid_c[2]+servo_direct[2]*int((2000.0/180.0)*joint_state.position[2]*180.0/pi);
		joint_state_pwm[3]=mid_c[3]+servo_direct[3]*int((2000.0/180.0)*joint_state.position[6]*180.0/pi);
		joint_state_pwm[4]=mid_c[4]+servo_direct[4]*int((2000.0/180.0)*joint_state.position[7]*180.0/pi);
		joint_state_pwm[5]=mid_c[5]+servo_direct[5]*int((2000.0/180.0)*joint_state.position[8]*180.0/pi);
		joint_state_pwm[6]=mid_c[6]+servo_direct[6]*int((2000.0/180.0)*joint_state.position[12]*180.0/pi);
		joint_state_pwm[7]=mid_c[7]+servo_direct[7]*int((2000.0/180.0)*joint_state.position[13]*180.0/pi);
		joint_state_pwm[8]=mid_c[8]+servo_direct[8]*int((2000.0/180.0)*joint_state.position[14]*180.0/pi);
		joint_state_pwm[9]=mid_c[9]+servo_direct[9]*int((2000.0/180.0)*joint_state.position[18]*180.0/pi);
		joint_state_pwm[10]=mid_c[10]+servo_direct[10]*int((2000.0/180.0)*joint_state.position[19]*180.0/pi);
		joint_state_pwm[11]=mid_c[11]+servo_direct[11]*int((2000.0/180.0)*joint_state.position[20]*180.0/pi);
		joint_state_pwm[12]=mid_c[12]+servo_direct[12]*int((2000.0/180.0)*joint_state.position[24]*180.0/pi);
		joint_state_pwm[13]=mid_c[13]+servo_direct[13]*int((2000.0/180.0)*joint_state.position[25]*180.0/pi);
		joint_state_pwm[14]=mid_c[14]+servo_direct[14]*int((2000.0/180.0)*joint_state.position[26]*180.0/pi);
		joint_state_pwm[15]=mid_c[15]+servo_direct[15]*int((2000.0/180.0)*joint_state.position[30]*180.0/pi);
		joint_state_pwm[16]=mid_c[16]+servo_direct[16]*int((2000.0/180.0)*joint_state.position[31]*180.0/pi);
		joint_state_pwm[17]=mid_c[17]+servo_direct[17]*int((2000.0/180.0)*joint_state.position[32]*180.0/pi);

		for(size_t i=0;i<18;i++)
		{
		    servo_cmd=servo_cmd+"#"+std::to_string(servo[i])+"P"+std::to_string(joint_state_pwm[i]);
		}
		servo_cmd=servo_cmd+"T100";

		uint8_t buffer[servo_cmd.length()+2];
		
		for(size_t i=0;i<servo_cmd.length();i++)
			buffer[i]=servo_cmd[i];
			buffer[servo_cmd.length()]=0x0D;
			buffer[servo_cmd.length()+1]=0x0A;

		sp.write(buffer,servo_cmd.length()+2);

	      	ROS_INFO("%s",buffer);  
		//ROS_INFO("%s",servo_cmd.c_str());	

		joint_pub.publish(joint_state);
	 
		resu_last_rb=result_rb;
		resu_last_rm=result_rm;
		resu_last_rf=result_rf;
		resu_last_lf=result_lf;
		resu_last_lm=result_lm;
		resu_last_lb=result_lb;
		r.sleep();
	    }
	    
	 
	    for(int i=1;i<=20;i++)
	    { 
		double t=pi*i/20;
		double x=Amx*t;
		double z=Amz*sin(t)+k;
		rb_end_effector_pose1.p.data[0]=rb_end_effector_pose.p.data[0]+Amx*pi/2-Amx*pi+x;
		rb_end_effector_pose1.p.data[2]=rb_end_effector_pose.p.data[2]-0.04;
		int rc_rb = tracik_rb_solver.CartToJnt(resu_last_rb, rb_end_effector_pose1, result_rb);

		rm_end_effector_pose1.p.data[0]=rm_end_effector_pose.p.data[0]+Amx*pi/2-x;
		rm_end_effector_pose1.p.data[2]=rm_end_effector_pose.p.data[2]+z-0.04;
		int rc_rm = tracik_rm_solver.CartToJnt(resu_last_rm, rm_end_effector_pose1, result_rm);

		rf_end_effector_pose1.p.data[0]=rf_end_effector_pose.p.data[0]+Amx*pi/2-Amx*pi+x;
		rf_end_effector_pose1.p.data[2]=rf_end_effector_pose.p.data[2]-0.04;
		int rc_rf = tracik_rf_solver.CartToJnt(resu_last_rf, rf_end_effector_pose1, result_rf);

		lf_end_effector_pose1.p.data[0]=lf_end_effector_pose.p.data[0]-Amx*pi/2+x;
		lf_end_effector_pose1.p.data[2]=lf_end_effector_pose.p.data[2]+z-0.04;
		int rc_lf = tracik_lf_solver.CartToJnt(resu_last_lf, lf_end_effector_pose1, result_lf);

		lm_end_effector_pose1.p.data[0]=lm_end_effector_pose.p.data[0]-Amx*pi/2+Amx*pi-x;
		lm_end_effector_pose1.p.data[2]=lm_end_effector_pose.p.data[2]-0.04;
		int rc_lm = tracik_lm_solver.CartToJnt(resu_last_lm, lm_end_effector_pose1, result_lm);

		lb_end_effector_pose1.p.data[0]=lb_end_effector_pose.p.data[0]-Amx*pi+x;
		lb_end_effector_pose1.p.data[2]=lb_end_effector_pose.p.data[2]+z-0.04;
		int rc_lb = tracik_lb_solver.CartToJnt(resu_last_lb, lb_end_effector_pose1, result_lb);
	 
		joint_state.header.stamp = ros::Time::now();
		    for(size_t i = 0; i < 6; i ++)
		    {
			joint_state.position[i] = result_rb(i);
		    }
		    for(size_t i = 6; i < 12; i ++)
		    {
			joint_state.position[i] = result_rm(i-6);
		    }
		    for(size_t i = 12; i < 18; i ++)
		    {
			joint_state.position[i] = result_rf(i-12);
		    }
		    for(size_t i = 18; i < 24; i ++)
		    {
			joint_state.position[i] = result_lf(i-18);
		    }
		    for(size_t i = 24; i < 30; i ++)
		    {
			joint_state.position[i] = result_lm(i-24);
		    }
		    for(size_t i = 30; i < 36; i ++)
		    {
			joint_state.position[i] = result_lb(i-30);
		    }
		
		std::string servo_cmd="";

		joint_state_pwm[0]=mid_c[0]+servo_direct[0]*int((2000.0/180.0)*joint_state.position[0]*180.0/pi);
		joint_state_pwm[1]=mid_c[1]+servo_direct[1]*int((2000.0/180.0)*joint_state.position[1]*180.0/pi);
		joint_state_pwm[2]=mid_c[2]+servo_direct[2]*int((2000.0/180.0)*joint_state.position[2]*180.0/pi);
		joint_state_pwm[3]=mid_c[3]+servo_direct[3]*int((2000.0/180.0)*joint_state.position[6]*180.0/pi);
		joint_state_pwm[4]=mid_c[4]+servo_direct[4]*int((2000.0/180.0)*joint_state.position[7]*180.0/pi);
		joint_state_pwm[5]=mid_c[5]+servo_direct[5]*int((2000.0/180.0)*joint_state.position[8]*180.0/pi);
		joint_state_pwm[6]=mid_c[6]+servo_direct[6]*int((2000.0/180.0)*joint_state.position[12]*180.0/pi);
		joint_state_pwm[7]=mid_c[7]+servo_direct[7]*int((2000.0/180.0)*joint_state.position[13]*180.0/pi);
		joint_state_pwm[8]=mid_c[8]+servo_direct[8]*int((2000.0/180.0)*joint_state.position[14]*180.0/pi);
		joint_state_pwm[9]=mid_c[9]+servo_direct[9]*int((2000.0/180.0)*joint_state.position[18]*180.0/pi);
		joint_state_pwm[10]=mid_c[10]+servo_direct[10]*int((2000.0/180.0)*joint_state.position[19]*180.0/pi);
		joint_state_pwm[11]=mid_c[11]+servo_direct[11]*int((2000.0/180.0)*joint_state.position[20]*180.0/pi);
		joint_state_pwm[12]=mid_c[12]+servo_direct[12]*int((2000.0/180.0)*joint_state.position[24]*180.0/pi);
		joint_state_pwm[13]=mid_c[13]+servo_direct[13]*int((2000.0/180.0)*joint_state.position[25]*180.0/pi);
		joint_state_pwm[14]=mid_c[14]+servo_direct[14]*int((2000.0/180.0)*joint_state.position[26]*180.0/pi);
		joint_state_pwm[15]=mid_c[15]+servo_direct[15]*int((2000.0/180.0)*joint_state.position[30]*180.0/pi);
		joint_state_pwm[16]=mid_c[16]+servo_direct[16]*int((2000.0/180.0)*joint_state.position[31]*180.0/pi);
		joint_state_pwm[17]=mid_c[17]+servo_direct[17]*int((2000.0/180.0)*joint_state.position[32]*180.0/pi);

		for(size_t i=0;i<18;i++)
		{
		    servo_cmd=servo_cmd+"#"+std::to_string(servo[i])+"P"+std::to_string(joint_state_pwm[i]);
		}
		servo_cmd=servo_cmd+"T100";

		uint8_t buffer[servo_cmd.length()+2];
		
		for(size_t i=0;i<servo_cmd.length();i++)
			buffer[i]=servo_cmd[i];
			buffer[servo_cmd.length()]=0x0D;
			buffer[servo_cmd.length()+1]=0x0A;
			
		
		sp.write(buffer,servo_cmd.length()+2);
		
	      	ROS_INFO("%s",buffer);  
		//ROS_INFO("%s",servo_cmd.c_str());

		joint_pub.publish(joint_state);
	 
		resu_last_rb=result_rb;
		resu_last_rm=result_rm;
		resu_last_rf=result_rf;
		resu_last_lf=result_lf;
		resu_last_lm=result_lm;
		resu_last_lb=result_lb;
		r.sleep();
	    }
    }
    else if(cmd=='e');

    }
    return 0;

}
