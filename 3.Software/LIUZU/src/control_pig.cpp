#include "ros/ros.h"
#include "std_msgs/Float64.h" 
#include <sstream>

int main(int argc, char  *argv[])
{   
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"control_pig");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float64>("/pig_dog/joint1_position_controller/command",10);
    std_msgs::Float64 control_pig;
    control_pig.data=0.0;
    bool flag=0;//0weizeng 1weijian
    ros::Rate r(10);
    while (ros::ok())
    {
        
        if(control_pig.data==3.0)
        {
          flag=1;
        }
        if(control_pig.data==0.0)
        {
          flag=0;
        }
        if(flag==0)
        control_pig.data=control_pig.data+0.10;
        else if(flag==1)
        control_pig.data=control_pig.data-0.10;

        pub.publish(control_pig);

        ROS_INFO("发送的消息:%f",control_pig);
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}
