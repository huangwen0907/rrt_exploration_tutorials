#include "ros/ros.h"
#include "string.h"
#include <ros/console.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/String.h"

#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Header.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::TransformStamped laser;
geometry_msgs::Pose pose;
geometry_msgs::PoseStamped path[4];



void tfCb(const tf2_msgs::TFMessage::ConstPtr &msg) {
    laser = msg->transforms[0];
    pose.position.x = laser.transform.translation.x;
    pose.position.y = laser.transform.translation.y;
}

int main(int argc, char ** argv)
{
        ros::init(argc,argv,"path_design");

        ros::NodeHandle nh;
        ros::Subscriber tf_sub = nh.subscribe<tf2_msgs::TFMessage>("/tf",10,tfCb);
        ros::Publisher path_pub = nh.advertise<geometry_msgs::PoseStamped>("goal",10);

        ros::Rate loop_rate(1);
        path[0].pose.position.x = 4.0;
        path[0].pose.position.y = 4.0;

        path[1].pose.position.x = 4.0;
        path[1].pose.position.y = -4.0;

        path[2].pose.position.x = -4.0;
        path[2].pose.position.y = -4.0;

        path[3].pose.position.x = -4.0;
        path[3].pose.position.y = 4.0;
        int count_pub = 0;
        int num = 0;
        float deltax = 0.0f;
        float deltay = 0.0f;
        float delat_d = 0.0f;
        while(ros::ok()) {

            if (count_pub >= 1) {
                deltax = pose.position.x - path[count_pub - 1].pose.position.x;
                deltay = pose.position.y - path[count_pub - 1].pose.position.y;
                delat_d =  sqrtf(deltax * deltax + deltay * deltay);
            }
            ROS_INFO("DIS :%f",delat_d);
            if ((fabsf(delat_d) < 0.5 || count_pub == 0) && ros::Time::now().sec > 100)
            {

                path[count_pub].header.frame_id = "robot_1/map";
                path[count_pub].pose.orientation.w = 1.0;

                path[count_pub].header.stamp = ros::Time::now();
                    ROS_INFO("Tagrget[%d] x:%f y%f",count_pub,path[count_pub].pose.position.x,path[count_pub].pose.position.y);

                    path_pub.publish(path[count_pub]);


		num ++;

                if (count_pub >4 && num > 5) {
                    count_pub = 0;
		    num = 0;
                }

		if (num > 5) {
			count_pub ++;
		}

            }


            ros::spinOnce();
            loop_rate.sleep();
        }

     return 0;
}
