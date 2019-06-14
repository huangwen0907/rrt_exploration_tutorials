#include "ros/ros.h"
#include "string.h"
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>


geometry_msgs::Transform robot1_state;
geometry_msgs::Transform robot2_state;
geometry_msgs::Transform robot3_state;
geometry_msgs::Twist vel_command;

float delta_dt;
float last_time = 0;
float safe_distance = 0.2;
float robot2_yaw_last = 0.0f;
float rates_last = 0.0f;


/*******The function  list *********/
float get_dt(ros::Time last);
geometry_msgs::Twist  calculate_vel(float delta_dt,geometry_msgs::Transform state);
geometry_msgs::Vector3 get_euler_from_quaternion(tf::Quaternion q);
float sign(float delta);
float constrain(float delta,float min,float max);


void tf_Cb(const tf2_msgs::TFMessage::ConstPtr& msg) {
    if (msg->transforms[0].header.frame_id == "robot_1/odom") {
        robot1_state.translation.x = msg->transforms[0].transform.translation.x;
        robot1_state.translation.y = msg->transforms[0].transform.translation.y;
        robot1_state.rotation = msg->transforms[0].transform.rotation;
    }
    if (msg->transforms[0].header.frame_id == "robot_2/odom") {
        robot2_state.translation.x = msg->transforms[0].transform.translation.x;
        robot2_state.translation.y = msg->transforms[0].transform.translation.y;
        robot2_state.rotation = msg->transforms[0].transform.rotation;
    }

    if (msg->transforms[0].header.frame_id == "robot_3/odom") {
        robot3_state.translation.x = msg->transforms[0].transform.translation.x;
        robot3_state.translation.y = msg->transforms[0].transform.translation.y;
        robot3_state.rotation = msg->transforms[0].transform.rotation;
    }
}


//void tf_cb2(const tf2_msgs::TFMessage::ConstPtr& msg){
//    if (msg->transforms[0].header.frame_id == "robot_2/odom") {
//        robot2_state.translation.x = msg->transforms[0].transform.translation.x;
//        robot2_state.translation.y = msg->transforms[0].transform.translation.y;
//        robot2_state.rotation = msg->transforms[0].transform.rotation;
//    }
//}


int main(int argc, char** argv) {
    ros::init(argc,argv,"pid_follow");

    ros::NodeHandle nh;
    ros::Subscriber tf_sub = nh.subscribe<tf2_msgs::TFMessage>("/tf",10,tf_Cb);

//    ros::Subscriber tf_sub2 = nh.subscribe<tf2_msgs::TFMessage>("/tf",10,tf_cb2);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/robot_2/mobile_base/commands/velocity",10);

    ros::Publisher vel_pub_3 = nh.advertise<geometry_msgs::Twist>("/robot_3/mobile_base/commands/velocity",10);





    ros::Rate loop(10);

    ros::Time begin_time = ros::Time::now();

    while(ros::ok()) {

        float curr_time = get_dt(begin_time);
        delta_dt = curr_time - last_time;

        // calculate the robot2's vel
        geometry_msgs::Twist vel_target = calculate_vel(delta_dt,robot2_state);
        vel_pub.publish(vel_target);

        // calculate the robot3's vel
        vel_target = calculate_vel(delta_dt,robot3_state);
        vel_pub_3.publish(vel_target);



        last_time = curr_time;
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}


float get_dt(ros::Time last)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - last.sec;
    float currTimenSec = time_now.nsec*1e-9 -last.nsec*1e-9;
    return (currTimeSec + currTimenSec);
}

geometry_msgs::Twist calculate_vel(float delta_dt,geometry_msgs::Transform state)
{
    ROS_INFO("DT:%f ............................",delta_dt);
    if (delta_dt > 0){
        float x_err = (robot1_state.translation.x - state.translation.x);
        float y_err = (robot1_state.translation.y - state.translation.y);
        // the angle between robot2 and robot1
        float robot2_2_robot1_direction = atan2(y_err,x_err);

        // Robot2's Attitude
        tf::Quaternion quat_robot2(state.rotation.x,state.rotation.y,state.rotation.z,state.rotation.w);
        geometry_msgs::Vector3 robot2_direction = get_euler_from_quaternion(quat_robot2);

        // 'robot2_2_robot1_direction' target direction
        // 'robot2_direction.z' current direction
        float angle_err =  robot2_2_robot1_direction - robot2_direction.z;
        // calculate the desired rates
        vel_command.angular.z = constrain(0.1 * angle_err / delta_dt,-2.0,2.0);

        float pos_err = sqrt(pow(x_err,2) + pow(y_err,2));
        geometry_msgs::Vector3 vel;

        if (pos_err > safe_distance){
            vel.x = pos_err/delta_dt;
            vel.y = 0.0f;
            vel.z = 0.0f;

            float vel_max = 0.3;

            float vel_length = sqrt(pow(vel.x,2)+ pow(vel.y,2));
            vel_max = fabsf(angle_err) < 0.05? vel_max: 0.001f * vel_max;
            if (vel_length > vel_max) {
                 vel.x = vel_max;
//                 vel.y = vel.y * vel_max/vel_length;
            };

            ROS_INFO("angle_err:%f  x_err :%f y_err :%f  rates:%f", angle_err*57.3,x_err,y_err,vel_command.angular.z*57.3);

            robot2_yaw_last = robot2_2_robot1_direction;
            rates_last = vel_command.angular.z;

        } else {
            vel.x =0;
            vel.y =0;
            vel.z = 0;
            vel_command.angular.z = 0.0f;
        }

         vel_command.linear = vel;
    }
    return vel_command;

}


// get the euler from the quaternion
geometry_msgs::Vector3 get_euler_from_quaternion(tf::Quaternion q){
    geometry_msgs::Vector3 euler;

    float q1 = q.w();
    float q2 = q.x();
    float q3 = q.y();
    float q4 = q.z();

    euler.x = atan2(2*q3*q4 + 2*q1*q2,q1*q1-q2*q2-q3*q3+q4*q4);
    euler.y = -asin(2*q2*q4-2*q1*q3);
    euler.z =  atan2(2*q2*q3+2*q1*q4,q1*q1+q2*q2-q3*q3-q4*q4);
    return euler;
}

float sign(float delta) {
    if (delta > 0) {
        return 1;
    } else if (delta < 0) {
        return -1;
    } else {
        return 0;
    }
}

float constrain(float delta,float min,float max) {
    if (delta > max) {
         return max;
    } else if (delta < min) {
        return min;
    } else {
        return delta;
    }
}

