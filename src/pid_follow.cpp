#include "ros/ros.h"
#include "string.h"
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>

#include <rrt_exploration_tutorials/Command.h>
#define PI 3.1415926

geometry_msgs::Transform robot1_state;
geometry_msgs::Transform robot2_state;
geometry_msgs::Transform robot3_state;
geometry_msgs::Twist vel_command;
nav_msgs::Path plan_path_sample;
bool robot2_command = true;
bool robot3_command = true;
bool direction_follow = false; // the command which allow the slave's path is following the master's postion orientation.
bool path_plan_updated = false;
unsigned int path_plan_length = 0;
unsigned int nearest_point = 0;
bool robot1_arrived_nearest = 0;
bool robot2_arrived_nearest = 0;
float delet_PID_I;

bool exist_master_path_plan = false;

rrt_exploration_tutorials::Command command; //the command for control the robot2 or robot3

float delta_dt;
float last_time = 0;
float safe_distance = 0.1;
float robot2_yaw_last = 0.0f;
float rates_last = 0.0f;
float vel_last = 0.0f;


/*******The function  list *********/
float get_dt(ros::Time last);
geometry_msgs::Twist  calculate_vel(float delta_dt,geometry_msgs::Transform state);
geometry_msgs::Vector3 get_euler_from_quaternion(tf::Quaternion q);
float sign(float delta);
float constrain(float delta,float min,float max);
unsigned int check_nearest_point(geometry_msgs::Transform state,bool force = false);
bool check_robot_arrived_nearest_point(geometry_msgs::Transform state,unsigned int index);
geometry_msgs::Twist robot_movements(float delta_dt,geometry_msgs::Transform state);
geometry_msgs::Twist go_forward(float delta_dt,geometry_msgs::Transform state,const nav_msgs::Path goal,int index,bool force_follow = false);


void tf_Cb(const tf2_msgs::TFMessage::ConstPtr& msg) {
    if (msg->transforms[0].header.frame_id == "robot_1/odom") {
        robot1_state.translation.x = msg->transforms[0].transform.translation.x;
        robot1_state.translation.y = msg->transforms[0].transform.translation.y;
        robot1_state.rotation = msg->transforms[0].transform.rotation;
    }
    if (msg->transforms[0].header.frame_id == "robot_2/odom") {
        robot2_state.translation.x = msg->transforms[0].transform.translation.x;
        robot2_state.translation.y = msg->transforms[0].transform.translation.y - 0.8;
        robot2_state.rotation = msg->transforms[0].transform.rotation;
        ROS_INFO("robot2_state.translation.x:%f ------ robot2_state.translation.y :%f",robot2_state.translation.x,robot2_state.translation.y);

    }

    if (msg->transforms[0].header.frame_id == "robot_3/odom") {
        robot3_state.translation.x = msg->transforms[0].transform.translation.x;
        robot3_state.translation.y = msg->transforms[0].transform.translation.y + 0.8;
        robot3_state.rotation = msg->transforms[0].transform.rotation;
    }
}


void command_Cb(const rrt_exploration_tutorials::Command::ConstPtr& msg){
    command.robot_id = msg->robot_id;
    command.start = msg->start;
}

void plan_Cb(const nav_msgs::Path::ConstPtr& msg){
    if (msg->header.frame_id == "robot_1/map") {
        path_plan_updated = true;
        plan_path_sample.header = msg->header;
        plan_path_sample.poses.resize(msg->poses.size());
        for (unsigned int i = 0; i < msg->poses.size(); i++) {
            plan_path_sample.poses[i] = msg->poses[i];
            exist_master_path_plan = true;
        }
        path_plan_length = msg->poses.size();
        robot2_arrived_nearest = false;
    }

}


int main(int argc, char** argv) {
    ros::init(argc,argv,"pid_follow");

    ros::NodeHandle nh;
    ros::Subscriber tf_sub = nh.subscribe<tf2_msgs::TFMessage>("/tf",10,tf_Cb);

    ros::Subscriber command_sub = nh.subscribe<rrt_exploration_tutorials::Command>("command", 1, command_Cb);

    ros::Subscriber plan_sub = nh.subscribe<nav_msgs::Path>("/robot_1/move_base_node/NavfnROS/plan", 1, plan_Cb);


//    ros::Subscriber tf_sub2 = nh.subscribe<tf2_msgs::TFMessage>("/tf",10,tf_cb2);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/robot_2/mobile_base/commands/velocity",10);

    ros::Publisher vel_pub_3 = nh.advertise<geometry_msgs::Twist>("/robot_3/mobile_base/commands/velocity",10);


    ros::Rate loop(10);

    ros::Time begin_time = ros::Time::now();


    while(ros::ok()) {

        float curr_time = get_dt(begin_time);
        delta_dt = curr_time - last_time;

        // the command which allow the slave's path is following the master's postion orientation.
        if (direction_follow) {
            // calculate the robot2's vel
            geometry_msgs::Twist vel_target = calculate_vel(delta_dt,robot2_state);

            // start robot2's movements
            if (command.robot_id == 2) {
                if (command.start) {
                    robot2_command = true;
                } else {
                    robot2_command = false;
                }
            }

            if (robot2_command) {
                vel_pub.publish(vel_target);
            }

            // calculate the robot3's vel
            vel_target = calculate_vel(delta_dt,robot3_state);

            if (command.robot_id == 3) {
                if (command.start) {
                    robot3_command = true;
                } else {
                    robot3_command = false;
                }
            }

            if (robot3_command) {
                vel_pub_3.publish(vel_target);
            }
            ROS_INFO("~~~~~~~ robot2_command:%d  robot3_command: %d ~~~~~~~~~~~",robot2_command,robot3_command);
        }

        {
        // Three step:
        // 1. Get to the nearest path plan routes
        // 2. wait or follow the master's path
        }

        if (path_plan_updated) {
            nearest_point = check_nearest_point(robot2_state);
            ROS_INFO("nearest_point .............%d ...........",nearest_point);
        }
        geometry_msgs::Twist vel_target2 = robot_movements(delta_dt,robot2_state);
        vel_pub.publish(vel_target2);


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
            vel.x = (pos_err- safe_distance)/delta_dt;
            vel.y = 0.0f;
            vel.z = 0.0f;

            float vel_max = 0.3;

            float vel_length = sqrt(pow(vel.x,2)+ pow(vel.y,2));
            vel_max = fabsf(angle_err) < 0.05? vel_max: 0.001f * vel_max;
            if (vel_length > vel_max) {
                 vel.x = vel_max;
//                 vel.y = vel.y * vel_max/vel_length;
            };

            ROS_INFO("angle_err:%f  x_err :%f y_err :%f  rates:%f",angle_err*57.3,x_err,y_err,vel_command.angular.z*57.3);

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

unsigned int check_nearest_point(geometry_msgs::Transform state, bool force) {
    if (path_plan_updated || force) {
        path_plan_updated = false;
        unsigned int nearest_index = 0;
        float nearest_distance = 0;
        path_plan_length = plan_path_sample.poses.size();
        for (unsigned int i = 0; i < path_plan_length; i++) {
            float x_err = (plan_path_sample.poses[i].pose.position.x - state.translation.x);
            float y_err = (plan_path_sample.poses[i].pose.position.y - state.translation.y);

            float pos_err = sqrt(pow(x_err,2) + pow(y_err,2));

            // record the nearest point
            if (pos_err < nearest_distance || i == 0) {
                nearest_distance = pos_err;
                nearest_index = i;
            }
        }
        return nearest_index;
    }
}

bool check_robot_arrived_nearest_point(geometry_msgs::Transform state,unsigned int index)
{
    float x_err = plan_path_sample.poses[index].pose.position.x - state.translation.x;
    float y_err = plan_path_sample.poses[index].pose.position.y - state.translation.y;

    float pos_err = sqrt(pow(x_err,2) + pow(y_err,2));
    if (pos_err < safe_distance) {
        return true;
    } else {
        return false;
    }

}

// calculate the robots' movements
geometry_msgs::Twist robot_movements(float delta_dt,geometry_msgs::Transform state)
{

     geometry_msgs::Twist vel_target;

     // without the matser robot's path plan, exit imediately.
     if (!exist_master_path_plan) {
         vel_target.linear.x = 0;
         vel_target.linear.y = 0;
         vel_target.angular.z = 0;
         return vel_target;
     }

    if (!robot1_arrived_nearest) {
        robot1_arrived_nearest = check_robot_arrived_nearest_point(robot1_state,nearest_point);
    }

    if (!robot2_arrived_nearest) {
        robot2_arrived_nearest = check_robot_arrived_nearest_point(state,nearest_point);
    }

    ROS_INFO("robot1_arrived_nearest : %d.......... robot2_arrived_nearest: %d",robot1_arrived_nearest,robot2_arrived_nearest);

    if (robot2_arrived_nearest && robot1_arrived_nearest) {
       // robot2 go forward

        int index = check_nearest_point(state,true);
        ROS_INFO("check_nearest_point:%d ............................",index);

        if (index+10 < path_plan_length-1) {
            vel_target = go_forward(delta_dt,state,plan_path_sample,index+10,true);
        }

    } else if (robot2_arrived_nearest && !robot1_arrived_nearest) {
       // wait for robot1' arrival
       vel_target.linear.x = 0;
       vel_target.linear.y = 0;
       vel_target.angular.z = 0;

    } else if (!robot2_arrived_nearest) {
       // go to the nearest point
//       vel_target = go_forward(delta_dt,state,plan_path_sample,nearest_point);
        vel_target = go_forward(delta_dt,state,plan_path_sample,nearest_point);

    }

    return vel_target;
}

geometry_msgs::Twist go_forward(float delta_dt,geometry_msgs::Transform state,const nav_msgs::Path goal,int index,bool force_follow)
{
    geometry_msgs::Twist vel;
    if (delta_dt > 0) {
        float x_err = (goal.poses[index].pose.position.x - state.translation.x);
        float y_err = (goal.poses[index].pose.position.y - state.translation.y);

        // the angle between current position to the goal
        float current_2_goal = atan2(y_err,x_err);
        tf::Quaternion quat_robot(state.rotation.x,state.rotation.y,state.rotation.z,state.rotation.w);
        geometry_msgs::Vector3 robot_direction = get_euler_from_quaternion(quat_robot);

        float angle_err = (current_2_goal - robot_direction.z);
        if (angle_err > PI) {
            angle_err -= PI;
        } else if (angle_err < -PI){
            angle_err += PI;
        }
        // calculate the desired rates
        delet_PID_I += 0.1 * angle_err * delta_dt * 0.2;
        vel_command.angular.z = constrain(1.25 * (0.1 * angle_err / delta_dt),-2.0,2.0) + delet_PID_I;
//        vel_command.angular.z = 0.1 * vel_command.angular.z + 0.9  * rates_last;


        float pos_err = sqrt(pow(x_err,2) + pow(y_err,2));

        ROS_INFO("angle_err:%f  x_err :%f y_err :%f  rates:%f",angle_err*57.3,x_err,y_err,vel_command.angular.z*57.3);


        geometry_msgs::Vector3 vel;

        if (pos_err > safe_distance || force_follow){
            float vel_max = 0.45;

            vel.x = force_follow ? vel_max * 0.9 : (pos_err/delta_dt);
            vel.y = 0.0f;
            vel.z = 0.0f;

            float vel_length = sqrt(pow(vel.x,2)+ pow(vel.y,2));
//            vel_max = fabsf(angle_err) < (force_follow? 0.15 :0.05)? vel_max: 0.001f * vel_max;
            vel_max = fabsf(angle_err) < 0.07? vel_max: 0.001f * vel_max;
            if (vel_length > vel_max) {
                // the robot just can go forward
                 vel.x = vel_max;
            };
            vel.x = 0.8 * vel_last + vel.x * 0.2;

        } else {
            vel.x =0;
            vel.y =0;
            vel.z = 0;
            vel_command.angular.z = 0.0f;
        }

        vel_command.linear = vel;
        vel_last = vel.x;
        rates_last = vel_command.angular.z;
    }

    return vel_command;

}

