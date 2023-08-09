#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <fulanghua_action/special_moveAction.h>
#include <actionlib/server/simple_action_server.h>
#include "cs_signal/WpRobotStatus.h"
#include "orne_waypoints_msgs/Pose.h"

typedef actionlib::SimpleActionServer<fulanghua_action::special_moveAction> Server;

bool start = false;
double wait_time = 0;
int status_mg400_01 = 0;
int status_mg400_02 = 0;
std::string which_mg = "";
ros::Time start_time;
int align_fail = 0;

void mg400_01Callback(const cs_signal::WpRobotStatus& msg){
    int new_status = msg.robotStatus;
    if(which_mg == "mg400_01"){
        ROS_INFO("%s : %d, wait time : %0.1lf",which_mg.c_str(), new_status, wait_time);
        if (wait_time > 45){
            if((status_mg400_01 > 1) && new_status == 1){
                start = true;
                start_time = ros::Time::now();
            }
            if (new_status == 0 || status_mg400_01 == 0){
                align_fail++;
                ROS_INFO("align fail : %d", align_fail);
            }
            if (align_fail > 10){
                start = true;
                align_fail = 0;
            }
        }
    }
    status_mg400_01 = new_status;
}

void mg400_02Callback(const cs_signal::WpRobotStatus& msg){
    int new_status = msg.robotStatus;
    if(which_mg == "mg400_02"){
        ROS_INFO("%s : %d, wait time : %0.1lf",which_mg.c_str(), new_status, wait_time);
        if (wait_time > 45){
            if((status_mg400_02 > 1) && new_status == 1){
                start = true;
                start_time = ros::Time::now();
            }
            if (new_status == 0 || status_mg400_02 == 0){
                align_fail++;
                ROS_INFO("align fail : %d", align_fail);
            }
            if (align_fail > 10){
                start = true;
                align_fail = 0;
            }
        }
    }
    status_mg400_02 = new_status;
}

void nextWpCallback(const orne_waypoints_msgs::Pose &msg){
    std::string wp = msg.position.action;
    if (wp =="align1" && status_mg400_01 < 2){
        which_mg = "mg400_01";
    }else if (wp =="align2" && status_mg400_02 < 2){
        which_mg = "mg400_02";
    }else{
        which_mg = "";
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "wait_server");

    ros::NodeHandle nh;
    Server server(nh, "wait", false);
    ros::Subscriber mg400_01_sub = nh.subscribe("/mg400_01/wp_robot_status", 10000, mg400_01Callback);
    ros::Subscriber mg400_02_sub = nh.subscribe("/mg400_02/wp_robot_status", 10000, mg400_02Callback);
    ros::Subscriber next_wp_sub = nh.subscribe("/next_waypoint", 10000, nextWpCallback);

    ros::Rate rate(10);
    fulanghua_action::special_moveGoalConstPtr current_goal;
    ros::Time wait_start;

    server.start();
    while (ros::ok()){
        if (server.isNewGoalAvailable()){
            current_goal = server.acceptNewGoal(); 
            start_time = ros::Time::now();
        }
        if(server.isActive()){
            wait_time = (ros::Time::now() - start_time).toSec();
            if(server.isPreemptRequested()){
                server.setPreempted(); // cancel the goal
                ROS_WARN("wait: Preemmpt Goal\n");
            }else{
                if (start){
                    server.setSucceeded();
                    start = false;
                }else if (wait_time < 45){
                    ROS_INFO("wait 45 seconds : %0.1lf", wait_time);
                }else{
                    ROS_INFO("wait for mg400");
                    ros::Duration(1).sleep();
                }
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}