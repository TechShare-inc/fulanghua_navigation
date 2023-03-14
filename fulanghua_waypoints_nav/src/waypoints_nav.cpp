/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Daiki Maekawa and Chiba Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <actionlib/client/simple_action_client.h>
#include <fulanghua_action/special_moveAction.h>
#include <fulanghua_msg/_LimoStatus.h>
#include <fulanghua_srvs/_Pose.h>
#include <fulanghua_srvs/actions.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <yaml-cpp/yaml.h>

#include <exception>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "orne_waypoints_msgs/Pose.h"
#include "orne_waypoints_msgs/Waypoint.h"
#include "orne_waypoints_msgs/WaypointArray.h"
#include "std_msgs/Bool.h"

#ifdef NEW_YAMLCPP
template <typename T>
void operator>>(const YAML::Node &node, T &i) {
    i = node.as<T>();
}
#endif

class SwitchRunningStatus : public std::exception {
   public:
    SwitchRunningStatus() : std::exception() {}
};

class WaypointsNavigation {
   public:
    WaypointsNavigation()
        : has_activate_(false),
          move_base_action_("move_base", true),
          action_client("action", true),
          rotate_client("rotate", true),
          rate_(10),
          last_moved_time_(0),
          dist_err_(0.8),
          amcl_filename_("") {
        while ((move_base_action_.waitForServer(ros::Duration(1.0)) == false) &&
               (ros::ok() == true)) {
            ROS_INFO("Waiting...");
        }

        ros::NodeHandle private_nh("~");
        private_nh.param("robot_frame", robot_frame_, std::string("base_link"));
        private_nh.param("robot_name", robot_name_, std::string("limo"));
        private_nh.param("world_frame", world_frame_, std::string("map"));
        private_nh.param("cmd_vel", cmd_vel_, std::string("cmd_vel"));
        private_nh.param("amcl_filename", amcl_filename_, amcl_filename_);
        double max_update_rate;
        private_nh.param("max_update_rate", max_update_rate, 10.0);
        rate_ = ros::Rate(max_update_rate);
        std::string filename = "";
        private_nh.param("filename", filename, filename);
        if (filename != "") {
            ROS_INFO_STREAM("Read waypoints data from " << filename);
            if (!readFile(filename)) {
                ROS_ERROR("Failed loading waypoints file");
            } else {
                first_waypoint_ = waypoints_.poses.begin();
                last_waypoint_ = waypoints_.poses.end() - 2;
                finish_pose_ = waypoints_.poses.end() - 1;
                computeWpOrientation();
            }
            current_waypoint_ = waypoints_.poses.begin();
        } else {
            ROS_ERROR("waypoints file doesn't have name");
        }

        private_nh.param("dist_err", dist_err_, dist_err_);

        ros::NodeHandle nh;
        start_server_ = nh.advertiseService(
            "start_wp_nav", &WaypointsNavigation::startNavigationCallback,
            this);
        pause_server_ = nh.advertiseService(
            "pause_wp_nav", &WaypointsNavigation::pauseNavigationCallback,
            this);
        unpause_server_ = nh.advertiseService(
            "unpause_wp_nav", &WaypointsNavigation::unpauseNavigationCallback,
            this);
        stop_server_ = nh.advertiseService(
            "stop_wp_nav", &WaypointsNavigation::stopNavigationCallback, this);
        suspend_server_ = nh.advertiseService(
            "suspend_wp_pose", &WaypointsNavigation::suspendPoseCallback, this);
        resume_server_ = nh.advertiseService(
            "resume_wp_pose", &WaypointsNavigation::resumePoseCallback, this);
        search_server_ = nh.advertiseService(
            "near_wp_nav", &WaypointsNavigation::searchPoseCallback, this);
        cmd_vel_sub_ = nh.subscribe(cmd_vel_, 1,
                                    &WaypointsNavigation::cmdVelCallback, this);
        scan_sub_ = nh.subscribe("/scan", 100,
                                 &WaypointsNavigation::scanCallback, this);
        cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>(cmd_vel_, 100);
        robot_coordinate_pub =
            nh.advertise<geometry_msgs::Point>("robot_coordinate", 100);
        current_waypoint_pub = nh.advertise<orne_waypoints_msgs::Pose>(
            "next_waypoint", 100);  // publish next waypoint
        wp_pub_ =
            nh.advertise<orne_waypoints_msgs::WaypointArray>("waypoints", 10);
        clear_costmaps_srv_ =
            nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
        action_cmd_srv =
            nh.serviceClient<fulanghua_srvs::actions>("/action/start");
        // added below
        loop_start_server = nh.advertiseService(
            "loop_start_wp_nav", &WaypointsNavigation::loopStartCallback, this);
        loop_stop_server = nh.advertiseService(
            "loop_stop_wp_nav", &WaypointsNavigation::loopStopCallback, this);
        roundtrip_on_server_ = nh.advertiseService(
            "roundtrip_on_nav", &WaypointsNavigation::roundTripOnCallback,
            this);
        roundtrip_off_server_ = nh.advertiseService(
            "roundtrip_off_nav", &WaypointsNavigation::roundTripOffCallback,
            this);
        command_server = nh.advertiseService(
            "finish_action", &WaypointsNavigation::action_service_stop_callback,
            this);
    }

    bool roundTripOnCallback(std_srvs::Empty::Request &req,
                             std_srvs::Empty::Response &res) {
        ROS_INFO("roundtrip is on");
        REVERSE = true;
    }

    bool roundTripOffCallback(std_srvs::Empty::Request &req,
                              std_srvs::Empty::Response &res) {
        ROS_INFO("roundtrip is off");
        REVERSE = false;
    }

    bool loopStartCallback(std_srvs::Empty::Request &req,
                           std_srvs::Empty::Response &res) {
        ROS_INFO("loop is on");
        LOOP = true;
    }

    bool loopStopCallback(std_srvs::Empty::Request &req,
                          std_srvs::Empty::Response &res) {
        ROS_INFO("loop is off");
        LOOP = false;
    }

    const std::vector<orne_waypoints_msgs::Pose>::iterator makeQueue(
        const std::string &command) {
        orne_waypoints_msgs::WaypointArray dummy_array;
        orne_waypoints_msgs::Pose pose;
        pose.position.action = "guide";
        pose.position.file = command;
        pose.position.duration = INT_MAX;
        pose.position.x = 0;
        pose.position.y = 0;
        pose.position.z = 0;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 0;
        dummy_array.poses.push_back(pose);
        return dummy_array.poses.begin();
    }

    std::vector<orne_waypoints_msgs::Pose>::iterator makeQueue(
        const std::string &command, const int &duration) {
        orne_waypoints_msgs::WaypointArray dummy_array;
        orne_waypoints_msgs::Pose pose;
        pose.position.action = command;
        pose.position.duration = duration;
        pose.position.x = 0;
        pose.position.y = 0;
        pose.position.z = 0;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 0;
        dummy_array.poses.push_back(pose);
        return dummy_array.poses.begin();
    }

    void scanCallback(const sensor_msgs::LaserScan &msg) {
        auto ranges = msg.ranges;
        int obs_num = 0;
        for (int i = 0; i < ranges.size(); i++) {
            if (ranges[i] > 0.05 && ranges[i] < 0.18) {
                obs_num++;
            }
        }
        if (obs_num > 1) {
            stop++;
        } else {
            stop = 0;
        }
    }

    bool startNavigationCallback(std_srvs::Trigger::Request &request,
                                 std_srvs::Trigger::Response &response) {
        if (has_activate_) {
            response.success = false;
            return false;
        }
        ros::NodeHandle private_nh("~");
        private_nh.param("robot_frame", robot_frame_, std::string("base_link"));
        private_nh.param("world_frame", world_frame_, std::string("map"));
        private_nh.param("cmd_vel", cmd_vel_, std::string("cmd_vel"));
        std::string filename = "";
        private_nh.param("filename", filename, filename);
        if (filename != "") {
            ROS_INFO_STREAM("Read waypoints data from " << filename);
            if (!readFile(filename)) {
                ROS_ERROR("Failed loading waypoints file");
            } else {
                first_waypoint_ = waypoints_.poses.begin();
                last_waypoint_ = waypoints_.poses.end() - 2;
                finish_pose_ = waypoints_.poses.end() - 1;
                computeWpOrientation();
            }
            current_waypoint_ = waypoints_.poses.begin();
        } else {
            ROS_ERROR("waypoints file doesn't have name");
        }

        std_srvs::Empty empty;
        while (!clear_costmaps_srv_.call(empty)) {
            ROS_WARN("Resend clear costmap service");
            sleep();
        }

        current_waypoint_ = waypoints_.poses.begin();
        ROS_WARN("Start!");
        has_activate_ = true;
        response.success = true;
        return true;
    }

    bool pauseNavigationCallback(std_srvs::Empty::Request &request,
                                 std_srvs::Empty::Response &response) {
        if (!has_activate_) {
            ROS_WARN("Navigation is already pause");
            return false;
        }
        ROS_WARN("Navigation has been paused");
        has_activate_ = false;
        return true;
    }

    bool unpauseNavigationCallback(std_srvs::Trigger::Request &request,
                                   std_srvs::Trigger::Response &response) {
        if (has_activate_) {
            ROS_WARN("Navigation is already active");
            response.success = false;
        }

        has_activate_ = true;
        response.success = true;
        return true;
    }

    bool stopNavigationCallback(std_srvs::Empty::Request &request,
                                std_srvs::Empty::Response &response) {
        ROS_WARN("Navigation has been stopped");
        has_activate_ = false;
        move_base_action_.cancelAllGoals();
        return true;
    }

    bool resumePoseCallback(std_srvs::Empty::Request &request,
                            std_srvs::Empty::Request &response) {
        if (has_activate_) {
            return false;
        }
        tf::StampedTransform robot_gl = getRobotPosGL();
        std_srvs::Empty empty;
        clear_costmaps_srv_.call(empty);
        // move_base_action_.cancelAllGoals();

        ///< @todo calculating metric with request orientation
        double min_dist = std::numeric_limits<double>::max();
        for (std::vector<orne_waypoints_msgs::Pose>::iterator it =
                 current_waypoint_;
             it != finish_pose_; it++) {
            double dist = hypot(it->position.x - robot_gl.getOrigin().x(),
                                it->position.y - robot_gl.getOrigin().y());
            if (dist < min_dist) {
                min_dist = dist;
                current_waypoint_ = it;
                ROS_WARN("Navigation has been resumed");
            }
        }

        has_activate_ = true;

        return true;
    }

    bool suspendPoseCallback(fulanghua_srvs::_Pose::Request &request,
                             fulanghua_srvs::_Pose::Response &response) {
        if (!has_activate_) {
            response.status = false;
            return false;
        }

        // move_base_action_.cancelAllGoals();
        startNavigationGL(request.pose);
        bool isNavigationFinished = false;
        while (!isNavigationFinished && ros::ok()) {
            actionlib::SimpleClientGoalState state =
                move_base_action_.getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                isNavigationFinished = true;
                response.status = true;
            } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
                isNavigationFinished = true;
                response.status = false;
            }
            sleep();
        }
        has_activate_ = false;

        return true;
    }

    bool searchPoseCallback(std_srvs::Trigger::Request &request,
                            std_srvs::Trigger::Response &response) {
        if (has_activate_) {
            response.success = false;
            return false;
        }

        tf::StampedTransform robot_gl = getRobotPosGL();
        std_srvs::Empty empty;
        clear_costmaps_srv_.call(empty);

        double min_dist = std::numeric_limits<double>::max();
        for (std::vector<orne_waypoints_msgs::Pose>::iterator it =
                 current_waypoint_;
             it != finish_pose_; it++) {
            double dist = hypot(it->position.x - robot_gl.getOrigin().x(),
                                it->position.y - robot_gl.getOrigin().y());
            if (dist < min_dist) {
                min_dist = dist;
                current_waypoint_ = it;
            }
        }

        response.success = true;
        has_activate_ = true;

        return true;
    }
    void callRotateClient() {
        if (rotate_client.isServerConnected()) {
            bool state = true;

            fulanghua_action::special_moveGoal current_goal;
            current_goal.duration = 20;
            current_goal.angle = -90;
            for (int i = 0; i < 5; i++) {
                rotate_client.sendGoal(current_goal);
                actionlib::SimpleClientGoalState client_state =
                    rotate_client.getState();
                while (client_state !=
                       actionlib::SimpleClientGoalState::SUCCEEDED) {
                    client_state = rotate_client.getState();
                    if (client_state ==
                            actionlib::SimpleClientGoalState::PREEMPTED ||
                        client_state ==
                            actionlib::SimpleClientGoalState::ABORTED) {
                        ROS_WARN("failed %d times\n", i + 1);
                        state = false;
                        break;
                    }
                    ros::Duration(0.1).sleep();
                }
                if (state) {
                    //    rotate_client.cancelAllGoals();
                    break;
                }
            }
            ros::Duration(1).sleep();
        }
    }


    void cmdVelCallback(const geometry_msgs::Twist &msg) {
        if (msg.linear.x > -0.001 && msg.linear.x < 0.001 &&
            msg.linear.y > -0.001 && msg.linear.y < 0.001 &&
            msg.linear.z > -0.001 && msg.linear.z < 0.001 &&
            msg.angular.x > -0.001 && msg.angular.x < 0.001 &&
            msg.angular.y > -0.001 && msg.angular.y < 0.001 &&
            msg.angular.z > -0.001 && msg.angular.z < 0.001) {
            ROS_INFO("command velocity all zero");
        } else {
            last_moved_time_ = ros::Time::now().toSec();
        }
    }

    bool action_service_stop_callback(std_srvs::Empty::Request &req,
                                      std_srvs::Empty::Response &re) {
        ROS_INFO("Finshing action");
        action_client.cancelGoal();
        // re.success = true;
    }

    bool readFile(const std::string &filename) {
        waypoints_.poses.clear();
        try {
            std::ifstream ifs(filename.c_str(), std::ifstream::in);
            if (ifs.good() == false) {
                return false;
            }

            YAML::Node node;

#ifdef NEW_YAMLCPP
            node = YAML::Load(ifs);
#else
            YAML::Parser parser(ifs);
            parser.GetNextDocument(node);
#endif

#ifdef NEW_YAMLCPP
            const YAML::Node &wp_node_tmp = node["waypoints"];
            const YAML::Node *wp_node = wp_node_tmp ? &wp_node_tmp : NULL;
#else
            const YAML::Node *wp_node = node.FindValue("waypoints");
#endif

            orne_waypoints_msgs::Pose pose;

            if (wp_node != NULL) {
                for (int i = 0; i < wp_node->size(); i++) {
                    (*wp_node)[i]["point"]["pose"]["x"] >> pose.position.x;
                    (*wp_node)[i]["point"]["pose"]["y"] >> pose.position.y;
                    (*wp_node)[i]["point"]["pose"]["z"] >> pose.position.z;
                    (*wp_node)[i]["point"]["action"]["a"] >>
                        pose.position.action;
                    (*wp_node)[i]["point"]["action"]["d"] >>
                        pose.position.duration;
                    (*wp_node)[i]["point"]["action"]["f"] >> pose.position.file;
                    (*wp_node)[i]["point"]["orientation"]["x"] >>
                        pose.orientation.x;
                    (*wp_node)[i]["point"]["orientation"]["y"] >>
                        pose.orientation.y;
                    (*wp_node)[i]["point"]["orientation"]["z"] >>
                        pose.orientation.z;
                    (*wp_node)[i]["point"]["orientation"]["w"] >>
                        pose.orientation.w;
                    waypoints_.poses.push_back(pose);
                }
            } else {
                return false;
            }

#ifdef NEW_YAMLCPP
            const YAML::Node &fp_node_tmp = node["finish_pose"];
            const YAML::Node *fp_node = fp_node_tmp ? &fp_node_tmp : NULL;
#else
            const YAML::Node *fp_node = node.FindValue("finish_pose");
#endif

            if (fp_node != NULL) {
                (*fp_node)["pose"]["position"]["x"] >> pose.position.x;
                (*fp_node)["pose"]["position"]["y"] >> pose.position.y;
                (*fp_node)["pose"]["position"]["z"] >> pose.position.z;

                (*fp_node)["pose"]["orientation"]["x"] >> pose.orientation.x;
                (*fp_node)["pose"]["orientation"]["y"] >> pose.orientation.y;
                (*fp_node)["pose"]["orientation"]["z"] >> pose.orientation.z;
                (*fp_node)["pose"]["orientation"]["w"] >> pose.orientation.w;

                waypoints_.poses.push_back(pose);

            } else {
                return false;
            }

        } catch (YAML::ParserException &e) {
            return false;

        } catch (YAML::RepresentationException &e) {
            return false;
        }

        return true;
    }

    void computeWpOrientation() {
        for (std::vector<orne_waypoints_msgs::Pose>::iterator it =
                 waypoints_.poses.begin();
             it != finish_pose_; it++) {
            if (it->position.action == "passthrough") {
                double goal_direction =
                    atan2((it + 1)->position.y - (it)->position.y,
                          (it + 1)->position.x - (it)->position.x);
                (it)->orientation =
                    tf::createQuaternionMsgFromYaw(goal_direction);
            }
        }
        waypoints_.header.frame_id = world_frame_;
    }
    
    void computeWpOrientationReverse() {
        for (std::vector<orne_waypoints_msgs::Pose>::iterator it = finish_pose_;
             it != waypoints_.poses.begin(); it--) {
            if (it->position.action == "passthrough") {
                double goal_direction =
                    atan2((it - 1)->position.y - (it)->position.y,
                          (it - 1)->position.x - (it)->position.x);
                (it)->orientation =
                    tf::createQuaternionMsgFromYaw(goal_direction);
            }
        }
        waypoints_.header.frame_id = world_frame_;
    }

    bool shouldSendGoal() {
        bool ret = true;
        actionlib::SimpleClientGoalState state = move_base_action_.getState();
        if ((state != actionlib::SimpleClientGoalState::ACTIVE) &&
            (state != actionlib::SimpleClientGoalState::PENDING) &&
            (state != actionlib::SimpleClientGoalState::RECALLED) &&
            (state != actionlib::SimpleClientGoalState::PREEMPTED)) {
            ret = false;
        }

        if (waypoints_.poses.empty()) {
            ret = false;
        }

        return ret;
    }

    bool navigationFinished() {
        return move_base_action_.getState() ==
               actionlib::SimpleClientGoalState::SUCCEEDED;
    }

    bool onNavigationPoint(const orne_waypoints_msgs::Waypoint &dest,
                           double dist_err = 0.8) {
        tf::StampedTransform robot_gl = getRobotPosGL();

        const double wx = dest.x;
        const double wy = dest.y;
        const double rx = robot_gl.getOrigin().x();
        const double ry = robot_gl.getOrigin().y();
        const double dist =
            std::sqrt(std::pow(wx - rx, 2) + std::pow(wy - ry, 2));

        return dist < dist_err;
    }

    tf::StampedTransform getRobotPosGL() {
        tf::StampedTransform robot_gl;
        geometry_msgs::Point pt;
        try {
            tf_listener_.lookupTransform(world_frame_, robot_frame_,
                                         ros::Time(0.0), robot_gl);
            pt.x = robot_gl.getOrigin().x();
            pt.y = robot_gl.getOrigin().y();
            location_update(robot_gl);
            robot_coordinate_pub.publish(pt);
        } catch (tf::TransformException &e) {
            ROS_WARN_STREAM("tf::TransformException: " << e.what());
        }

        return robot_gl;
    }

    void sleep() {
        rate_.sleep();
        ros::spinOnce();
        publishPoseArray();
        current_waypoint_pub.publish(
            *current_waypoint_);  // publish next waypoint to central server
        if (stop == 1) {
            ros::Time t_stop = ros::Time::now();
            while ((ros::Time::now() - t_stop).toSec() < 3) {
                geometry_msgs::Twist twist;
                twist.linear.x = 0;
                twist.angular.z = 0;
                cmd_vel_pub_.publish(twist);
                ROS_INFO("stopped to avoid collision for %lf seconds",
                         (ros::Time::now() - t_stop).toSec());
            }
        }
    }

    void actionServiceCall(
        const std::vector<orne_waypoints_msgs::Pose>::iterator &dest) {
        bool initial_goal = false;
        for (int i = 0; i < 1; i++) {
            bool state = true;
            if (action_client.isServerConnected()) {
                fulanghua_action::special_moveGoal goal;
                // goal.task_id = task_id;
                ROS_WARN("goal setting");
                goal.command = dest->position.action;
                goal.wp.position = dest->position;
                goal.wp.orientation = dest->orientation;
                goal.duration = INT_MAX;
                goal.file = dest->position.file;
                std::cout << "publish command:" << goal.command << std::endl;
                action_client.sendGoal(goal);
                actionlib::SimpleClientGoalState client_state =
                    action_client.getState();
                while (client_state !=
                       actionlib::SimpleClientGoalState::SUCCEEDED) {
                    getRobotPosGL();
                    client_state = action_client.getState();
                    if (client_state ==
                            actionlib::SimpleClientGoalState::PREEMPTED ||
                        client_state ==
                            actionlib::SimpleClientGoalState::ABORTED) {
                        ROS_WARN("failed %d times\n", i + 1);
                        state = false;
                        break;
                    }
                    ros::Duration(0.1).sleep();
                }
            } else {
                break;
            }

            if (state) {
                has_activate_ = true;
                break;
            } else if (i == 0) {
                ROS_WARN(
                    "Failed the whole process 5 times so please call the "
                    "operator to fix this");
                stopNavigationCallback(req, res);
                ros::Duration(10).sleep();
                has_activate_ = false;
                break;
            } else {
                ROS_WARN("You Failed %d times\n ", i + 1);
                continue;
            }
            rate_.sleep();
        }
        if ((dest->position.x != 0 && dest->position.y != 0) &&
            (dest->orientation.x != 0 && dest->orientation.y != 0)) {
            ROS_INFO("call next action");
            actionServiceCall(makeQueue("next"));
            printf("Action finished\n");
        }
    }

    void location_update(const tf::StampedTransform &robot_gl) {
        std::ofstream ofs(amcl_filename_.c_str(), std::ios::out);
        tf::Quaternion q(robot_gl.getRotation().x(), robot_gl.getRotation().y(),
                         robot_gl.getRotation().z(),
                         robot_gl.getRotation().w());
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        ofs << "initial_pose_x: " << robot_gl.getOrigin().x() << std::endl;
        ofs << "initial_pose_y: " << robot_gl.getOrigin().y() << std::endl;
        ofs << "initial_pose_a: " << yaw << std::endl;
        // printf("yaw: %f\n", yaw);
        ofs.close();
    }
    void startNavigationGL(const orne_waypoints_msgs::Pose &dest) {
        move_base_msgs::MoveBaseGoal move_base_goal;
        move_base_goal.target_pose.header.stamp = ros::Time::now();
        move_base_goal.target_pose.header.frame_id = world_frame_;
        move_base_goal.target_pose.pose.position.x = dest.position.x;
        move_base_goal.target_pose.pose.position.y = dest.position.y;
        move_base_goal.target_pose.pose.position.z = dest.position.z;
        move_base_goal.target_pose.pose.orientation = dest.orientation;

        move_base_action_.sendGoal(move_base_goal);
    }

    bool actionConfirm(const orne_waypoints_msgs::Pose &dest) {
        if (dest.position.action == "passthrough") return false;
        return true;
    }
    void publishPoseArray() {
        waypoints_.header.stamp = ros::Time::now();
        wp_pub_.publish(waypoints_);
    }

    void run() {
        while (ros::ok()) {
            getRobotPosGL();
            try {
                if (has_activate_) {
                    // send guide_{robot name} action to action server when
                    // started
                    if (_initial) {
                        std::string rname = "guide_" + robot_name_;
                        actionServiceCall(makeQueue(rname));
                        _initial = false;
                    }

                    // ROS_INFOs
                    if (current_waypoint_ == last_waypoint_ && !REVERSE) {
                        ROS_INFO("prepare finish pose");
                    } else if (current_waypoint_ == first_waypoint_ + 1 &&
                               REVERSE && _reached) {
                        ROS_INFO("prepare finish pose");
                    } else {
                        ROS_INFO("calculate waypoint direction");
                        ROS_INFO_STREAM("goal_direction = "
                                        << current_waypoint_->orientation);
                        if (REVERSE && _reached) {
                            ROS_INFO_STREAM(
                                "current_waypoint_-1 "
                                << (current_waypoint_ - 1)->position.y);
                        } else {
                            ROS_INFO_STREAM(
                                "current_waypoint_+1 "
                                << (current_waypoint_ + 1)->position.y);
                        }
                        ROS_INFO_STREAM("current_waypoint_"
                                        << current_waypoint_->position.y);
                    }
                    startNavigationGL(
                        *current_waypoint_);  // Go to the waypoint
                    int resend_goal = 0;
                    double start_nav_time = ros::Time::now().toSec();
                    while (!onNavigationPoint(current_waypoint_->position,
                                              dist_err_)) {
                        if (!has_activate_) throw SwitchRunningStatus();

                        double time = ros::Time::now().toSec();
                        if (time - start_nav_time > 5.0 &&
                            time - last_moved_time_ > 5.0) {
                            ROS_WARN("Resend the navigation goal.");
                            std_srvs::Empty empty;
                            clear_costmaps_srv_.call(empty);
                            startNavigationGL(*current_waypoint_);
                            resend_goal++;
                            if (resend_goal == 3) {
                                ROS_WARN("Skip waypoint.");
                                actionServiceCall(makeQueue("skip"));
                                if (REVERSE && _reached)
                                    current_waypoint_--;
                                else
                                    current_waypoint_++;
                                startNavigationGL(*current_waypoint_);
                            }
                            start_nav_time = time;
                        }
                        sleep();
                    }
                    // do the action here
                    // call the function that calls service with action code
                    orne_waypoints_msgs::Pose temp_wp;
                    if (actionConfirm(*current_waypoint_)) {
                        while (!navigationFinished() && ros::ok()) sleep();
                        has_activate_ = false;
                        actionServiceCall(current_waypoint_);
                    }

                    if (_reached && REVERSE) {
                        current_waypoint_--;
                    } else {
                        current_waypoint_++;
                    }
                    if (has_activate_) {
                        if (current_waypoint_ == finish_pose_ && !REVERSE) {
                            has_activate_ = false;
                            if (LOOP) {
                                ROS_INFO_STREAM("LOOP start!");
                                actionServiceCall(makeQueue("loop"));
                                has_activate_ = true;
                                current_waypoint_ = waypoints_.poses.begin();
                                _reached = true;
                            } else {
                                actionServiceCall(makeQueue("finish"));
                            }
                        } else if (current_waypoint_ == finish_pose_ &&
                                   REVERSE) {
                            ROS_INFO_STREAM("REVERSE start!");
                            actionServiceCall(makeQueue("reverse"));
                            computeWpOrientationReverse();
                            current_waypoint_--;
                            _reached = true;
                        }
                        if (_reached && LOOP &&
                            current_waypoint_ == first_waypoint_) {
                            has_activate_ = true;
                            computeWpOrientation();
                            startNavigationGL(*current_waypoint_);
                            while (!navigationFinished() && ros::ok()) sleep();
                            current_waypoint_++;
                            _reached = false;
                        } else if (_reached && !LOOP &&
                                   current_waypoint_ == first_waypoint_) {
                            startNavigationGL(*current_waypoint_);
                            while (!navigationFinished() && ros::ok()) sleep();
                            has_activate_ = false;
                        }
                    }
                }
            } catch (const SwitchRunningStatus &e) {
                ROS_INFO_STREAM("running status switched");
            }

            sleep();
        }
    }

   private:
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
        move_base_action_;
    actionlib::SimpleActionClient<fulanghua_action::special_moveAction>
        action_client;
    actionlib::SimpleActionClient<fulanghua_action::special_moveAction>
        rotate_client;
    std::string amcl_filename_;
    orne_waypoints_msgs::WaypointArray waypoints_;
    visualization_msgs::MarkerArray marker_;
    std::vector<orne_waypoints_msgs::Pose>::iterator current_waypoint_;
    std::vector<orne_waypoints_msgs::Pose>::iterator last_waypoint_;
    std::vector<orne_waypoints_msgs::Pose>::iterator first_waypoint_;
    std::vector<orne_waypoints_msgs::Pose>::iterator finish_pose_;
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response res;
    bool has_activate_;
    std::string robot_frame_, world_frame_, cmd_vel_, robot_name_;
    // MG400
    tf::TransformListener tf_listener_;
    ros::Rate rate_;
    ros::ServiceServer start_server_, pause_server_, unpause_server_,
        stop_server_, suspend_server_, resume_server_, search_server_,
        loop_start_server, loop_stop_server, roundtrip_on_server_,
        roundtrip_off_server_, command_server;
    ros::Subscriber cmd_vel_sub_, scan_sub_;
    ros::Publisher wp_pub_, cmd_vel_pub_, robot_coordinate_pub,
        current_waypoint_pub;
    ros::ServiceClient clear_costmaps_srv_, action_cmd_srv;
    double last_moved_time_, dist_err_;
    int failed_counter = 0;
    bool LOOP = false;
    bool REVERSE = false;
    bool action_finished = false;
    bool _reached = false;
    bool _initial = true;
    int stop = 0;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, ROS_PACKAGE_NAME);
    WaypointsNavigation w_nav;
    w_nav.run();

    return 0;
}
