#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <camera_action/camera_pkgAction.h>
#include <fulanghua_action/special_moveAction.h>
#include <fulanghua_msg/_LimoStatus.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sound_play/SoundRequest.h>
#include <sound_play/SoundRequestAction.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "orne_waypoints_msgs/Pose.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
typedef actionlib::SimpleActionServer<fulanghua_action::special_moveAction>
    Server;

class SpecialMove {
   public:
    SpecialMove()
        : server(nh, "action", false),
          sound_client("sound_play", true),
          rotate_client("rotate", true),
          ar_detect_client("ar_detect", true),
          go1_cmd_client("go1_command", true),
          ar_align_client("ar_align", true),
          charging_station_client("charging_station", true),
          rate_(2) {
        ros::NodeHandle private_nh("~");
        private_nh.param("cmd_vel_posture", cmd_vel_posture,
                         std::string("cmd_vel_posture"));
        private_nh.param("cmd_vel", cmd_vel_, std::string("cmd_vel"));
        private_nh.param("robot_name", robot_name_, std::string("go1"));
        private_nh.param("holonomic", holonomic_, true);
        private_nh.param("charge_threshold_higher", charge_threshold_higher_,
                         12.0);
        private_nh.param("max_vel", max_vel, 0.4);
        private_nh.param("min_vel", min_vel, 0.1);
        private_nh.param("dist_err", dist_err, 0.8);
        private_nh.param("voice_path", _voice_path, std::string(""));
        private_nh.param("volume", voice_volume, 1.0);

        twist_postgure_pub =
            nh.advertise<geometry_msgs::Twist>(cmd_vel_posture, 100);
        twist_move_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_, 100);
        charge_reset_srv =
            nh.serviceClient<std_srvs::Empty>(ARUCO_DETECT_SERVICE_RESET);
        robot_coordinate_sub = nh.subscribe(
            "robot_coordinate", 100, &SpecialMove::coordinate_callback, this);
        odom_sub = nh.subscribe("odom", 100, &SpecialMove::odom_callback, this);
        server.start();
    }

    void odom_callback(const nav_msgs::Odometry& odom) { _odom = odom; }

    void coordinate_callback(const geometry_msgs::Point& point) {
        rx = point.x;
        ry = point.y;
    }

    void speaking_function(const std::string& sound_file_name) {
        speak_start = true;
        if (sound_client.isServerConnected()) {
            sound_play::SoundRequestGoal goal;
            sound_play::SoundRequest sr;
            goal.sound_request.sound = sr.PLAY_FILE;
            goal.sound_request.command = sr.PLAY_ONCE;
            goal.sound_request.volume = voice_volume;
            goal.sound_request.arg = voice_path + sound_file_name + ".wav";
            std::cout << "sound file name:" << sound_file_name;
            sound_client.sendGoal(goal);
            sound_client.waitForResult();
            ros::Duration(1).sleep();
            if (posture) {
                // twist.linear.x = 0;
                // twist.angular.z = 0;
                double start_send_time = ros::Time::now().toSec();
                double end_send_time = ros::Time::now().toSec();
                while ((end_send_time - start_send_time) < 0.5) {
                    twist_postgure_pub.publish(twist);
                    end_send_time = ros::Time::now().toSec();
                    rate_.sleep();
                }
            }
            speak_start = false;
            printf("Voice Action finished\n");
            server.setSucceeded();
            ROS_INFO("Succeeded it");
        }
    }

    void speaking_callback(const std_msgs::Bool& msg) { speaking = msg.data; }

    void chargingFunction() {
        charging = true;
        bool state = true;
        int counter_ = 0;
        printf("charging action here\n");
        // ar marker detection to approach the charging station

        if (ar_detect_client.isServerConnected() && state) {
            fulanghua_action::special_moveGoal current_goal;
            current_goal.duration = 120;
            for (int i = 0; i < 5; i++) {
                state = true;
                ar_detect_client.sendGoal(current_goal);
                actionlib::SimpleClientGoalState client_state =
                    ar_detect_client.getState();
                while (client_state !=
                       actionlib::SimpleClientGoalState::SUCCEEDED) {
                    client_state = ar_detect_client.getState();
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
                    // ar_detect_client.cancelAllGoals();
                    break;
                }
            }

            ros::Duration(1).sleep();
        }
        // rotate the robot so that realsense can see it
        if (!holonomic_) {
            if (rotate_client.isServerConnected() && state) {
                fulanghua_action::special_moveGoal current_goal;
                current_goal.duration = 20;
                current_goal.angle = -90;
                for (int i = 0; i < 5; i++) {
                    state = true;
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
                        // rotate_client.cancelAllGoals();
                        break;
                    }
                }
                ros::Duration(1).sleep();
            }
        }

        if (robot_name_ == "go1") {
            if (go1_cmd_client.isServerConnected() && state) {
                fulanghua_action::special_moveGoal current_goal;
                current_goal.duration = 2;
                for (int i = 0; i < 5; i++) {
                    state = true;
                    go1_cmd_client.sendGoal(current_goal);
                    actionlib::SimpleClientGoalState client_state =
                        go1_cmd_client.getState();
                    while (client_state !=
                           actionlib::SimpleClientGoalState::SUCCEEDED) {
                        client_state = go1_cmd_client.getState();
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
                        // rotate_client.cancelAllGoals();
                        break;
                    }
                }
                ros::Duration(1).sleep();
            }
        }

        // chaging station action server here
        if (charging_station_client.isServerConnected() && state) {
            camera_action::camera_pkgGoal current_goal;
            current_goal.duration = 120;

            for (int i = 0; i < 5; i++) {
                state = true;
                charging_station_client.sendGoal(current_goal);
                actionlib::SimpleClientGoalState client_state =
                    charging_station_client.getState();
                while (client_state !=
                       actionlib::SimpleClientGoalState::SUCCEEDED) {
                    client_state = charging_station_client.getState();
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
                    // charging_station_client.cancelAllGoals();
                    break;
                }
            }
            ros::Duration(1).sleep();
        }

        // check everything is fine.
        if (!state) {
            charge_reset_srv.call(req, res);
            ros::Duration(5).sleep();
            charging = false;
            server.setPreempted();
            ROS_INFO("A whole charging process is preempted");
        } else {
            charging = false;
            server.setSucceeded();
            ROS_INFO("A whole charging process is Done");
        }
    }

    void P2P_move(const orne_waypoints_msgs::Pose& dest) {
        if (!onNavigationPoint(dest)) {
            twist_move_pub.publish(twist);
        } else {
            initial = true;
            t = 0;
            server.setSucceeded();
            ROS_INFO("Succeeded it\n");
        }
    }

    void AlignmentFunction() {
        bool state = true;
        if (ar_align_client.isServerConnected() && state) {
            fulanghua_action::special_moveGoal current_goal;
            current_goal.duration = INT_MAX -1;
            ROS_INFO("Alignment started");
            for (int i = 0; i < 3; i++) {
                state = true;
                ar_align_client.sendGoal(current_goal);
                actionlib::SimpleClientGoalState client_state =
                    ar_align_client.getState();
                while (client_state !=
                       actionlib::SimpleClientGoalState::SUCCEEDED) {
                    client_state = ar_align_client.getState();
                    if (client_state ==
                            actionlib::SimpleClientGoalState::ABORTED) {
                        ROS_WARN("aborted %d times\n", i + 1);
                        state = true;
                        break;
                    }
                     if (client_state ==
                            actionlib::SimpleClientGoalState::PREEMPTED) {
                        ROS_WARN("preemped %d times\n", i + 1);
                        state = false;
                        break;
                    }
                    ros::Duration(0.1).sleep();
                }
                if (state) {
                    // ar_align_client.cancelAllGoals();
                    break;
                }
            }
            if (!state) {
                ros::Duration(3).sleep();
                server.setPreempted();
                ROS_INFO("An alignment process is preempted");
            } else {
                ros::Duration(3).sleep();
                server.setSucceeded();
                ROS_INFO("An alignment process is Done");
            }
        }
    }

    bool onNavigationPoint(const orne_waypoints_msgs::Pose& dest) {
        const double wx = dest.position.x;
        const double wy = dest.position.y;
        const double dist =
            std::sqrt(std::pow(wx - rx, 2) + std::pow(wy - ry, 2));
        // get the angle the target from the current position
        double angle = std::atan2((wy - ry), (wx - rx));
        if (initial) {
            initial_odom = _odom;
            // steering = direction.orientation - angle;
            initial = false;
        }
        double odom_diff = (initial_odom.pose.pose.orientation.z -
                            _odom.pose.pose.orientation.z);
        // rn I only consider the x coordinate for determing the velocity
        double temp = 0;
        double diff = std::abs(dist - prev_location);
        if (t != 0) {
            // PD
            if (diff == 0) {
                diff = prev_diff;
            } else {
                prev_diff = diff;
            }
            velocity_x = Kp * std::abs(dist);  // - Kv * diff/interval;
            // P
            //  velocity_x = Kp* std::abs(dist);
            temp = velocity_x;
            velocity_x = std::min(max_vel, velocity_x);
            velocity_x = std::max(min_vel, velocity_x);

        } else {
            velocity_x = 0.4;
        }
        printf("dx: %f\n", (dist - prev_location));
        prev_location = dist;
        // twist.linear.x = velocity_x;
        twist.linear.x = velocity_x;
        twist.angular.z =
            3.33232 * -odom_diff + 0.32467;  // Calculated by linear regression
        printf("cmd_vel_x = %f\n", velocity_x);
        printf("calculated velocity %f\n", temp);
        printf("dist = %f\n", dist);
        t++;
        return dist < dist_err;
    }

    bool isPostureAvailable(
        const fulanghua_action::special_moveGoalConstPtr& current_goal) {
        if (current_goal->file == "guide_go1") {
            return true;
        } else {
            return false;
        }
    }
    bool posture_control(
        const fulanghua_action::special_moveGoalConstPtr& current_goal) {
        if ((current_goal->wp.position.x != 0 &&
             current_goal->wp.position.y != 0) &&
            (current_goal->wp.orientation.x != 0 &&
             current_goal->wp.orientation.y != 0))
            return true;
        else
            return false;
    }

    ros::NodeHandle nh;
    tf::TransformListener tf_listener_;
    std::string cmd_vel_, _dist_err, cmd_vel_posture;
    ros::Publisher twist_move_pub, twist_postgure_pub;
    ros::Subscriber robot_coordinate_sub, odom_sub, speaking_sub, charge_sub;
    geometry_msgs::Twist twist;
    nav_msgs::Odometry _odom, initial_odom;
    ;
    actionlib::SimpleActionServer<fulanghua_action::special_moveAction> server;
    actionlib::SimpleActionClient<sound_play::SoundRequestAction> sound_client;
    actionlib::SimpleActionClient<fulanghua_action::special_moveAction>
        rotate_client;
    actionlib::SimpleActionClient<fulanghua_action::special_moveAction>
        ar_detect_client;
    actionlib::SimpleActionClient<fulanghua_action::special_moveAction>
        go1_cmd_client;
    actionlib::SimpleActionClient<camera_action::camera_pkgAction>
        charging_station_client;
    actionlib::SimpleActionClient<fulanghua_action::special_moveAction>
        ar_align_client;

    // service client
    ros::ServiceClient charge_reset_srv;

    ros::Rate rate_;
    const double hz = 20;
    bool speak_start = false;
    bool isPosAvailable = false;
    bool posture = false;
    std::string _voice_path;
    std::string voice_path;
    const std::string ARUCO_DETECT_SERVICE_RESET = "/arucodetect/reset";
    bool charging = false;

   private:
    // used for battery
    std::string CHARGE_TOPIC;
    std::string robot_name_;
    double battery_, battery_criteria_,
        charge_threshold_higher_;  // get the battery info;
    bool check_battery_;
    bool holonomic_;
    //--------
    const double Kp = 0.5;
    const double Kv = 0.2865;
    orne_waypoints_msgs::Pose direction;
    double velocity_x;
    double rx, ry;
    double dist_err = 0;
    const double radian_90 = 1.5708;
    const double interval = 1 / hz;
    bool initial = true;
    double steering;
    double original_angle;
    double prev_location = 0;
    bool speaking = false;
    double max_vel;
    double min_vel;
    double t = 0;
    double prev_diff = 0;
    double voice_volume;
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response res;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "task_server");
    SpecialMove SpM;
    ros::Time start_time;
    ros::Rate loop_rate(SpM.hz);
    bool initial_goal = true;
    // Server server;

    // SpM.speaking_sub = SpM.nh.subscribe("sound_play/is_speaking", 1000,
    // &SpecialMove::speaking_callback, &SpM);
    fulanghua_action::special_moveGoalConstPtr current_goal;
    while (ros::ok()) {
        if (SpM.server.isNewGoalAvailable()) {
            current_goal = SpM.server.acceptNewGoal();
            start_time = ros::Time::now();
            printf("Update Goal\n");
        }
        if (SpM.server.isActive()) {
            if (SpM.server.isPreemptRequested()) {
                SpM.server.setPreempted();
                printf("Preempt Goal\n");
            } else {
                geometry_msgs::Twist twist;
                if (start_time + ros::Duration(current_goal->duration) < ros::Time::now()){
                    // ros::Time::now()) {
                    // SpM.server.setPreempted();
                    // printf("Preempt Goal\n");
                } else {
                    fulanghua_action::special_moveFeedback feedback;
                    feedback.rate = (ros::Time::now() - start_time).toSec() /
                                    current_goal->duration;
                    SpM.server.publishFeedback(feedback);
                    // printf("Active: publish feedback id:%i\n",
                    // current_goal->task_id);
                    std::cout << "received command: " << current_goal->command
                              << std::endl;
                    // printf("Active: publish result id:%i\n",
                    // current_goal->task_id);
                    if (current_goal->command == "stop") {
                        // provisionally set the charging funtion here
                        printf("stop\n");
                        if (!SpM.charging) SpM.chargingFunction();

                        // twist.linear.x = 0;
                        // twist.angular.z = 0;
                    } else if (current_goal->command == "speak") {
                        printf("speak\n");
                        if (initial_goal) {
                            SpM.isPosAvailable =
                                SpM.isPostureAvailable(current_goal);
                            initial_goal = false;
                        }
                        twist.linear.x = 0;
                        twist.angular.z = 0;
                        SpM.posture = SpM.posture_control(current_goal);
                        if (SpM.isPosAvailable && SpM.posture) {
                            twist.linear.x = 1;
                            twist.angular.z = 0;
                            double start_send_time = ros::Time::now().toSec();
                            double end_send_time = ros::Time::now().toSec();
                            while ((end_send_time - start_send_time) < 0.5) {
                                SpM.twist_postgure_pub.publish(twist);
                                end_send_time = ros::Time::now().toSec();
                                loop_rate.sleep();
                            }
                        }
                        if (!SpM.speak_start) {
                            SpM.speaking_function(current_goal->file);
                        }
                    } else if (current_goal->command == "guide") {
                        printf("guide\n");
                        SpM.voice_path = SpM._voice_path + "basic/";
                        if (initial_goal) {
                            SpM.isPosAvailable =
                                SpM.isPostureAvailable(current_goal);
                            initial_goal = false;
                        }
                        if (!SpM.speak_start) {
                            SpM.speaking_function(current_goal->file);
                            SpM.charging = false;
                        }
                    } else if (current_goal->command == "charge") {
                        SpM.chargingFunction();
                    } else if (current_goal->command == "p2p") {
                        SpM.P2P_move(current_goal->wp);
                    } else if (current_goal->command == "takephoto") {
                        printf("take photo\n");
                    } else if (current_goal->command == "videostream") {
                        printf(" watch video\n");
                    } else if (current_goal->command == "align1" || current_goal->command == "align2") {
                        SpM.AlignmentFunction();
                    }
                }
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}