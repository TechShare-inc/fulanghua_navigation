#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <camera_action/camera_pkgAction.h>
#include <fulanghua_action/special_moveAction.h>
#include <fulanghua_msg/_LimoStatus.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
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
          rotate_client("rotate", true),
          ar_align_client("ar_align", true),
          rate_(2) {
        ros::NodeHandle private_nh("~");
        private_nh.param("cmd_vel", cmd_vel_, std::string("cmd_vel"));
        private_nh.param("holonomic", holonomic_, true);
        private_nh.param("max_vel", max_vel, 0.4);
        private_nh.param("min_vel", min_vel, 0.1);
        private_nh.param("dist_err", dist_err, 0.8);
        twist_move_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_, 100);
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

    ros::NodeHandle nh;
    tf::TransformListener tf_listener_;
    std::string cmd_vel_, _dist_err;
    ros::Publisher twist_move_pub;
    ros::Subscriber robot_coordinate_sub, odom_sub;
    geometry_msgs::Twist twist;
    nav_msgs::Odometry _odom, initial_odom;
    ;
    actionlib::SimpleActionServer<fulanghua_action::special_moveAction> server;
    actionlib::SimpleActionClient<fulanghua_action::special_moveAction>
        rotate_client;
    actionlib::SimpleActionClient<fulanghua_action::special_moveAction>
        ar_align_client;


    ros::Rate rate_;
    const double hz = 20;
    bool isPosAvailable = false;

   private:
    std::string robot_name_;
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
    double max_vel;
    double min_vel;
    double t = 0;
    double prev_diff = 0;
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
                fulanghua_action::special_moveFeedback feedback;
                feedback.rate = (ros::Time::now() - start_time).toSec() /
                                current_goal->duration;
                SpM.server.publishFeedback(feedback);
                std::cout << "received command: " << current_goal->command
                            << std::endl;
                if (current_goal->command == "guide") {
                    printf("guide\n");
                    if (initial_goal) {
                        initial_goal = false;
                    }
                    SpM.server.setSucceeded();
                } else if (current_goal->command == "align1" || current_goal->command == "align2") {
                    SpM.AlignmentFunction();
                }
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}