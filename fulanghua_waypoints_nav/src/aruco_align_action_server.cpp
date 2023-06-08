#include <ros/ros.h>
// Include opencv2

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Include CvBridge, Image Transport, Image msg
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <cv_bridge/cv_bridge.h>
#include <fulanghua_action/special_moveAction.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>

typedef actionlib::SimpleActionServer<fulanghua_action::special_moveAction>
    Server;
struct timespec timer_start, timer_stop;
double fstart, fstop;

class AR_ALIGN {
   public:
    AR_ALIGN()
        : rate(10),
          server(nh, "ar_align", false),
          rotate_client("rotate", true),
          wait_client("wait", true) {
        ros::NodeHandle private_nh("~");
        private_nh.param("Kp", Kp, 0.03);
        private_nh.param("Ki", Ki, 0.003);
        private_nh.param("Kpang", Kpang, 1.5);
        private_nh.param("image_topic", IMAGE_TOPIC,
                         std::string("/camera/rgb/image_raw"));
        private_nh.param("cmd_vel", CMD_VEL_TOPIC, std::string("/cmd_vel"));
        private_nh.param("adjust_speed", adjust_speed, 0.01);
        private_nh.param("holonomic", holonomic_, false);
        private_nh.param("offset_fixed_x", fixed_x, 0.35);
        private_nh.param("offset_fixed_y", fixed_y, 0.00);
        private_nh.param("offset_fixed_z", fixed_z, 0.37);
        private_nh.param("calibration_path", CALIBRATION, std::string(""));
        image_sub =
            nh.subscribe(IMAGE_TOPIC, 1000, &AR_ALIGN::image_callback, this);
        scan_sub = nh.subscribe("/scan", 1000, &AR_ALIGN::scan_callback, this);
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(CMD_VEL_TOPIC, 10);
        cv::FileStorage fs;
        fs.open(CALIBRATION, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            std::cout << "Failed to open " << CALIBRATION << std::endl;
        }
        fs["camera_matrix"] >> camera_matrix;
        fs["distortion_coefficients"] >> dist_coeffs;
        std::cout << "camera_matrix\n" << camera_matrix << std::endl;
        std::cout << "\ndist coeffs\n" << dist_coeffs << std::endl;

        server.start();  // start the server
    }

    void image_callback(const sensor_msgs::ImageConstPtr &msg) {
        std_msgs::Header msg_header = msg->header;
        std::string frame_id = msg_header.frame_id.c_str();
        // ROS_INFO_STREAM("New Image from " << frame_id);

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr =
                cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        src = cv_ptr->image;
        w = src.size().width;
        h = src.size().height;
        // cv::line(src, cv::Point(w/3,0), cv::Point(w/3,h), cv::Scalar(255, 0,
        // 0), thickness, cv::LINE_8); cv::line(src, cv::Point(w/2,0),
        // cv::Point(w/2,h), cv::Scalar(255, 255, 0), thickness, cv::LINE_8);
    }

    void scan_callback(const sensor_msgs::LaserScan &msg) {
        auto ranges = msg.ranges;
        int obs_num = 0;
        for (int i = 0; i < ranges.size() / 3; i++) {
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

    bool wait(){
        if(wait_client.isServerConnected()){
              bool state =true;
              fulanghua_action::special_moveGoal current_goal;
              current_goal.duration = INT_MAX-1;
              for (int i = 0; i < 5; i++)
              {
                wait_client.sendGoal(current_goal);
                ROS_INFO("send goal to wait_server");
                actionlib::SimpleClientGoalState client_state = wait_client.getState();
                while(client_state !=actionlib::SimpleClientGoalState::SUCCEEDED){
                  client_state = wait_client.getState();
                  if (client_state == actionlib::SimpleClientGoalState::PREEMPTED
                    || client_state == actionlib::SimpleClientGoalState::ABORTED){
                    ROS_WARN("failed %d times\n", i+1);
                    state = false;
                    break;
                  }
                  ros::Duration(1).sleep();
                }
                if (state){
                    break;
                }
              }
              return true;
              ros::Duration(1).sleep();
        }
    }

    void escape(){
        for(int i=0; i < 25; i++){
            twist.linear.x = 0.2;
            twist.angular.z = -0.5;
            cmd_vel_pub.publish(twist);
            ros::Duration(0.1).sleep();
        }
        for(int i=0; i < 25; i++){
            twist.linear.x = 0.1;
            twist.angular.z = 0.4;
            cmd_vel_pub.publish(twist);
            ros::Duration(0.1).sleep();
        }
    }

    bool adjustPosition(double &x, double &y, double &z, double &ang) {
        Done_x = false;
        ros::Time t_stop;
        double t = (ros::Time::now() - start_time).toSec();
        threshold_x = 0.005;//0.002;

        // robot(x, y, z) <-> aruco(z, x, y)
        offset_x = z - (double)fixed_x;
        offset_y = (double)fixed_y - x;
        offset_z = (double)fixed_z - y;
        ROS_INFO("start approching");
        offset_I += offset_x;
        double move_x = Kp * offset_x + Ki * offset_I;
        double rotate_z = Kpang * offset_y;
        ROS_INFO("move x: %lf,\n", move_x);
        twist.linear.x = move_x;
        twist.angular.z = rotate_z;  // horizontal
        clock_gettime(CLOCK_MONOTONIC, &timer_start);
        fstart = (double)timer_start.tv_sec +
                 ((double)timer_start.tv_nsec / 1000000000.0);

        // if offset is smaller than threshold, position adjastment has done
        if (std::abs(offset_x) <= threshold_x) {
            Done_x = true;
        }

        // getting the angle after position adjastment has done
        if (Done_x) {
            // Done_z = false;
            // move it to the center
            
            // wait();
            ROS_INFO("aligmnent is done!");
            double angle = 0;
            offset_I = 0;
            return true;
        } else {
            ROS_INFO("Offset_x: %lf,\n", offset_x);
            ROS_INFO("linear x: %lf,\n", twist.linear.x);
            ROS_INFO("threshold: %lf,\n", threshold_x);
            ROS_INFO("Agnle: %lf,\n", ang);
            if (stop == 1) {
                t_stop = ros::Time::now();
                while ((ros::Time::now() - t_stop).toSec() < 3) {
                    twist.linear.x = 0;
                    twist.angular.z = 0;
                    cmd_vel_pub.publish(twist);
                    ROS_INFO("stopped to avoid collision for %lf seconds",
                             (ros::Time::now() - t_stop).toSec());
                }
            } else {
                cmd_vel_pub.publish(twist);
            }
            return false;
        }
    }


    void put_commnets(double &x, double &y, double &z, double &_angle) {
        vector_to_marker.str(std::string());
        vector_to_marker << std::setprecision(4) << "x: " << std::setw(8) << z;
        cv::putText(src, vector_to_marker.str(), cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 252, 124), 1,
                    CV_AVX);

        vector_to_marker.str(std::string());
        vector_to_marker << std::setprecision(4) << "y: " << std::setw(8) << y;
        cv::putText(src, vector_to_marker.str(), cv::Point(10, 90),
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 252, 124), 1,
                    CV_AVX);

        vector_to_marker.str(std::string());
        vector_to_marker << std::setprecision(4) << "z: " << std::setw(8) << z;
        cv::putText(src, vector_to_marker.str(), cv::Point(10, 150),
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 252, 124), 1,
                    CV_AVX);

        vector_to_marker.str(std::string());
        vector_to_marker << std::setprecision(4) << "angle: " << std::setw(8)
                         << _angle;
        cv::putText(src, vector_to_marker.str(), cv::Point(10, 210),
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 252, 124), 1,
                    CV_AVX);
    }

    ros::NodeHandle nh;

    // publishers and  subscribers

    actionlib::SimpleActionServer<fulanghua_action::special_moveAction>
        server;  // make a server
    actionlib::SimpleActionClient<fulanghua_action::special_moveAction>
        rotate_client;  // for rotation client
    actionlib::SimpleActionClient<fulanghua_action::special_moveAction>
        wait_client;
    cv::Mat src, camera_matrix, dist_coeffs;
    double offset_I = 0;
    double Kp = 0;  // proportional coefficient
    double Ki = 0;
    double Kpang = 0;
    double Kv;      // derivative coefficient
    double target;  // target angle(radian)
    double offset_x = 0;
    double offset_y = 0;
    double offset_z = 0;
    bool Done_x = false;
    bool Done_y = false;
    bool Done_z = false;
    bool Done_r = false;
    double fixed_x, fixed_y, fixed_z;
    double threshold_x, threshold_y, threshold_z;
    double c_x = 0.0;
    double c_y = 0.0;
    double w = 0.0;
    double h = 0.0;  // in case the marker is on the edge
    double adjust_speed;
    double _counter = 0;
    std::vector<double> angle_array;  // for rotation
    int stop = 0;

    int thickness = 2;
    std::string CMD_VEL_TOPIC;  // target cmd_vel
    std::string IMAGE_TOPIC;    // image topic
    ros::Rate rate;             // set the rate
    geometry_msgs::Twist twist;
    ros::Time start_time;

    fulanghua_action::special_moveGoalConstPtr
        current_goal;  // instance of a goal
    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    std::ostringstream vector_to_marker;

   private:
    ros::Publisher cmd_vel_pub;
    ros::Subscriber image_sub, scan_sub;
    std::string CALIBRATION;
    bool holonomic_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "aruco_align_server");
    AR_ALIGN align;
    bool visualize = true;
    bool done_align;
    double y = 0;
    ros::Time t, t_start_wait;
    while (ros::ok()) {
        if (align.server.isNewGoalAvailable()) {
            align.current_goal = align.server.acceptNewGoal();
            align.start_time = ros::Time::now();
            t = ros::Time::now();
            align._counter = 0;
            visualize = true;
            align.Done_z = false;
            done_align = false;
            
        }
        if (align.server.isActive()) {
            if (align.server.isPreemptRequested()) {
                align.server.setPreempted();  // cancel the goal
                ROS_WARN("AR align: Preemmpt Goal\n");
            } else {
                if (align.start_time +
                        ros::Duration(align.current_goal->duration) <
                    ros::Time::now()) {
                    align.server.setAborted();  // abort it
                } else {
                    if (done_align){
                        align.escape();
                        done_align = false;
                        align.server.setSucceeded();
                        ROS_INFO("AR align: Succeeded!");
                        visualize = false;
                        
                    }else if (!align.src.empty()) {/*if camera iamges are received*/
                        cv::aruco::detectMarkers(
                            align.src, align.dictionary, align.corners,
                            align.ids);  // detecting a marker
                        std::vector<cv::Vec3d> rvecs, tvecs;
                        cv::aruco::estimatePoseSingleMarkers(
                            align.corners, 0.05, align.camera_matrix,
                            align.dist_coeffs, rvecs,
                            tvecs);  // gettting x,y,z and angle
                        if (align.ids.size() > 0) {/*if a marker is detected*/
                            t = ros::Time::now();
                            cv::drawFrameAxes(
                                align.src, align.camera_matrix,
                                align.dist_coeffs, rvecs[0], tvecs[0],
                                0.1);  // drawing them on the marker
                            double _angle = rvecs[0](2) * 180 / M_PI;
                            align.put_commnets(tvecs[0](0), tvecs[0](1),
                                               tvecs[0](2),
                                               _angle);  // putting texst on src
                            if (align.adjustPosition(
                                    tvecs[0](0), tvecs[0](1), tvecs[0](2),
                                    _angle)) {  /*align the potision to the marker*/
                                done_align = true; // alignment's done here
                                visualize = false;
                            }
                            
                        } else {/*if the robot cannot see a marker for 3 seconds, this action is aborted.*/
                            double t_preempt = (ros::Time::now() - t).toSec();
                            ROS_INFO("Cannot see the marker for %lf seconds",
                                     t_preempt);
                            if (t_preempt > 3) {
                                visualize = false;
                                align.server.setAborted();  // cancel the goal
                                ROS_WARN("AR align: aborted Goal\n");
                            }
                        }
                        t_start_wait = ros::Time::now();
                        cv::imshow("src", align.src);
                        cv::waitKey(3);
                        if (!visualize) cv::destroyAllWindows();
                    }
                }
            }
        }
        ros::spinOnce();
        align.rate.sleep();
    }
    return 0;
};
