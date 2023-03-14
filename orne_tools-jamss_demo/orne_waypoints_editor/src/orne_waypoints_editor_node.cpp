#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <std_msgs/String.h>
#include "orne_waypoints_msgs/Pose.h"
#include <std_srvs/Trigger.h>

#include <yaml-cpp/yaml.h>

#include <string>
#include <fstream>
#include <math.h>
#include <stdio.h>
#include <dirent.h>
#include <iostream>
#include <algorithm>

#ifdef NEW_YAMLCPP
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
    i = node.as<T>();
}
#endif

using namespace visualization_msgs;

class WaypointsEditor{
public:
    WaypointsEditor() :
        filename_(""), fp_flag_(false), rate_(2)
    {
        ros::NodeHandle private_nh("~");
        private_nh.param("world_frame", world_frame_, std::string("map"));
        private_nh.param("robot_frame", robot_frame_, std::string("base_link"));
        private_nh.param("save_joy_button", save_joy_button_, 0);

        server.reset(new interactive_markers::InteractiveMarkerServer("waypoints_marker_server", "", false));
        
        initMenu();

        ros::NodeHandle nh;
        waypoints_viz_sub_ = nh.subscribe("waypoints_viz", 1, &WaypointsEditor::waypointsVizCallback, this);
        waypoints_joy_sub_ = nh.subscribe("waypoints_joy", 1, &WaypointsEditor::waypointsJoyCallback, this);
        finish_pose_sub_ = nh.subscribe("finish_pose", 1, &WaypointsEditor::finishPoseCallback, this);
        marker_description_pub_ = nh.advertise<visualization_msgs::MarkerArray>("marker_descriptions",1);
        save_server_ = nh.advertiseService("save_waypoints", &WaypointsEditor::saveWaypointsCallback, this);

        private_nh.param("filename", filename_, filename_);
        
        if(filename_ != ""){
            ROS_INFO_STREAM("Read waypoints data from " << filename_);
            if(readFile(filename_)) {
                fp_flag_ = true;
                makeMarker();
            } else {
                ROS_ERROR("Failed loading waypoints file");
            }
        } else {
            ROS_ERROR("waypoints file doesn't have name");
        }
        
    }


    ~WaypointsEditor(){
        server.reset();
    }

    void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
        std::ostringstream s;
        s << "Feedback from marker '" << feedback->marker_name << "'";
    
        std::ostringstream mouse_point_ss;
    
        switch(feedback->event_type){
          case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
            ROS_INFO_STREAM(s.str() << ":button click" << mouse_point_ss.str() << ".");
            break;
    
          case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
            ROS_INFO_STREAM(s.str() << ":menu item" << feedback->menu_entry_id << "clicked" << mouse_point_ss.str() << ".");
            break;
    
          case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
            ROS_INFO_STREAM(s.str() << ":pose changed"
                << "\nposition = "
                << feedback->pose.position.x
                << "," << feedback->pose.position.y
                << "," << feedback->pose.position.z
                << "\norientation = "
                << "," << feedback->pose.orientation.x
                << "," << feedback->pose.orientation.y
                << "," << feedback->pose.orientation.z);
            break;
        }

        server->applyChanges();

        if (feedback->marker_name == "finish_pose") {
            finish_pose_.pose = feedback->pose;
        } else {
          waypoints_.at(std::stoi(feedback->marker_name)).position.x = feedback->pose.position.x;
          waypoints_.at(std::stoi(feedback->marker_name)).position.y = feedback->pose.position.y;
          waypoints_.at(std::stoi(feedback->marker_name)).position.z = feedback->pose.position.z;
          waypoints_.at(std::stoi(feedback->marker_name)).orientation.x = feedback->pose.orientation.x;
          waypoints_.at(std::stoi(feedback->marker_name)).orientation.y = feedback->pose.orientation.y;
          waypoints_.at(std::stoi(feedback->marker_name)).orientation.z = feedback->pose.orientation.z;
          waypoints_.at(std::stoi(feedback->marker_name)).orientation.w = feedback->pose.orientation.w;
        }
    }

    void initMenu(){
        interactive_markers::MenuHandler::EntryHandle wp_delete_menu_handler = wp_menu_handler_.insert("delete", boost::bind(&WaypointsEditor::wpDeleteCb, this, _1)); //3
        interactive_markers::MenuHandler::EntryHandle wp_insert_menu_handler = wp_menu_handler_.insert("Insert");
        interactive_markers::MenuHandler::EntryHandle wp_action_menu_handler = wp_menu_handler_.insert("setAction");

        interactive_markers::MenuHandler::EntryHandle wp_mode = wp_menu_handler_.insert(wp_insert_menu_handler, "Prev", boost::bind(&WaypointsEditor::wpInsertCb, this, _1)); //4
        wp_mode = wp_menu_handler_.insert(wp_insert_menu_handler, "Next", boost::bind(&WaypointsEditor::wpInsertCb, this, _1)); //5


        //set action
        interactive_markers::MenuHandler::EntryHandle action_mode = wp_menu_handler_.insert(wp_action_menu_handler, "Pass Through", boost::bind(&WaypointsEditor::actionCb, this, _1)); //6
        wp_menu_handler_.insert(wp_action_menu_handler, "Align1", boost::bind(&WaypointsEditor::actionCb, this, _1)); //7
        wp_menu_handler_.insert(wp_action_menu_handler, "Align2", boost::bind(&WaypointsEditor::actionCb, this, _1)); //8
    }

    void actionCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
        ROS_INFO_STREAM("menu_entry_id : " << feedback->menu_entry_id);
        int wp_num= std::stoi(feedback->marker_name);
        ROS_INFO_STREAM("set action : " << feedback->menu_entry_id);

        if(feedback->menu_entry_id == 6){
            ROS_INFO_STREAM("Pass Through");
            waypoints_.at(wp_num).position.action = "passthrough";
            waypoints_.at(wp_num).position.duration = 0;
            waypoints_.at(wp_num).position.file = "none";
        }else if(feedback->menu_entry_id == 7){
            waypoints_.at(wp_num).position.action = "align1";
            waypoints_.at(wp_num).position.duration = INT_MAX;
            waypoints_.at(wp_num).position.file = "none";
        }else if(feedback->menu_entry_id == 8){
            waypoints_.at(wp_num).position.action = "align2";
            waypoints_.at(wp_num).position.duration = INT_MAX;
            waypoints_.at(wp_num).position.file = "none";
        }

        makeWpsInteractiveMarker();
        server->applyChanges();
    }


    void wpDeleteCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
        ROS_INFO_STREAM("delete : " << feedback->marker_name);
        int wp_num = std::stoi(feedback->marker_name);
        waypoints_.erase(waypoints_.begin() + wp_num);
        for (int i=wp_num; i<waypoints_.size(); i++) {
            geometry_msgs::Pose wp;
            wp.position.x = waypoints_.at(i).position.x;
            wp.position.y = waypoints_.at(i).position.y;
            wp.position.z = waypoints_.at(i).position.z;
            server->setPose(std::to_string(i), wp);
        }
        server->erase(std::to_string((int)waypoints_.size()));
        makeWpsInteractiveMarker();
        server->applyChanges();
    }

    void wpInsertCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
        ROS_INFO_STREAM("menu_entry_id : " << feedback->menu_entry_id);
        int wp_num= std::stoi(feedback->marker_name);
        ROS_INFO_STREAM("insert : " << feedback->menu_entry_id);
        geometry_msgs::Pose p = feedback->pose;
        orne_waypoints_msgs::Pose wp;
        if (feedback->menu_entry_id == 4){
            p.position.x -= 1.0;
            wp.position.x = p.position.x;
            wp.position.y = p.position.y;
            wp.position.z = p.position.z;
            wp.position.action = "passthrough";
            wp.position.duration =0;
            wp.position.file = "none"; 
            waypoints_.insert(waypoints_.begin() + wp_num, wp);

        } else if (feedback->menu_entry_id == 5) {
            p.position.x += + 1.0;
            wp.position.x = p.position.x;
            wp.position.y = p.position.y;
            wp.position.z = p.position.z;
            wp.position.action = "passthrough";
            wp.position.duration =0;
            wp.position.file = "none"; 
            waypoints_.insert(waypoints_.begin() + wp_num + 1, wp);
        }

        for (int i=wp_num; i<waypoints_.size()-1; i++) {
            geometry_msgs::Pose p;
            p.position.x = waypoints_.at(i).position.x;
            p.position.y = waypoints_.at(i).position.y;
            p.position.z = waypoints_.at(i).position.z;
            server->setPose(std::to_string(i), p);
        }
        makeWpsInteractiveMarker();
        server->applyChanges();
    }

    void makeMarker(){
        makeWpsInteractiveMarker();
        makeFinishPoseMarker();

        server->applyChanges();
    }

    void publishMarkerDescription(){
        marker_description_.markers.clear();
        for(int i=0; i!=waypoints_.size(); i++){
            Marker marker;
            marker.type = Marker::TEXT_VIEW_FACING;
            marker.text = std::to_string(i) +":" +waypoints_.at(i).position.action;
            marker.header.frame_id = world_frame_;
            marker.header.stamp = ros::Time(0);
            std::stringstream name;
            name << "waypoint";
            marker.ns = name.str();
            marker.id = i;
            marker.lifetime = ros::Duration(0.5);
            marker.pose.position.x = waypoints_.at(i).position.x+ 0.5;
            marker.pose.position.y = waypoints_.at(i).position.y;
            marker.pose.position.z = 3.0;
            marker.pose.orientation.x = waypoints_.at(i).orientation.x;
            marker.pose.orientation.y = waypoints_.at(i).orientation.y;
            marker.pose.orientation.z = waypoints_.at(i).orientation.z;
            marker.pose.orientation.w = waypoints_.at(i).orientation.w;
            marker.scale.z = 0.5;
            marker.color.r = 2.0;
            marker.color.g = 2.0;
            marker.color.b = 3.0;
            marker.color.a = 1.0;
            marker.action = visualization_msgs::Marker::ADD;
            marker_description_.markers.push_back(marker);
        }if(fp_flag_){
            Marker marker;
            marker.type = Marker::TEXT_VIEW_FACING;
            marker.text = "goal";
            marker.header.frame_id = world_frame_;
            marker.header.stamp = ros::Time(0);
            std::stringstream name;
            name << "finish pose";
            marker.ns = name.str();
            marker.id = 0;
            marker.lifetime = ros::Duration(0.5);
            marker.pose = finish_pose_.pose;
            marker.pose.position.z = 3.0;
            marker.scale.z = 0.5;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.action = visualization_msgs::Marker::ADD;
            marker_description_.markers.push_back(marker);
        }
        marker_description_pub_.publish(marker_description_);
    }

    Marker makeWpMarker(const std::string &action){
        Marker marker;

        if (action=="align1"){
            marker.type = Marker::ARROW;
            marker.scale.x = 0.7;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.r = 0.3;
            marker.color.g = 0.6;
            marker.color.b = 1.0;
            marker.color.a = 0.5;
        }else if (action=="align2"){
            marker.type = Marker::ARROW;
            marker.scale.x = 0.7;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.r = 0.3;
            marker.color.g = 0.6;
            marker.color.b = 1.0;
            marker.color.a = 0.5;
        } else{
            marker.type = Marker::SPHERE;
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.color.r = 0.08;
            marker.color.g = 2.0;
            marker.color.b = 0.8;
            marker.color.a = 0.5;
        }

        return marker;
    }
    
    InteractiveMarkerControl& makeWpControl(InteractiveMarker &msg, const std::string &action) {
        InteractiveMarkerControl control;
        control.markers.clear();
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        if(action=="align1" || action=="align2")
            control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
        else
            control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
        control.always_visible = true;
        control.markers.push_back(makeWpMarker(action));
        msg.controls.push_back(control);

        return msg.controls.back();
    }



    void makeWpInteractiveMarker(const std::string name, const orne_waypoints_msgs::Pose &point){
        InteractiveMarker int_marker;
        int_marker.controls.clear();
        int_marker.header.frame_id = world_frame_;
        int_marker.pose.position.x = point.position.x;
        int_marker.pose.position.y = point.position.y;
        int_marker.pose.position.z = point.position.z;
        int_marker.pose.orientation.x = point.orientation.x;
        int_marker.pose.orientation.y = point.orientation.y;
        int_marker.pose.orientation.z = point.orientation.z;
        int_marker.pose.orientation.w = point.orientation.w;
        int_marker.scale = 1;
        int_marker.name = name;
        int_marker.description = "";
        int_marker.controls.push_back(makeWpControl(int_marker, point.position.action));

        

        server->insert(int_marker, boost::bind(&WaypointsEditor::processFeedback, this, _1));
        wp_menu_handler_.apply(*server, name);
    }

    void makeWpsInteractiveMarker(){
        for (int i=0; i!=waypoints_.size(); i++){
            makeWpInteractiveMarker(std::to_string(i), waypoints_.at(i));
        }
    }

    void makeFinishPoseMarker(){
        if (fp_flag_) {
            InteractiveMarker int_marker;
            int_marker.controls.clear();
            int_marker.header.frame_id = world_frame_;
            int_marker.pose = finish_pose_.pose;
            int_marker.scale = 1;
            int_marker.name = "finish_pose";
            int_marker.description = "";

            InteractiveMarkerControl control;
            control.markers.clear();
            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 1;
            control.orientation.z = 0;
            control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
    
            Marker marker;
            marker.type = Marker::ARROW;
            marker.scale.x = 1.0;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;
            marker.color.r = 0.08;
            marker.color.g = 0.0;
            marker.color.b = 0.8;
            marker.color.a = 0.8;
            control.markers.push_back(marker);
            
            control.always_visible = true;
            int_marker.controls.push_back(control);
    
            server->insert(int_marker, boost::bind(&WaypointsEditor::processFeedback, this, _1));
        }
    }

    bool readFile(const std::string &filename){
        waypoints_.clear();
        try{
            std::ifstream ifs(filename.c_str(), std::ifstream::in);
            if(ifs.good() == false){
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

            if(wp_node != NULL){
                for(int i=0; i < wp_node->size(); i++){
                    orne_waypoints_msgs::Pose pose;
                    (*wp_node)[i]["point"]["pose"]["x"] >> pose.position.x;
                    (*wp_node)[i]["point"]["pose"]["y"] >> pose.position.y;
                    (*wp_node)[i]["point"]["pose"]["z"] >> pose.position.z;
                    (*wp_node)[i]["point"]["action"]["a"] >> pose.position.action;
                    (*wp_node)[i]["point"]["action"]["d"] >> pose.position.duration;
                    (*wp_node)[i]["point"]["action"]["f"] >> pose.position.file;
                    (*wp_node)[i]["point"]["orientation"]["x"] >> pose.orientation.x;
                    (*wp_node)[i]["point"]["orientation"]["y"] >> pose.orientation.y;
                    (*wp_node)[i]["point"]["orientation"]["z"] >> pose.orientation.z;
                    (*wp_node)[i]["point"]["orientation"]["w"] >> pose.orientation.w;
                    waypoints_.push_back(pose);
                    //I think here I need to push_back the action below
                    
                }
            }else{
                return false;
            }

            #ifdef NEW_YAMLCPP
                const YAML::Node &fp_node_tmp = node["finish_pose"];
                const YAML::Node *fp_node = fp_node_tmp ? &fp_node_tmp : NULL;
            #else
                const YAML::Node *fp_node = node.FindValue("finish_pose");
            #endif

            if(fp_node != NULL){
                (*fp_node)["pose"]["position"]["x"] >> finish_pose_.pose.position.x;
                (*fp_node)["pose"]["position"]["y"] >> finish_pose_.pose.position.y;
                (*fp_node)["pose"]["position"]["z"] >> finish_pose_.pose.position.z;

                (*fp_node)["pose"]["orientation"]["x"] >> finish_pose_.pose.orientation.x;
                (*fp_node)["pose"]["orientation"]["y"] >> finish_pose_.pose.orientation.y;
                (*fp_node)["pose"]["orientation"]["z"] >> finish_pose_.pose.orientation.z;
                (*fp_node)["pose"]["orientation"]["w"] >> finish_pose_.pose.orientation.w;
            }else{
                return false;
            }

        }catch(YAML::ParserException &e){
            return false;

        }catch(YAML::RepresentationException &e){
            return false;
        }

        return true;
    }

    void waypointsVizCallback(const geometry_msgs::PointStamped &msg){
        orne_waypoints_msgs::Pose _wp;
        _wp.position.x = msg.point.x;
        _wp.position.y = msg.point.y;
        _wp.position.z = msg.point.z;
        // _wp.position.x = msg.pose.position.x;
        // _wp.position.y = msg.pose.position.y;
        // _wp.position.z = msg.pose.position.z;
        _wp.position.action = "passthrough";
        _wp.position.duration = 0;
        _wp.position.file = "none";
        // _wp.orientation.x = msg.pose.orientation.x;
        // _wp.orientation.y = msg.pose.orientation.y;
        // _wp.orientation.z = msg.pose.orientation.z;
        // _wp.orientation.w = msg.pose.orientation.w;
        ROS_INFO_STREAM("point = " << msg);
        makeWpInteractiveMarker(std::to_string(waypoints_.size()), _wp);
        server->applyChanges();

        waypoints_.push_back(_wp);
    }

    void waypointsJoyCallback(const sensor_msgs::Joy &msg) {
        static ros::Time saved_time(0.0);
        if(msg.buttons[save_joy_button_] == 1 && (ros::Time(0) - saved_time).toSec() > 3.0){
            tf::StampedTransform robot_gl;
            try{
                tf_listener_.lookupTransform(world_frame_, robot_frame_, msg.header.stamp, robot_gl);
                orne_waypoints_msgs::Pose pose;
                pose.position.x = robot_gl.getOrigin().x();
                pose.position.y = robot_gl.getOrigin().y();
                pose.position.z = robot_gl.getOrigin().z();
                makeWpInteractiveMarker(std::to_string(waypoints_.size()), pose);
                waypoints_.push_back(pose);
                server->applyChanges();
                saved_time = msg.header.stamp;
            }catch(tf::TransformException &e){
                ROS_WARN_STREAM("tf::TransformException: " << e.what());
            }
        }
    }

    void finishPoseCallback(const geometry_msgs::PoseStamped &msg){
        ROS_INFO_STREAM("pose = " << msg);
        finish_pose_ = msg;
        if (fp_flag_) {
            server->setPose("finish_pose", msg.pose);
        } else {
            fp_flag_ = true;
            makeFinishPoseMarker();
        }
        server->applyChanges();
    }

    bool saveWaypointsCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response) {
        if (save()) {
            response.success = true;
            return true;
        } else {
            response.success = false;
            return false;
        }
    }

    bool save(){
        if (fp_flag_) {
            std::ofstream ofs(filename_.c_str(), std::ios::out);
            
            ofs << "waypoints:" << std::endl;
            for(int i=0; i < waypoints_.size(); i++){
                ofs << "    - point:" << std::endl;
                ofs << "        pose:" << std::endl;
                ofs << "            x: " << waypoints_[i].position.x << std::endl;
                ofs << "            y: " << waypoints_[i].position.y << std::endl;
                ofs << "            z: " << waypoints_[i].position.z << std::endl;
                ofs << "        action:" << std::endl;
                ofs << "            a: " << waypoints_[i].position.action  << std::endl;
                ofs << "            d: " << waypoints_[i].position.duration  << std::endl;
                ofs << "            f: " << waypoints_[i].position.file  << std::endl;
                ofs << "        orientation:" << std::endl;
                ofs << "            x: "        << waypoints_[i].orientation.x << std::endl;
                ofs << "            y: "        << waypoints_[i].orientation.y << std::endl;
                ofs << "            z: "        << waypoints_[i].orientation.z << std::endl;
                ofs << "            w: "        << waypoints_[i].orientation.w << std::endl;
            }
            
            ofs << "finish_pose:"           << std::endl;
            ofs << "    header:"            << std::endl;
            ofs << "        seq: "          << finish_pose_.header.seq << std::endl;
            ofs << "        stamp: "        << finish_pose_.header.stamp << std::endl;
            ofs << "        frame_id: "     << finish_pose_.header.frame_id << std::endl;;
            ofs << "    pose:"              << std::endl;
            ofs << "        position:"      << std::endl;
            ofs << "            x: "        << finish_pose_.pose.position.x << std::endl;
            ofs << "            y: "        << finish_pose_.pose.position.y << std::endl;
            ofs << "            z: "        << finish_pose_.pose.position.z << std::endl;
            ofs << "        orientation:"   << std::endl;
            ofs << "            x: "        << finish_pose_.pose.orientation.x << std::endl;
            ofs << "            y: "        << finish_pose_.pose.orientation.y << std::endl;
            ofs << "            z: "        << finish_pose_.pose.orientation.z << std::endl;
            ofs << "            w: "        << finish_pose_.pose.orientation.w << std::endl;

            ofs.close();

            ROS_INFO_STREAM("write success");
            return true;
        } else {
            ROS_WARN_STREAM("Please set finish_pose");
            return false;
        }

    }



    // }

    void run() {
        while(ros::ok()){
            rate_.sleep();
            ros::spinOnce();
            publishMarkerDescription();
        }
    }

private:
    ros::Subscriber waypoints_viz_sub_;
    ros::Subscriber waypoints_joy_sub_;
    ros::Subscriber finish_pose_sub_;
    ros::Publisher marker_description_pub_;
    std::vector<orne_waypoints_msgs::Pose> waypoints_;
    geometry_msgs::PoseStamped finish_pose_;
    visualization_msgs::MarkerArray marker_description_;
    tf::TransformListener tf_listener_;
    int save_joy_button_;
    std::string filename_;
    std::string world_frame_;
    std::string robot_frame_;
    ros::ServiceServer save_server_;
    bool fp_flag_;
    ros::Rate rate_;

    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
    interactive_markers::MenuHandler wp_menu_handler_;

};


int main(int argc, char** argv){
  ros::init(argc, argv, "pose_marker");
  WaypointsEditor w_edit;
  w_edit.run();

  return 0;
}
