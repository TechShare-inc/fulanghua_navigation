#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include "state_trigger_panel.h"

namespace orne_rviz_plugins
{

StateTriggerPanel::StateTriggerPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  start_client_ = nh_.serviceClient<std_srvs::Trigger>("start_wp_nav", false);
  resume_client_ = nh_.serviceClient<std_srvs::Empty>("resume_wp_pose", false);
  loop_start_client_ = nh_.serviceClient<std_srvs::Empty>("loop_start_wp_nav", false);
  loop_stop_client_ = nh_.serviceClient<std_srvs::Empty>("loop_stop_wp_nav", false);
  roundtrip_on_client_ =  nh_.serviceClient<std_srvs::Empty>("roundtrip_on_nav", false);
  roundtrip_off_client_ = nh_.serviceClient<std_srvs::Empty>("roundtrip_off_nav", false);
  pause_client_ = nh_.serviceClient<std_srvs::Empty>("pause_wp_nav", false);
  stop_client_ = nh_.serviceClient<std_srvs::Empty>("stop_wp_nav", false);
  action_client_ = nh_.serviceClient<std_srvs::Empty>("finish_action", false);


  start_nav_button_ = new QPushButton("StartWaypointsNavigation");
  resume_nav_button_ = new QPushButton("ResumeWaypointsNavigation");
  loop_start_button_ =  new QPushButton("LoopOn");
  loop_stop_button_ =  new QPushButton("LoopOff");
  roundtrip_on_button_ =  new QPushButton("RoundTrip_On");
  roundtrip_off_button_ =  new QPushButton("RoundTrip_Off");
  pause_button_ =  new QPushButton("PauseWaypointsNavigation");
  stop_button_ =  new QPushButton("Stop(Cancel)WaypointsNavigation");
  finish_action_button_ =  new QPushButton("Finish_Action");

  QGridLayout* layout = new QGridLayout;
  layout->addWidget(start_nav_button_,0,0);
  layout->addWidget(resume_nav_button_,0,1);
  layout->addWidget(pause_button_,0,2);
  layout->addWidget(stop_button_,1,0);
  layout->addWidget(loop_start_button_,1,1);
  layout->addWidget(loop_stop_button_,1,2);
  layout->addWidget(roundtrip_on_button_,2,0);
  layout->addWidget(roundtrip_off_button_,2,1);
  layout->addWidget(finish_action_button_,2,2);
  setLayout( layout );
  
  connect(start_nav_button_, SIGNAL(clicked()), this, SLOT(pushStartNavigation()));
  connect(resume_nav_button_, SIGNAL(clicked()), this, SLOT(pushResumeNavigation()));
  connect(loop_start_button_, SIGNAL(clicked()), this, SLOT(pushLoopStart()));
  connect(loop_stop_button_, SIGNAL(clicked()), this, SLOT(pushLoopStop()));

  connect(pause_button_, SIGNAL(clicked()), this, SLOT(pushPauseNavigation()));
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(pushStopNavigation()));
  connect(roundtrip_on_button_, SIGNAL(clicked()), this, SLOT(pushRountTripOn()));
  connect(roundtrip_off_button_, SIGNAL(clicked()), this, SLOT(pushRountTripOff()));
  connect(finish_action_button_, SIGNAL(clicked()), this, SLOT(pushFinishAction()));
}

void StateTriggerPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
}

void StateTriggerPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
}

void StateTriggerPanel::pushStartNavigation() {
    ROS_INFO("Service call: start waypoints navigation");
    
    std_srvs::Trigger trigger;
    start_client_.call(trigger);
}

void StateTriggerPanel::pushResumeNavigation() {
    ROS_INFO("Service call: resume waypoints navigation");
    
    std_srvs::Empty empty;
    resume_client_.call(empty);
}
void StateTriggerPanel::pushLoopStart() {
    ROS_INFO("Service call: loop start navigation");
    
    std_srvs::Empty empty;
    loop_start_client_.call(empty);
}

void StateTriggerPanel::pushLoopStop() {
    ROS_INFO("Service call: loop stop navigation");
    
    std_srvs::Empty empty;
    loop_stop_client_.call(empty);
}


void StateTriggerPanel::pushStopNavigation() {
    ROS_INFO("Service call: stop navigation");
    
    std_srvs::Empty empty;
    
    stop_client_.call(empty);
}

void StateTriggerPanel::pushPauseNavigation() {
    ROS_INFO("Service call:  pause navigation");
    
    std_srvs::Empty empty;
    pause_client_.call(empty);
}

void StateTriggerPanel::pushRountTripOn() {
    ROS_INFO("Service call: Round trip on");
    
    std_srvs::Empty empty;
    roundtrip_on_client_.call(empty);
}

void StateTriggerPanel::pushRountTripOff() {
    ROS_INFO("Service call: Round trip off");
    
    std_srvs::Empty empty;
    roundtrip_off_client_.call(empty);
}

void StateTriggerPanel::pushFinishAction() {
    ROS_INFO("Service call:Finish_Action");
    std_srvs::Empty empty;
    action_client_.call(empty);
}


} // end namespace orne_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(orne_rviz_plugins::StateTriggerPanel,rviz::Panel )
