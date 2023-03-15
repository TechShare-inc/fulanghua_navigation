#ifndef STATE_TRIGGER_PANEL_H
#define STATE_TRIGGER_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

class QPushButton;

namespace orne_rviz_plugins
{

class StateTriggerPanel: public rviz::Panel
{
Q_OBJECT
public:
  StateTriggerPanel( QWidget* parent = 0 );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

public Q_SLOTS:
  void pushStartNavigation();
  void pushResumeNavigation();
  void pushLoopStart();
  void pushLoopStop();
  void pushStopNavigation();
  void pushPauseNavigation();
  void pushRountTripOn();
  void pushRountTripOff();
  void pushFinishAction();
  
protected:
  ros::NodeHandle nh_;
  ros::ServiceClient start_client_, resume_client_, loop_start_client_, loop_stop_client_, action_client_, 
  roundtrip_on_client_, roundtrip_off_client_, pause_client_, stop_client_;
  QPushButton *start_nav_button_;
  QPushButton *resume_nav_button_;
  QPushButton *loop_start_button_;
  QPushButton *loop_stop_button_;
  QPushButton *roundtrip_on_button_;
  QPushButton *roundtrip_off_button_;
  QPushButton *pause_button_;
  QPushButton *stop_button_;
  QPushButton *finish_action_button_;
};

} // end namespace orne_rviz_plugins

#endif // STATE_TRIGGER_PANEL_H
