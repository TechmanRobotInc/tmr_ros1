/*****************************************************************************
** Includes
*****************************************************************************/
#include "tm_ros_driver_windows.hpp"
#include <QApplication>
#include <ros/console.h>

/*****************************************************************************
** Main
*****************************************************************************/
int main(int argc, char *argv[])
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn))//Debug,Info,Warn,Error,Fatal
  {
      ros::console::notifyLoggerLevelsChanged();
  }
  ros::init(argc, argv, "robot_ui");
  qRegisterMetaType<tm_msgs::FeedbackState>("tm_msgs::FeedbackState");
  qRegisterMetaType<std::string>("std::string");
  QApplication a(argc, argv);
  MainWindow w;
  w.show();
  a.connect(&a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()));
  
  return a.exec();
}
