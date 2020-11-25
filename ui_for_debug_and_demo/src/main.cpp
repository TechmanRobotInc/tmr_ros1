#include "tm_ros_driver_windows.hpp"

#include <QApplication>


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "robot_ui");
  qRegisterMetaType<tm_msgs::FeedbackState>("tm_msgs::FeedbackState");
  qRegisterMetaType<std::string>("std::string");
  QApplication a(argc, argv);
  MainWindow w;
  w.show();
  return a.exec();
}