/*****************************************************************************
** Ifdefs
*****************************************************************************/
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <string>

#include <QMainWindow>
#include <QMessageBox>
#include <QStringListModel>
#include <QThread>
#include <QStandardItemModel>

#include "tm_msgs/FeedbackState.h"
#include "tm_msgs/SctResponse.h"
#include "tm_msgs/StaResponse.h"
#include "tm_msgs/SvrResponse.h"
#include "tm_msgs/ConnectTM.h"
#include "tm_msgs/SetIO.h"
#include <ctime>

using std::chrono::system_clock;
/*****************************************************************************
** Namespaces
*****************************************************************************/
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

/*****************************************************************************
** Class
*****************************************************************************/
class RosPage : public QThread {
 Q_OBJECT
 
public:
  RosPage();
  ~RosPage();
  void run();

private:
  ros::NodeHandle node;
  ros::Subscriber feedBackStatusSubscription;
  ros::Subscriber sctResponseSubscription;
  ros::Subscriber staResponseSubscription;
  ros::Subscriber svrResponseSubscription;
  ros::ServiceClient connectClient;
  void feedback_states_callback(const tm_msgs::FeedbackState::ConstPtr& msg);
  void sct_response_callback(const tm_msgs::SctResponse::ConstPtr& msg);
  void sta_response_callback(const tm_msgs::StaResponse::ConstPtr& msg);
  void svr_response_callback(const tm_msgs::SvrResponse::ConstPtr& msg);
  void send_service(ros::ServiceClient client,tm_msgs::ConnectTM request);
  bool lastStatus;
  void initial_subscriber();
  void initial_client();
  std::string current_time();
  
signals:
  void send_ui_feed_back_status(tm_msgs::FeedbackState msg);
  void send_to_ui_list(std::string);
  
private slots:
  void send_sct_as_re_connect();
  void send_svr_as_re_connect();
  void change_control_box_io_button();
  
};

class MainWindow : public QDialog {
  Q_OBJECT

public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

private:
  Ui::MainWindow *ui;
  std::shared_ptr<RosPage> rosPage;
  std::shared_ptr<QStandardItemModel> statusItemModel;
  int rowIndex;
  void initial_ros_thread_to_ui_page();
  void initial_ui_page_to_ros_thread();
  void set_text_nan_initial(QLabel* label);    
  void set_text_true_false(bool isTrue, QLabel* label, bool isReverse);
  void set_text_on_off(bool isTrue, QLabel* label, bool isReverse);
  void set_text_high_low(bool isTrue, QLabel* label, bool isReverse);
  void set_text_null_reserve(bool isTrue, QLabel* label);    
  void initial_text_ctrl_label();      
  void initial_ui_component();
  QString format_change(std::string msg);
  
signals:
  void send_sct_as_re_connect();
  void send_svr_as_re_connect();
  void change_control_box_io_button();

private slots:
  void send_ui_feed_back_status(tm_msgs::FeedbackState msg);
  void send_to_ui_list(std::string);
  void click_set_sct_re_connect_button();
  void click_set_svr_re_connect_button();
  void click_change_control_box_io_button();
  void click_clear_response_button();
  void quit();  

};
#endif // MAINWINDOW_H