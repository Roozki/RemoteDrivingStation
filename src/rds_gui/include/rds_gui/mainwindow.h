#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include "ui_mainwindow.h"  // This will be generated from your .ui file
#include <rclcpp/rclcpp.hpp>
//#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
//#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
//#include "autoware_auto_vehicle_msgs/msg/gear_command.hpp"
// #include "autoware_auto_vehicle_msgs/msg/gear_command.hpp"
// #include "autoware_auto_vehicle_msgs/msg/vehicle_status.hpp"
//#include "sensor_msgs/msg/joy.hpp" 
#include "rds_msgs/msg/vehicle_interface.hpp"
#include "rds_msgs/msg/hud_manage.hpp"

class ManageWindow : public QMainWindow, public rclcpp::Node {
    Q_OBJECT

public:
    ManageWindow(QWidget *parent = nullptr);
    //auto node = rclcpp::Node();
    void init(){
        //node
    }

private:
    Ui::MainWindow ui;  // Replace 'ArmControl' with the actual class name from your .ui file

//rclcpp::Subscription velocitySub<autoware_auto_vehicle_msgs>();
   // rclcpp::Subscription vehicle_1_sub<rds_msgs>
   rclcpp::Subscription<rds_msgs::msg::VehicleInterface>::SharedPtr _Vehicle_1_subscriber;

};

#endif // MAIN_WINDOW_H













// #include "../include/mainwindow.h"
// #include "ui_mainwindow.h"

// // QT
// #include "QDebug"
// #include "QDir"
// #include "QLabel"
// #include "QMessageBox"
// #include "QPixmap"
// #include "QProcess"

// #include "rclcpp.h"


// namespace Ui {
// class MainWindow;
// }
// class MainWindow : public QMainWindow {
//     Q_OBJECT
//   public:
//     explicit MainWindow(QWidget* parent = nullptr);
//     virtual ~MainWindow();
//     bool ui_debug = false;


//     void handleButton1(){
//         //ui->Pos1_button->setText("Clicked!");
        
//         // std_msgs::String msg;
//         // std::stringstream ss;
//         // ss << "pos 1 clicked!";
//         // msg.data = ss.str();
//         // ROS_INFO("%s", msg.data.c_str());

//         // arm_pos.publish(msg);
//         // ros::spinOnce();
//     }

//     //Display info callbacks

//     // void
//     // elec_box_temp_callback(const std_msgs::Float64::ConstPtr& msg) {
//     //     elec_box_tempurature = msg->data;
//     //     if(ui_debug){
//     //     ROS_INFO("elec_box_tempurature update data, %f", elec_box_tempurature);
//     //     }
   
//     // }

//     // void velocity_status_callback(const std_msgs::Int16::ConstPtr& msg){
//     //        // chassis_velocity = msg->data;
//     // }

//     // void arm_status_callback(const std_msgs::Int16::ConstPtr& msg){
//     //         arm_status = msg->data;
//     // }

//     // void rover_status_callback(const std_msgs::Int16::ConstPtr& msg){
//     //         rover_status = msg->data;
//     // }

//     // void gnss_status_callback(const std_msgs::Int16::ConstPtr& msg){
//     //         gnss_status = msg->data;
//     // }





//   public Q_SLOTS:
//     void update_UI();
//     //void handleButton1();
//     // void handleButton2();
//     // void handleButton3();
//     // void handleButton4();
//     // void handleButton5();
//     // void handleButton6();

//     /*/private slots: Note if you want to create function from mainwindow.ui
//      * delete
//      * private slots and put them with public QSLOTS
//      * */

//   private:
//     Ui::MainWindow* ui;
//     // RosIntegration* ros_f;
//     QTimer* timer;
//     // ros::NodeHandle n;

//     // //ros::Rate loop_rate(10);

//     // // twist topics

//     // ros::Subscriber twist_controller_sub;

//     // ros::Publisher arm_pos;

    
    
//     // ros::Subscriber sub;

// };

// #endif // MAINWINDOW_H