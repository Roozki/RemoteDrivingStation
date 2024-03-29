#include <QApplication>
#include <QTimer>
#include "mainwindow.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);  // Initializte ROS2
    QApplication app(argc, argv);  // Initialize Qt

    auto node = std::make_shared<ManageWindow>();
    node->init();
    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [node]() {
        rclcpp::spin_some(node);
    });
    timer.start(20); // Set the interval to 20 ms

    ManageWindow window;
    window.show();
    
    //rclcpp::spin(node);
    return app.exec();  // Start the Qt event loop
}