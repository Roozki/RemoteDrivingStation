#include <QApplication>
#include "mainwindow.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);  // Initializte ROS2
    QApplication app(argc, argv);  // Initialize Qt

    auto node = std::make_shared<ManageWindow>();
    node->init();
    ManageWindow window;
    window.show();
    app.exec();
    rclcpp::spin(node);
    return 0;  // Start the Qt event loop
}