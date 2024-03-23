#include "mainwindow.h"

ManageWindow::ManageWindow(QWidget *parent) : QMainWindow(parent), rclcpp::Node("rds_gui") {
    ui.setupUi(this);  // This sets up the GUI as designed in Qt Designer

    // Connect signals to slots here, for example:
    // connect(ui.startButton, &QPushButton::clicked, this, &MainWindow::onStartButtonClicked);
    
}


