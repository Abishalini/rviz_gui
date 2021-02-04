#include <QApplication>
#include <QMessageBox>

#include "myviz.h"

#include <memory>
#include <rclcpp/rclcpp.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("rviz_gui_main");

static void siginthandler(int) {
  QApplication::quit();
}

int main(int argc, char *argv[])
{
  // Initialize and Start ROS node
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rviz_gui");

  // Equivalent to ROS spin?
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() { executor.spin(); });

  RCLCPP_DEBUG_STREAM(LOGGER, "Finished initializing and spinning ROS node");

  // Create Qt Application
  QApplication app( argc, argv );

  // Load MyViz
  MyViz* myviz = new MyViz();
  myviz->setMinimumWidth(1090);
  myviz->setMinimumHeight(600);
  myviz->show();

  signal(SIGINT, siginthandler);

  // Wait here until Qt App is finished
  const int result = app.exec();

  // Shutdown ROS
  rclcpp::shutdown();

  return result;

}