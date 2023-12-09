
#ifndef Q_MOC_RUN
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <chrono>
#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#endif

#include <rviz/panel.h>

namespace Ui
{
class PluginUI;
}

namespace rosbag_record_rviz_plugin
{
class RosbagRecordPlugin : public rviz::Panel
{
  Q_OBJECT
public:
  RosbagRecordPlugin(QWidget* parent = nullptr);
  ~RosbagRecordPlugin() override;

  void onInitialize() override;
  void onEnable();
  void onDisable();

private Q_SLOTS:
  void startClicked();
  void stopClicked();

private:
  ros::NodeHandle nh_;
  std::string start_script_;
  std::string stop_script_;
  std::string comment_dir_;
  std::string comment_file_;

protected:
  Ui::PluginUI* ui_;
};
}
