
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
  void pb_diskClicked();
  void pb_updateClicked();
  void pb_startClicked();
  void pb_stopClicked();

private:
  ros::NodeHandle nh_;
  std::string recorder_name_;

protected:
  Ui::PluginUI* ui_;
};
}
