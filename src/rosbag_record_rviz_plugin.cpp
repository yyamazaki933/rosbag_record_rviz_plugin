#include <rosbag_record_rviz_plugin.h>

#include <pluginlib/class_list_macros.h>
#include <QMessageBox>
#include "ui_rosbag_record_rviz_plugin.h"
#include "ros/ros.h"

namespace rosbag_record_rviz_plugin
{
RosbagRecordPlugin::RosbagRecordPlugin(QWidget* parent) : Panel(parent), ui_(new Ui::PluginUI())
{
  ui_->setupUi(this);
}

RosbagRecordPlugin::~RosbagRecordPlugin() = default;

void RosbagRecordPlugin::onInitialize()
{
  connect(ui_->pb_start, SIGNAL(clicked()), this, SLOT(startClicked()));
  connect(ui_->pb_stop, SIGNAL(clicked()), this, SLOT(stopClicked()));

  ui_->pb_stop->setDisabled(false);
  ui_->pb_stop->setDisabled(true);
  ui_->lb_status->setText("Stopped");

  parentWidget()->setVisible(true);
}

void RosbagRecordPlugin::onEnable()
{
  show();
  parentWidget()->show();
}

void RosbagRecordPlugin::onDisable()
{
  hide();
  parentWidget()->hide();
}

void RosbagRecordPlugin::startClicked()
{
  ui_->pb_start->setDisabled(true);
  ui_->le_name->setDisabled(true);
  ui_->pb_stop->setDisabled(false);

  std::string record_name = ui_->le_name->text().toStdString();

  if (record_name == "")
  {
    time_t t = time(nullptr);
    const tm* localTime = localtime(&t);

    std::stringstream ss;
    ss << localTime->tm_year + 1900 << '-';
    ss << std::setw(2) << std::setfill('0') << localTime->tm_mon + 1 << '-';
    ss << std::setw(2) << std::setfill('0') << localTime->tm_mday << '-';
    ss << std::setw(2) << std::setfill('0') << localTime->tm_hour << '-';
    ss << std::setw(2) << std::setfill('0') << localTime->tm_min << '-';
    ss << std::setw(2) << std::setfill('0') << localTime->tm_sec;
    record_name = ss.str();
  }

  ROS_INFO_STREAM("record_name : " << record_name);

  std::string command = "rosbag record __name:=record_from_rviz /talker &";
  ROS_INFO_STREAM(command);
  system(command.c_str());

  sleep(2);
}

void RosbagRecordPlugin::stopClicked()
{
  QMessageBox msgBox(this);
  msgBox.setText(tr("Are you sure you want to stop recording?"));
  msgBox.setWindowTitle(tr("Stop Recording"));
  msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::Cancel);
  msgBox.setDefaultButton(QMessageBox::Cancel);
  msgBox.setIcon(QMessageBox::Warning);
  int res = msgBox.exec();

  if (res != QMessageBox::Yes) { return; }

  ui_->pb_start->setDisabled(false);
  ui_->le_name->setDisabled(false);
  ui_->pb_stop->setDisabled(true);

  std::string command = "rosnode kill /record_from_rviz";
  ROS_INFO_STREAM(command);
  system(command.c_str());

  return;
}

}
PLUGINLIB_EXPORT_CLASS(rosbag_record_rviz_plugin::RosbagRecordPlugin, rviz::Panel)
