#include <rosbag_record_rviz_plugin.h>

#include <pluginlib/class_list_macros.h>
#include <Qt>
#include <QMessageBox>
#include <QListWidgetItem>
#include <QFileDialog>
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
  connect(ui_->pb_disk, SIGNAL(clicked()), this, SLOT(pb_diskClicked()));
  connect(ui_->pb_update, SIGNAL(clicked()), this, SLOT(pb_updateClicked()));
  connect(ui_->pb_start, SIGNAL(clicked()), this, SLOT(pb_startClicked()));
  connect(ui_->pb_stop, SIGNAL(clicked()), this, SLOT(pb_stopClicked()));

  ui_->pb_stop->setDisabled(false);
  ui_->pb_stop->setDisabled(true);
  ui_->lb_status->setText("Stopped");
  this->pb_updateClicked();

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

void RosbagRecordPlugin::pb_diskClicked()
{
  QString dir = QFileDialog::getExistingDirectory(this, "Open Directory", "/home", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
  ui_->le_disk->setText(dir);
}

void RosbagRecordPlugin::pb_updateClicked()
{
  std::string command = "rostopic list";
  ROS_INFO_STREAM(command);

  FILE *fp;
  fp = popen(command.c_str(), "r");
  if (fp == NULL) { return; }

  ui_->lw_topic->clear();
  QListWidgetItem *item = new QListWidgetItem("All", ui_->lw_topic);
  item->setCheckState(Qt::Unchecked);
	char buf[1024];
  while (fgets(buf, sizeof(buf), fp) != NULL) {
    char *p = strchr(buf, '\n');
    if (p) { *p = '\0'; }
    QListWidgetItem *item = new QListWidgetItem(buf, ui_->lw_topic);
    item->setCheckState(Qt::Unchecked);
	}
}

void RosbagRecordPlugin::pb_startClicked()
{
  std::string topics;
  for (int i = 0; i < ui_->lw_topic->count(); i++)
  {
    auto item = ui_->lw_topic->item(i);
    if (item->checkState() == Qt::Checked)
    {
      if (item->text() == "All")
      {
        topics = "-a";
        break;
      }
      topics += item->text().toStdString() + " ";
    }
  }

  if (topics == "")
  {
    QMessageBox msgBox(this);
    msgBox.setText(tr("No topic selected!"));
    msgBox.setWindowTitle(tr("Error"));
    msgBox.setStandardButtons(QMessageBox::Cancel);
    msgBox.setIcon(QMessageBox::Critical);
    msgBox.exec();
    return;
  }

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

  std::string disk = ui_->le_disk->text().toStdString();
  if (disk != "")
  {
    record_name = disk + "/" + record_name;
  }

  ROS_INFO_STREAM("record_name : " << record_name);
  recorder_name_ = "record_plugin_" + std::to_string(ros::Time::now().sec);

  std::string command = "rosbag record __name:=" + recorder_name_ + " " + topics + " -O " + record_name + " &";
  ROS_INFO_STREAM(command);
  system(command.c_str());

  std::string msg = "Recording " + record_name;
  ui_->lb_status->setText(msg.c_str());
}

void RosbagRecordPlugin::pb_stopClicked()
{
  QMessageBox msgBox(this);
  msgBox.setText(tr("Are you sure you want to stop recording?"));
  msgBox.setWindowTitle(tr("Record Stop"));
  msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::Cancel);
  msgBox.setDefaultButton(QMessageBox::Cancel);
  msgBox.setIcon(QMessageBox::Information);
  int res = msgBox.exec();

  if (res != QMessageBox::Yes) { return; }

  ui_->pb_start->setDisabled(false);
  ui_->le_name->setDisabled(false);
  ui_->pb_stop->setDisabled(true);

  std::string command = "rosnode kill " + recorder_name_;
  ROS_INFO_STREAM(command);
  system(command.c_str());

  ui_->lb_status->setText("Stopped");
}

}
PLUGINLIB_EXPORT_CLASS(rosbag_record_rviz_plugin::RosbagRecordPlugin, rviz::Panel)
