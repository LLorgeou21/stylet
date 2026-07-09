#ifndef STYLET_UI__TARGET_DEFINITION_PANEL_HPP_
#define STYLET_UI__TARGET_DEFINITION_PANEL_HPP_

#include "rviz_common/panel.hpp"
#include "rviz_common/tool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "stylet_msgs/action/execute_procedure.hpp"

class QPushButton;
class QLabel;
class QProgressBar;
class QTimer;

namespace stylet_ui
{

// RViz panel: entry-point definition (via EntryPointTool, decoupled by
// topic), coordinate display (needle_tcp, entry, target) and procedure-state
// tracking (/stylet/system/state, ADR-012) to color the entry marker. The
// target point is fixed (a procedure_planner parameter) - this panel only
// displays it, never defines it.
class TargetDefinitionPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit TargetDefinitionPanel(QWidget * parent = nullptr);
  ~TargetDefinitionPanel() override;

  void onInitialize() override;

private Q_SLOTS:
  void onSetEntryPointClicked();
  void onLaunchClicked();
  void updateNeedleTcpDisplay();

private:
  void entryPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void targetPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void systemStateCallback(const std_msgs::msg::String::SharedPtr msg);
  void progressCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void updateMarker();
  void colorForState(const std::string & state, float & r, float & g, float & b) const;

  QPushButton * set_entry_point_button_;
  QPushButton * launch_button_;
  QProgressBar * progress_bar_;
  QLabel * needle_tcp_label_;
  QLabel * entry_point_label_;
  QLabel * target_point_label_;
  QLabel * current_step_label_;
  QLabel * result_label_;

  QTimer * tf_poll_timer_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr entry_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_point_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr progress_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp_action::Client<stylet_msgs::action::ExecuteProcedure>::SharedPtr execute_procedure_client_;
  rviz_common::Tool * entry_point_tool_ = nullptr;

  geometry_msgs::msg::PointStamped last_entry_point_;
  bool has_entry_point_ = false;
  std::string current_state_ = "READY";
};

}  // namespace stylet_ui

#endif  // STYLET_UI__TARGET_DEFINITION_PANEL_HPP_
