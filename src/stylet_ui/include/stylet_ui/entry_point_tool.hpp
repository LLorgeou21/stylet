#ifndef STYLET_UI__ENTRY_POINT_TOOL_HPP_
#define STYLET_UI__ENTRY_POINT_TOOL_HPP_

#include "rviz_common/tool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace stylet_ui
{

// RViz tool: left click in the 3D view -> 3D point under the cursor (via
// RViz's SelectionManager, same mechanism as the native "Publish Point" tool)
// -> transformed into target_frame -> published on /procedure/entry_point.
// Handles no display itself (see TargetDefinitionPanel for that), decoupled
// via a ROS topic like RViz's own native tools.
class EntryPointTool : public rviz_common::Tool
{
  Q_OBJECT
public:
  EntryPointTool();
  ~EntryPointTool() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr entry_point_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace stylet_ui

#endif  // STYLET_UI__ENTRY_POINT_TOOL_HPP_
