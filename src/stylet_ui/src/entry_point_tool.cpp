#include "stylet_ui/entry_point_tool.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/interaction/view_picker_iface.hpp"
#include "rviz_common/view_manager.hpp"
#include "rviz_common/view_controller.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <OgreVector.h>

namespace stylet_ui
{

EntryPointTool::EntryPointTool()
{
  shortcut_key_ = 'e';
}

EntryPointTool::~EntryPointTool() = default;

void EntryPointTool::onInitialize()
{
  setName("Entry point");

  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  entry_point_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>(
    "/procedure/entry_point", 10);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void EntryPointTool::activate() {}
void EntryPointTool::deactivate() {}

int EntryPointTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  // Let the camera react normally (orbit/pan/zoom) even while this tool is
  // active - without this, no other mouse event is processed as long as the
  // "Entry point" tool stays selected (same behavior as the native "Move
  // Camera" tool, which most picking tools mimic).
  context_->getViewManager()->getCurrent()->handleMouseEvent(event);

  if (!event.leftDown())
  {
    return Render;
  }

  Ogre::Vector3 position;
  bool success = context_->getViewPicker()->get3DPoint(
    event.panel, event.x, event.y, position);

  if (!success)
  {
    RCLCPP_WARN(node_->get_logger(),
      "EntryPointTool: no 3D point under the cursor (nothing rendered there)");
    return Render;
  }

  geometry_msgs::msg::PointStamped point_fixed_frame;
  point_fixed_frame.header.frame_id = context_->getFixedFrame().toStdString();
  // stamp left at zero (default): TF2 convention for "latest available
  // transform", avoids clock issues between sim time (Gazebo) and the wall
  // clock (rviz2 without use_sim_time) across nodes that don't share the
  // same time source.
  point_fixed_frame.point.x = position.x;
  point_fixed_frame.point.y = position.y;
  point_fixed_frame.point.z = position.z;

  try
  {
    geometry_msgs::msg::PointStamped point_target_frame =
      tf_buffer_->transform(point_fixed_frame, "target_frame", tf2::durationFromSec(0.1));
    entry_point_pub_->publish(point_target_frame);
  }
  catch (const tf2::TransformException & ex)
  {
    RCLCPP_WARN(node_->get_logger(),
      "EntryPointTool: could not transform into target_frame: %s", ex.what());
  }

  return Render;
}

}  // namespace stylet_ui

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(stylet_ui::EntryPointTool, rviz_common::Tool)
