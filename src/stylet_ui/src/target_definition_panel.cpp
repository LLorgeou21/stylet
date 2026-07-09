#include "stylet_ui/target_definition_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QProgressBar>
#include <QTimer>

#include "rviz_common/display_context.hpp"
#include "rviz_common/tool_manager.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace stylet_ui
{

TargetDefinitionPanel::TargetDefinitionPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);

  set_entry_point_button_ = new QPushButton("Set entry point", this);
  set_entry_point_button_->setCheckable(true);
  launch_button_ = new QPushButton("Launch operation", this);
  progress_bar_ = new QProgressBar(this);
  progress_bar_->setRange(0, 100);
  progress_bar_->setValue(0);

  needle_tcp_label_ = new QLabel("needle_tcp: -", this);
  entry_point_label_ = new QLabel("Entry point: -", this);
  target_point_label_ = new QLabel("Target: -", this);
  // Current named step (ExecuteProcedure action feedback - text such as
  // "Moving through the 'up' position", "Insertion in progress", etc., see
  // the step strings in procedure_planner.cpp) and final result
  // (success+metrics, or the precise failure reason - previously only
  // logged server-side, invisible from the panel).
  current_step_label_ = new QLabel("Step: -", this);
  result_label_ = new QLabel("Result: -", this);
  result_label_->setWordWrap(true);

  layout->addWidget(set_entry_point_button_);
  layout->addWidget(launch_button_);
  layout->addWidget(progress_bar_);
  layout->addWidget(current_step_label_);
  layout->addWidget(result_label_);
  layout->addWidget(needle_tcp_label_);
  layout->addWidget(entry_point_label_);
  layout->addWidget(target_point_label_);

  setLayout(layout);

  connect(set_entry_point_button_, &QPushButton::clicked,
    this, &TargetDefinitionPanel::onSetEntryPointClicked);
  connect(launch_button_, &QPushButton::clicked,
    this, &TargetDefinitionPanel::onLaunchClicked);

  tf_poll_timer_ = new QTimer(this);
  connect(tf_poll_timer_, &QTimer::timeout,
    this, &TargetDefinitionPanel::updateNeedleTcpDisplay);
}

TargetDefinitionPanel::~TargetDefinitionPanel() = default;

void TargetDefinitionPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Plain Marker (not InteractiveMarker): this point is never dragged with
  // the mouse, just displayed/colored. An InteractiveMarkerServer hosted by
  // this panel (i.e. inside RViz's own PROCESS, via its shared node) exposes
  // a get_interactive_markers service - if RViz's "InteractiveMarkers"
  // display calls it synchronously when the topic is selected, the thread
  // waiting for the reply is the same one that should be running it:
  // self-deadlock (RViz's topic selector spun forever for this topic
  // specifically, never for other display types that just do a plain
  // subscription). A plain Marker published on a topic (transient_local QoS
  // so a display added afterward still gets the latest state, same pattern
  // as target_point_pub_ in procedure_planner) needs no synchronous
  // round-trip at all.
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
    "/stylet_ui/entry_point_marker", rclcpp::QoS(1).transient_local());

  entry_point_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
    "/procedure/entry_point", 10,
    std::bind(&TargetDefinitionPanel::entryPointCallback, this, std::placeholders::_1));
  // transient_local QoS (not the default 10/volatile) - must match
  // target_point_pub_/system_state_pub_ on the procedure_planner side. The
  // target is only ever published ONCE (the timer is then cancelled for
  // good) - a volatile subscriber that connects afterward (the common case
  // with full_demo.launch.py's staggered startup) never catches that
  // message, even though the publisher keeps it around for late subscribers.
  target_point_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
    "/procedure/target_point", rclcpp::QoS(1).transient_local(),
    std::bind(&TargetDefinitionPanel::targetPointCallback, this, std::placeholders::_1));
  system_state_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/stylet/system/state", rclcpp::QoS(1).transient_local(),
    std::bind(&TargetDefinitionPanel::systemStateCallback, this, std::placeholders::_1));
  progress_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    "/procedure/progress", 10,
    std::bind(&TargetDefinitionPanel::progressCallback, this, std::placeholders::_1));

  // Registers EntryPointTool in RViz's tool list (visible in the toolbar at
  // the top), so it can be activated programmatically from the button.
  entry_point_tool_ = getDisplayContext()->getToolManager()->addTool("stylet_ui/EntryPointTool");

  execute_procedure_client_ = rclcpp_action::create_client<stylet_msgs::action::ExecuteProcedure>(
    node_, "execute_procedure");

  tf_poll_timer_->start(200);  // refreshes needle_tcp 5x/s
}

void TargetDefinitionPanel::onSetEntryPointClicked()
{
  if (entry_point_tool_ == nullptr)
  {
    return;
  }

  if (set_entry_point_button_->isChecked())
  {
    getDisplayContext()->getToolManager()->setCurrentTool(entry_point_tool_);
  }
  else
  {
    // Switch back to the default tool ("Move Camera") to hand control back
    // to normal navigation - without this, there's no way to "exit" picking
    // mode.
    getDisplayContext()->getToolManager()->setCurrentTool(
      getDisplayContext()->getToolManager()->getDefaultTool());
  }
}

void TargetDefinitionPanel::onLaunchClicked()
{
  using ExecuteProcedure = stylet_msgs::action::ExecuteProcedure;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteProcedure>;

  // action_server_is_ready() (non-blocking) rather than
  // wait_for_action_server() - this panel runs on RViz's GUI thread, any
  // blocking call here freezes the window (same lesson as the
  // InteractiveMarkerServer deadlock).
  if (!execute_procedure_client_->action_server_is_ready())
  {
    RCLCPP_WARN(node_->get_logger(),
      "Launch operation: /execute_procedure unavailable (is procedure_planner running?)");
    return;
  }

  current_step_label_->setText("Step: (starting...)");
  result_label_->setText("Result: -");
  result_label_->setStyleSheet("");

  auto goal = ExecuteProcedure::Goal();

  rclcpp_action::Client<ExecuteProcedure>::SendGoalOptions options;
  options.goal_response_callback =
    [this](const GoalHandle::SharedPtr & handle)
    {
      if (!handle)
      {
        RCLCPP_WARN(node_->get_logger(),
          "Launch operation: goal rejected (entry and/or target point not ready yet, or "
          "a procedure is already in progress)");
        current_step_label_->setText("Step: -");
        result_label_->setText("Result: goal rejected (see procedure_planner logs)");
        result_label_->setStyleSheet("color: red;");
      }
    };
  // Current named step, live from THIS goal's own feedback (finer-grained
  // than /stylet/system/state, which only knows a few broad states).
  options.feedback_callback =
    [this](GoalHandle::SharedPtr, const std::shared_ptr<const ExecuteProcedure::Feedback> feedback)
    {
      current_step_label_->setText(QString("Step: %1 (%2%)")
        .arg(QString::fromStdString(feedback->current_step))
        .arg(static_cast<int>(feedback->progress * 100.0f)));
    };
  options.result_callback =
    [this](const GoalHandle::WrappedResult & result)
    {
      RCLCPP_INFO(node_->get_logger(), "Launch operation: finished (%s) - %s",
        result.result->success ? "success" : "failure", result.result->message.c_str());
      result_label_->setText(QString("Result: %1")
        .arg(QString::fromStdString(result.result->message)));
      result_label_->setStyleSheet(result.result->success ? "color: green;" : "color: red;");
    };

  execute_procedure_client_->async_send_goal(goal, options);
}

void TargetDefinitionPanel::updateNeedleTcpDisplay()
{
  try
  {
    geometry_msgs::msg::TransformStamped transform =
      tf_buffer_->lookupTransform("world", "needle_tcp", tf2::TimePointZero);
    needle_tcp_label_->setText(QString("needle_tcp: %1, %2, %3")
      .arg(transform.transform.translation.x, 0, 'f', 4)
      .arg(transform.transform.translation.y, 0, 'f', 4)
      .arg(transform.transform.translation.z, 0, 'f', 4));
  }
  catch (const tf2::TransformException &)
  {
    needle_tcp_label_->setText("needle_tcp: (TF unavailable)");
  }
}

void TargetDefinitionPanel::entryPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  last_entry_point_ = *msg;
  has_entry_point_ = true;
  entry_point_label_->setText(QString("Entry point: %1, %2, %3")
    .arg(msg->point.x, 0, 'f', 4)
    .arg(msg->point.y, 0, 'f', 4)
    .arg(msg->point.z, 0, 'f', 4));
  updateMarker();
}

void TargetDefinitionPanel::targetPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  target_point_label_->setText(QString("Target: %1, %2, %3")
    .arg(msg->point.x, 0, 'f', 4)
    .arg(msg->point.y, 0, 'f', 4)
    .arg(msg->point.z, 0, 'f', 4));
}

void TargetDefinitionPanel::systemStateCallback(const std_msgs::msg::String::SharedPtr msg)
{
  current_state_ = msg->data;
  updateMarker();
}

void TargetDefinitionPanel::progressCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  progress_bar_->setValue(static_cast<int>(msg->data * 100.0f));
}

void TargetDefinitionPanel::colorForState(
  const std::string & state, float & r, float & g, float & b) const
{
  if (state == "APPROACHING" || state == "WAITING_FOR_PHASE" || state == "INSERTING")
  {
    r = 1.0f; g = 0.55f; b = 0.0f;  // orange
    return;
  }
  if (state == "COMPLETED")
  {
    r = 0.0f; g = 1.0f; b = 0.0f;  // green
    return;
  }
  if (state == "ERROR")
  {
    r = 0.0f; g = 0.0f; b = 0.0f;  // black
    return;
  }
  r = 1.0f; g = 0.0f; b = 0.0f;  // red by default (READY / unknown state)
}

void TargetDefinitionPanel::updateMarker()
{
  if (!has_entry_point_)
  {
    return;
  }

  float r, g, b;
  colorForState(current_state_, r, g, b);

  visualization_msgs::msg::Marker sphere;
  sphere.header.frame_id = last_entry_point_.header.frame_id;
  sphere.header.stamp = node_->now();
  sphere.ns = "entry_point";
  sphere.id = 0;
  sphere.type = visualization_msgs::msg::Marker::SPHERE;
  sphere.action = visualization_msgs::msg::Marker::ADD;
  sphere.pose.position = last_entry_point_.point;
  sphere.pose.orientation.w = 1.0;
  sphere.scale.x = 0.01;
  sphere.scale.y = 0.01;
  sphere.scale.z = 0.01;
  sphere.color.r = r;
  sphere.color.g = g;
  sphere.color.b = b;
  sphere.color.a = 1.0;

  marker_pub_->publish(sphere);
}

}  // namespace stylet_ui

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(stylet_ui::TargetDefinitionPanel, rviz_common::Panel)
