/**
 * @file shared_control_visualization_node.cpp
 * @brief Publishes RViz markers for the shared-control debug state.
 *
 * This visualizes the same quantities as the shared-control article:
 * - end-effector position xE,
 * - goal positions xGi,
 * - direct vectors ui = (xGi - xE) / ||xGi - xE||,
 * - decision cones around ui with aperture theta_l,
 * - user velocity vector v,
 * - assisted velocity vector,
 * - soft/selected goal,
 * - r1/r2 spheres around each goal.
 */

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "visual_servoing/msg/shared_control_debug.hpp"
#include "visual_servoing/msg/shared_control_goal_debug.hpp"

class SharedControlVisualizationNode : public rclcpp::Node
{
public:
  SharedControlVisualizationNode()
      : Node("shared_control_visualization_node")
  {
    declareParameters();
    loadParameters();
    initializePublishers();
    initializeSubscribers();

    RCLCPP_INFO(get_logger(), "Shared control debug visualization node ready.");
  }

private:
  // -------------------------------------------------------------------------
  // Parameters
  // -------------------------------------------------------------------------

  double ee_sphere_diameter_ = 0.055;
  double goal_sphere_diameter_ = 0.060;
  double soft_goal_sphere_diameter_ = 0.080;
  double marker_lifetime_sec_ = 0.20;

  double label_height_ = 0.045;
  double label_z_offset_ = 0.080;

  double user_velocity_scale_ = 4.0;
  double assisted_velocity_scale_ = 4.0;
  double velocity_arrow_shaft_ = 0.012;
  double velocity_arrow_head_diameter_ = 0.035;
  double velocity_arrow_head_length_ = 0.050;

  double goal_arrow_shaft_ = 0.008;
  double goal_arrow_head_diameter_ = 0.025;
  double goal_arrow_head_length_ = 0.040;

  double cone_length_ = 0.35;
  int cone_segments_ = 32;
  double cone_line_width_ = 0.006;

  double radius_sphere_alpha_ = 0.12;

  // -------------------------------------------------------------------------
  // ROS interfaces
  // -------------------------------------------------------------------------

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<visual_servoing::msg::SharedControlDebug>::SharedPtr debug_sub_;

  // -------------------------------------------------------------------------
  // Initialization
  // -------------------------------------------------------------------------

  void declareParameters()
  {
    declare_parameter<double>("ee_sphere_diameter", ee_sphere_diameter_);
    declare_parameter<double>("goal_sphere_diameter", goal_sphere_diameter_);
    declare_parameter<double>("soft_goal_sphere_diameter", soft_goal_sphere_diameter_);
    declare_parameter<double>("marker_lifetime_sec", marker_lifetime_sec_);

    declare_parameter<double>("label_height", label_height_);
    declare_parameter<double>("label_z_offset", label_z_offset_);

    declare_parameter<double>("user_velocity_scale", user_velocity_scale_);
    declare_parameter<double>("assisted_velocity_scale", assisted_velocity_scale_);
    declare_parameter<double>("velocity_arrow_shaft", velocity_arrow_shaft_);
    declare_parameter<double>("velocity_arrow_head_diameter", velocity_arrow_head_diameter_);
    declare_parameter<double>("velocity_arrow_head_length", velocity_arrow_head_length_);

    declare_parameter<double>("goal_arrow_shaft", goal_arrow_shaft_);
    declare_parameter<double>("goal_arrow_head_diameter", goal_arrow_head_diameter_);
    declare_parameter<double>("goal_arrow_head_length", goal_arrow_head_length_);

    declare_parameter<double>("cone_length", cone_length_);
    declare_parameter<int>("cone_segments", cone_segments_);
    declare_parameter<double>("cone_line_width", cone_line_width_);

    declare_parameter<double>("radius_sphere_alpha", radius_sphere_alpha_);
  }

  void loadParameters()
  {
    ee_sphere_diameter_ = get_parameter("ee_sphere_diameter").as_double();
    goal_sphere_diameter_ = get_parameter("goal_sphere_diameter").as_double();
    soft_goal_sphere_diameter_ = get_parameter("soft_goal_sphere_diameter").as_double();
    marker_lifetime_sec_ = get_parameter("marker_lifetime_sec").as_double();

    label_height_ = get_parameter("label_height").as_double();
    label_z_offset_ = get_parameter("label_z_offset").as_double();

    user_velocity_scale_ = get_parameter("user_velocity_scale").as_double();
    assisted_velocity_scale_ = get_parameter("assisted_velocity_scale").as_double();
    velocity_arrow_shaft_ = get_parameter("velocity_arrow_shaft").as_double();
    velocity_arrow_head_diameter_ = get_parameter("velocity_arrow_head_diameter").as_double();
    velocity_arrow_head_length_ = get_parameter("velocity_arrow_head_length").as_double();

    goal_arrow_shaft_ = get_parameter("goal_arrow_shaft").as_double();
    goal_arrow_head_diameter_ = get_parameter("goal_arrow_head_diameter").as_double();
    goal_arrow_head_length_ = get_parameter("goal_arrow_head_length").as_double();

    cone_length_ = get_parameter("cone_length").as_double();
    cone_segments_ = get_parameter("cone_segments").as_int();
    cone_line_width_ = get_parameter("cone_line_width").as_double();

    radius_sphere_alpha_ = get_parameter("radius_sphere_alpha").as_double();

    cone_segments_ = std::max(8, cone_segments_);
  }

  void initializePublishers()
  {
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/shared_control/visualization_markers",
        10);
  }

  void initializeSubscribers()
  {
    debug_sub_ = create_subscription<visual_servoing::msg::SharedControlDebug>(
        "/shared_control/debug",
        10,
        std::bind(&SharedControlVisualizationNode::debugCallback, this, std::placeholders::_1));
  }

  // -------------------------------------------------------------------------
  // Callback
  // -------------------------------------------------------------------------

  void debugCallback(const visual_servoing::msg::SharedControlDebug::SharedPtr msg)
  {
    visualization_msgs::msg::MarkerArray markers;
    addClearMarker(markers, msg->header);

    int marker_id = 0;

    const auto &header = msg->header;
    const geometry_msgs::msg::Point xE = msg->end_effector_pose.pose.position;

    addSphere(markers, header, "end_effector", marker_id++, xE,
              ee_sphere_diameter_, 1.0, 1.0, 1.0, 1.0);

    addVelocityArrow(markers, header, "user_velocity", marker_id++, xE,
                     msg->user_linear_velocity, user_velocity_scale_,
                     1.0, 1.0, 0.0, 1.0);

    addVelocityArrow(markers, header, "assisted_velocity", marker_id++, xE,
                     msg->assisted_linear_velocity, assisted_velocity_scale_,
                     0.0, 1.0, 1.0, 1.0);

    addSphere(markers, header, "soft_goal", marker_id++, msg->soft_goal_pose.pose.position,
              soft_goal_sphere_diameter_, 1.0, 0.0, 1.0, 0.90);

    addText(markers, header, "soft_goal_label", marker_id++, msg->soft_goal_pose.pose.position,
            "soft goal", 1.0, 0.0, 1.0, 1.0);

    for (const auto &goal : msg->goals)
    {
      const auto &xG = goal.pose.position;

      const bool inside_cone = goal.angle_rad <= msg->theta_l_rad;
      const bool selected = goal.selected;

      double r = 0.0;
      double g = 0.8;
      double b = 0.0;
      double a = 0.85;

      if (selected)
      {
        r = 0.0;
        g = 1.0;
        b = 0.0;
        a = 1.0;
      }
      else if (inside_cone)
      {
        r = 1.0;
        g = 0.65;
        b = 0.0;
        a = 0.95;
      }
      else
      {
        r = 0.4;
        g = 0.4;
        b = 0.4;
        a = 0.65;
      }

      addSphere(markers, header, "goals", marker_id++, xG,
                goal_sphere_diameter_, r, g, b, a);

      addArrowBetweenPoints(markers, header, "ui_vectors", marker_id++, xE, xG,
                            goal_arrow_shaft_, goal_arrow_head_diameter_, goal_arrow_head_length_,
                            r, g, b, 0.85);

      addDecisionCone(markers, header, "decision_cones", marker_id++, xE, xG,
                      msg->theta_l_rad, r, g, b, 0.85);

      addRadiusSphere(markers, header, "r1_spheres", marker_id++, xG, msg->r1,
                      1.0, 0.65, 0.0, radius_sphere_alpha_);

      addRadiusSphere(markers, header, "r2_spheres", marker_id++, xG, msg->r2,
                      1.0, 0.0, 0.0, radius_sphere_alpha_);

      addText(markers, header, "goal_labels", marker_id++, xG,
              makeGoalLabel(goal, msg->theta_l_rad), r, g, b, 1.0);
    }

    marker_pub_->publish(markers);
  }

  // -------------------------------------------------------------------------
  // Marker helpers
  // -------------------------------------------------------------------------

  void addClearMarker(visualization_msgs::msg::MarkerArray &markers,
                      const std_msgs::msg::Header &header) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(marker);
  }

  visualization_msgs::msg::Marker makeMarker(const std_msgs::msg::Header &header,
                                             const std::string &ns,
                                             int id,
                                             int type) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);
    return marker;
  }

  static geometry_msgs::msg::Point makePoint(double x, double y, double z)
  {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  }

  static geometry_msgs::msg::Point addPointAndVector(const geometry_msgs::msg::Point &p,
                                                     const geometry_msgs::msg::Vector3 &v,
                                                     double scale)
  {
    return makePoint(p.x + scale * v.x, p.y + scale * v.y, p.z + scale * v.z);
  }

  static geometry_msgs::msg::Vector3 vectorBetween(const geometry_msgs::msg::Point &from,
                                                   const geometry_msgs::msg::Point &to)
  {
    geometry_msgs::msg::Vector3 v;
    v.x = to.x - from.x;
    v.y = to.y - from.y;
    v.z = to.z - from.z;
    return v;
  }

  static double norm(const geometry_msgs::msg::Vector3 &v)
  {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  }

  static geometry_msgs::msg::Vector3 normalized(const geometry_msgs::msg::Vector3 &v)
  {
    const double n = norm(v);
    if (n < 1e-9)
    {
      geometry_msgs::msg::Vector3 fallback;
      fallback.x = 1.0;
      fallback.y = 0.0;
      fallback.z = 0.0;
      return fallback;
    }

    geometry_msgs::msg::Vector3 out;
    out.x = v.x / n;
    out.y = v.y / n;
    out.z = v.z / n;
    return out;
  }

  static geometry_msgs::msg::Vector3 cross(const geometry_msgs::msg::Vector3 &a,
                                           const geometry_msgs::msg::Vector3 &b)
  {
    geometry_msgs::msg::Vector3 out;
    out.x = a.y * b.z - a.z * b.y;
    out.y = a.z * b.x - a.x * b.z;
    out.z = a.x * b.y - a.y * b.x;
    return out;
  }

  static geometry_msgs::msg::Vector3 scaledVector(const geometry_msgs::msg::Vector3 &v,
                                                  double scale)
  {
    geometry_msgs::msg::Vector3 out;
    out.x = scale * v.x;
    out.y = scale * v.y;
    out.z = scale * v.z;
    return out;
  }

  static geometry_msgs::msg::Vector3 addVectors(const geometry_msgs::msg::Vector3 &a,
                                                const geometry_msgs::msg::Vector3 &b)
  {
    geometry_msgs::msg::Vector3 out;
    out.x = a.x + b.x;
    out.y = a.y + b.y;
    out.z = a.z + b.z;
    return out;
  }

  void setColor(visualization_msgs::msg::Marker &marker,
                double r, double g, double b, double a) const
  {
    marker.color.r = static_cast<float>(r);
    marker.color.g = static_cast<float>(g);
    marker.color.b = static_cast<float>(b);
    marker.color.a = static_cast<float>(a);
  }

  void addSphere(visualization_msgs::msg::MarkerArray &markers,
                 const std_msgs::msg::Header &header,
                 const std::string &ns,
                 int id,
                 const geometry_msgs::msg::Point &position,
                 double diameter,
                 double r, double g, double b, double a) const
  {
    auto marker = makeMarker(header, ns, id, visualization_msgs::msg::Marker::SPHERE);
    marker.pose.position = position;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = diameter;
    marker.scale.y = diameter;
    marker.scale.z = diameter;
    setColor(marker, r, g, b, a);
    markers.markers.push_back(marker);
  }

  void addRadiusSphere(visualization_msgs::msg::MarkerArray &markers,
                       const std_msgs::msg::Header &header,
                       const std::string &ns,
                       int id,
                       const geometry_msgs::msg::Point &position,
                       double radius,
                       double r, double g, double b, double a) const
  {
    addSphere(markers, header, ns, id, position, 2.0 * radius, r, g, b, a);
  }

  void addText(visualization_msgs::msg::MarkerArray &markers,
               const std_msgs::msg::Header &header,
               const std::string &ns,
               int id,
               const geometry_msgs::msg::Point &position,
               const std::string &text,
               double r, double g, double b, double a) const
  {
    auto marker = makeMarker(header, ns, id, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    marker.pose.position = position;
    marker.pose.position.z += label_z_offset_;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = label_height_;
    marker.text = text;
    setColor(marker, r, g, b, a);
    markers.markers.push_back(marker);
  }

  void addVelocityArrow(visualization_msgs::msg::MarkerArray &markers,
                        const std_msgs::msg::Header &header,
                        const std::string &ns,
                        int id,
                        const geometry_msgs::msg::Point &origin,
                        const geometry_msgs::msg::Vector3 &velocity,
                        double scale,
                        double r, double g, double b, double a) const
  {
    if (norm(velocity) < 1e-6)
    {
      return;
    }

    const auto end = addPointAndVector(origin, velocity, scale);
    addArrowBetweenPoints(markers, header, ns, id, origin, end,
                          velocity_arrow_shaft_, velocity_arrow_head_diameter_,
                          velocity_arrow_head_length_, r, g, b, a);
  }

  void addArrowBetweenPoints(visualization_msgs::msg::MarkerArray &markers,
                             const std_msgs::msg::Header &header,
                             const std::string &ns,
                             int id,
                             const geometry_msgs::msg::Point &start,
                             const geometry_msgs::msg::Point &end,
                             double shaft_diameter,
                             double head_diameter,
                             double head_length,
                             double r, double g, double b, double a) const
  {
    if (norm(vectorBetween(start, end)) < 1e-6)
    {
      return;
    }

    auto marker = makeMarker(header, ns, id, visualization_msgs::msg::Marker::ARROW);
    marker.points.push_back(start);
    marker.points.push_back(end);
    marker.scale.x = shaft_diameter;
    marker.scale.y = head_diameter;
    marker.scale.z = head_length;
    setColor(marker, r, g, b, a);
    markers.markers.push_back(marker);
  }

  void addDecisionCone(visualization_msgs::msg::MarkerArray &markers,
                       const std_msgs::msg::Header &header,
                       const std::string &ns,
                       int id,
                       const geometry_msgs::msg::Point &apex,
                       const geometry_msgs::msg::Point &goal,
                       double theta_l_rad,
                       double r, double g, double b, double a) const
  {
    const auto axis_raw = vectorBetween(apex, goal);
    if (norm(axis_raw) < 1e-6)
    {
      return;
    }

    const auto axis = normalized(axis_raw);
    geometry_msgs::msg::Vector3 helper;
    helper.x = 0.0;
    helper.y = 0.0;
    helper.z = 1.0;

    if (std::abs(axis.z) > 0.95)
    {
      helper.x = 0.0;
      helper.y = 1.0;
      helper.z = 0.0;
    }

    const auto basis1 = normalized(cross(axis, helper));
    const auto basis2 = normalized(cross(axis, basis1));

    const double radius = cone_length_ * std::tan(theta_l_rad);
    const auto center_vec = scaledVector(axis, cone_length_);
    const auto base_center = makePoint(apex.x + center_vec.x,
                                       apex.y + center_vec.y,
                                       apex.z + center_vec.z);

    auto marker = makeMarker(header, ns, id, visualization_msgs::msg::Marker::LINE_LIST);
    marker.scale.x = cone_line_width_;
    setColor(marker, r, g, b, a);

    std::vector<geometry_msgs::msg::Point> circle_points;
    circle_points.reserve(static_cast<size_t>(cone_segments_));

    const double pi = 3.14159265358979323846;
    for (int i = 0; i < cone_segments_; ++i)
    {
      const double angle = 2.0 * pi * static_cast<double>(i) / static_cast<double>(cone_segments_);
      const auto radial = addVectors(scaledVector(basis1, radius * std::cos(angle)),
                                     scaledVector(basis2, radius * std::sin(angle)));
      circle_points.push_back(makePoint(base_center.x + radial.x,
                                        base_center.y + radial.y,
                                        base_center.z + radial.z));
    }

    for (int i = 0; i < cone_segments_; ++i)
    {
      const auto &p0 = circle_points[static_cast<size_t>(i)];
      const auto &p1 = circle_points[static_cast<size_t>((i + 1) % cone_segments_)];

      // Base circle segment.
      marker.points.push_back(p0);
      marker.points.push_back(p1);

      // Cone side segment.
      marker.points.push_back(apex);
      marker.points.push_back(p0);
    }

    markers.markers.push_back(marker);
  }

  std::string makeGoalLabel(const visual_servoing::msg::SharedControlGoalDebug &goal,
                            double theta_l_rad) const
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);

    if (goal.id >= 0)
    {
      oss << "tag " << goal.id << "\n";
    }
    else
    {
      oss << "goal\n";
    }

    oss << "c=" << goal.confidence << "  d=" << goal.distance << "m\n";
    oss << "angle=" << radToDeg(goal.angle_rad) << "deg";

    if (goal.angle_rad <= theta_l_rad)
    {
      oss << " IN";
    }
    else
    {
      oss << " OUT";
    }

    if (goal.selected)
    {
      oss << " SELECTED";
    }

    return oss.str();
  }

  static double radToDeg(double rad)
  {
    return rad * 180.0 / 3.14159265358979323846;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SharedControlVisualizationNode>());
  rclcpp::shutdown();
  return 0;
}