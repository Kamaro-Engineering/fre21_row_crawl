#pragma once

#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <fre_row_navigation/CrawlRowAction.h>
#include <fre_row_navigation/CrawlRowParamConfig.h>
#include <fre_row_navigation/cone.h>
#include <fre_row_navigation/pid.h>
#include <fre_row_navigation/rectangle.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

class CrawlRowAction {
public:
  CrawlRowAction();

  void goalCb();
  void preemptCb();

  void lidarCb(const sensor_msgs::LaserScan::ConstPtr& msg);
  void cloudCb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);
  void odomCb(const nav_msgs::Odometry::ConstPtr& msg);

  /**
   * @brief call called when updating dynamic params
   **/
  void dynamicParamCb(const fre_row_navigation::CrawlRowParamConfig& config,
                      uint32_t level);

protected:
  void processLidar(const sensor_msgs::LaserScan& scan);

  ros::NodeHandle nh;
  ros::NodeHandle ph;
  actionlib::SimpleActionServer<fre_row_navigation::CrawlRowAction> as;

  ros::Subscriber lidar_front_sub;
  ros::Subscriber cloud_front_sub;
  ros::Subscriber odom_sub;
  ros::Publisher viz_pub;
  ros::Publisher drive_pub;

  dynamic_reconfigure::Server<fre_row_navigation::CrawlRowParamConfig>
      dyn_param_server;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudFront;
  fre_row_navigation::CrawlRowGoal goal;
  geometry_msgs::Point last_pos;
  bool got_initial_pos;

  int end_line_frame_count;
  float speed;
  float start_travelled_distance;
  float travelled_distance;

  float speed_low;
  float speed_high;
  float p_gain;
  float i_gain;
  float d_gain;

  /**
   * @brief the pid controller for the steering angle
   **/

  PID pidController;
  ros::Time start_time;

  // Params
  double ray_length;
  bool use_speed_control;
  int end_line_frame_count_threshold;

  /**
   * @brief if true, use three cones and select the best one
   **/
  bool multi_cone_enabled;

  /**
   * @brief minimal cone width in degrees
   *
   * If the cone width is smaller than this, then that cone is discarded
   **/
  double min_cone_width;

  /**
   * @brief maximal cone width in degrees
   *
   * If the best cones width is larger than this, drive straight
   **/
  double max_cone_width;

  /**
   * @brief the angle to consider when judging scan points as an obstacle (deg)
   **/
  double obstacle_angle;

  /**
   * @brief min density of the points in obstacle_angle to consider it as an
   *obstacle
   **/
  double min_obs_density;
};
