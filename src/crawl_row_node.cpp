#include <fre_row_navigation/crawl_row_node.h>

template <typename V, typename S>
V lerp(V a, V b, S t) {
  return a + t * (b - a);
}

float ramp_up(float begin, float later, float t) {
  if (t > 1.0) {
    return later;
  } else if (t < 0.0) {
    return begin;
  } else {
    return later * t + begin * (1 - t);
  }
}

CrawlRowAction::CrawlRowAction()
    : ph("~"),
      as(nh, "crawl_row", false),
      tf_listener(tf_buffer),
      p_gain(0),
      i_gain(0),
      d_gain(0),
      pidController(p_gain, i_gain, d_gain),
      travelled_distance(0),
      got_initial_pos(false) {
  as.registerGoalCallback(std::bind(&CrawlRowAction::goalCb, this));
  as.registerPreemptCallback(std::bind(&CrawlRowAction::preemptCb, this));

  lidar_front_sub =
      nh.subscribe("lidar_front", 1, &CrawlRowAction::lidarCb, this);
  cloud_front_sub = nh.subscribe("lidar_front_cloud_filtered", 1,
                                 &CrawlRowAction::cloudCb, this);
  odom_sub = nh.subscribe("/odom", 1, &CrawlRowAction::odomCb, this);

  viz_pub = ph.advertise<visualization_msgs::Marker>("viz", 50);
  drive_pub = ph.advertise<geometry_msgs::Twist>("drive_command", 1);

  decltype(dyn_param_server)::CallbackType dyn_param_callback =
      boost::bind(&CrawlRowAction::dynamicParamCb, this, _1, _2);
  dyn_param_server.setCallback(dyn_param_callback);

  as.start();
  ROS_INFO("Started crawl row action server");
}

void CrawlRowAction::goalCb() {
  goal = *as.acceptNewGoal();
  ROS_INFO("Got new goal");
  end_line_frame_count = 0;
  speed = 0.;
  start_travelled_distance = travelled_distance;
  start_time = ros::Time::now();
}

void CrawlRowAction::preemptCb() {
  as.setPreempted();
  drive_pub.publish(geometry_msgs::Twist());  // Stop
}

void CrawlRowAction::processLidar(const sensor_msgs::LaserScan& scan) {
  //
  // Find a cone in the laser scans
  //
  double obstacle_angle_rad = this->obstacle_angle / 180. * M_PI;
  Cone middleCone = Cone::findCone(scan, 0, ray_length, obstacle_angle_rad,
                                   this->min_obs_density);

  Cone bestCone = middleCone;
  if (this->multi_cone_enabled &&
      (middleCone.width() < this->min_cone_width / 180 * M_PI)) {
    Cone leftCone = Cone::findCone(scan, 0.4, ray_length, obstacle_angle_rad,
                                   this->min_obs_density);
    Cone rightCone = Cone::findCone(scan, -0.4, ray_length, obstacle_angle_rad,
                                    this->min_obs_density);

    if (middleCone.width() >= leftCone.width() &&
        middleCone.width() >= rightCone.width()) {
      bestCone = middleCone;
    } else if (leftCone.width() >= rightCone.width()) {
      bestCone = leftCone;
    } else {
      bestCone = rightCone;
    }
  }

  // If middle was bad but left and right were bad too just go forward
  if (bestCone.width() < this->min_cone_width / 180 * M_PI) {
    bestCone.startAngle = 0.0;
    bestCone.endAngle = 0.0;
  }

  // If cone is to big, this is probably a hole in the row
  if (bestCone.width() >= this->max_cone_width / 180 * M_PI) {
    bestCone.startAngle = 0.0;
    bestCone.endAngle = 0.0;
  }

  viz_pub.publish(bestCone.createMarker(1));

  // Get cone transform
  geometry_msgs::TransformStamped cone_transform_msg;
  try {
    cone_transform_msg =
        tf_buffer.lookupTransform("base_link", bestCone.frame_id, ros::Time(0));
  } catch (tf2::TransformException& ex) {
    ROS_ERROR_STREAM("Could not lookup lidar transform: " << ex.what());
    return;
  }
  tf::StampedTransform cone_transform;
  tf::transformStampedMsgToTF(cone_transform_msg, cone_transform);

  tf::Point cone_point_in_base_link =
      cone_transform * bestCone.getMiddleLinePoint();

  visualization_msgs::Marker pm;
  pm.header.frame_id = "base_link";
  pm.ns = "cone";
  pm.id = 123;
  pm.type = visualization_msgs::Marker::POINTS;
  pm.action = visualization_msgs::Marker::ADD;
  pm.scale.x = 0.2;
  pm.scale.y = 0.2;
  pm.scale.z = 0.2;
  pm.color.r = 1;
  pm.color.a = 1;
  geometry_msgs::Point p;
  tf::pointTFToMsg(cone_point_in_base_link, p);
  pm.points.push_back(p);
  viz_pub.publish(pm);

  //
  // Use resizing rectangles for correction
  //
  if (cloudFront) {
    float x1 = 0.4;
    float x2 = 0.7;

    Rectangle rect_left =
        Rectangle::grow_y(x1, x2, 0.1, 0.02, 0.5, *cloudFront, 8);
    viz_pub.publish(rect_left.createMarker(1, "base_link"));
    float yLeft = std::abs(rect_left.y2);

    Rectangle rect_right =
        Rectangle::grow_y(x1, x2, -0.1, -0.02, -0.5, *cloudFront, 8);
    viz_pub.publish(rect_right.createMarker(2, "base_link"));
    float yRight = std::abs(rect_right.y2);

    yLeft = -std::max(-yLeft + 0.35, 0.0);
    yRight = std::max(-yRight + 0.35, 0.0);

    double sum = yLeft + yRight;
    double lineShift = sum * 0.5;

    //
    // Detect end of row
    //
    Rectangle end_of_row_far_rect;
    end_of_row_far_rect.x1 = 0.8;
    end_of_row_far_rect.x2 = 1.0;
    end_of_row_far_rect.y1 = -0.6;
    end_of_row_far_rect.y2 = 0.6;
    visualization_msgs::Marker end_of_row_far_marker =
        end_of_row_far_rect.createMarker(4, "base_link");
    end_of_row_far_marker.color.a = 0.3;
    bool far_end_of_row =
        end_of_row_far_rect.getNumPointsInsideRectangle(*cloudFront) < 3;
    if (far_end_of_row) {
      end_of_row_far_marker.color.r = 1;
    }

    Rectangle end_of_row_rect;
    end_of_row_rect.x1 = 0.4;
    end_of_row_rect.x2 = 0.8;
    end_of_row_rect.y1 = -0.6;
    end_of_row_rect.y2 = 0.6;
    visualization_msgs::Marker end_of_row_marker =
        end_of_row_rect.createMarker(3, "base_link");
    end_of_row_marker.color.a = 0.3;
    bool near_end_of_row =
        end_of_row_rect.getNumPointsInsideRectangle(*cloudFront) < 3;
    if (near_end_of_row) {
      end_of_row_marker.color.r = 1;
    }

    bool n_and_f_eor = near_end_of_row && far_end_of_row;
    if (n_and_f_eor) {
      end_line_frame_count++;
    } else {
      end_line_frame_count--;
      if (end_line_frame_count < 0) end_line_frame_count = 0;
    }

    float travelled = std::abs(travelled_distance - start_travelled_distance);
    bool travelled_enough = travelled > 1.0f;
    bool end_of_row = end_line_frame_count >= end_line_frame_count_threshold &&
                      travelled_enough;
    if (end_of_row) {
      end_of_row_marker.color.g = 0;
    }
    viz_pub.publish(end_of_row_marker);
    viz_pub.publish(end_of_row_far_marker);

    if (end_of_row) {
      // Stop
      geometry_msgs::Twist driveCommand;
      drive_pub.publish(driveCommand);
      as.setSucceeded();
      return;
    }

    //
    // Speed control
    //
    if (use_speed_control) {
      float sreclenmin = 0.3;
      float sreclenmax = 2.0;
      float sx1 = sreclenmin;
      float sx2 = sreclenmax;
      float growth = 0.05;

      Rectangle rect_drive_dir =
          Rectangle::grow_x(sx1, 0.2, -0.2, growth, sx2, *cloudFront, 10);
      visualization_msgs::Marker marker =
          rect_drive_dir.createMarker(1287, "base_link");
      marker.color.r = 1;
      marker.color.a = 0.3;
      viz_pub.publish(marker);
      float speed_rect_len = std::abs(rect_drive_dir.x2 - rect_drive_dir.x1);
      float calc_speed =
          lerp(this->speed_low, this->speed_high,
               (speed_rect_len - sreclenmin) / (sreclenmax - sreclenmin));
      // Slow down so we dont overshoot end of row
      if (far_end_of_row) {
        calc_speed = this->speed_low;
      }
      // smooth
      speed = speed * 0.90 + calc_speed * 0.1;
    } else {
      speed = 0.25;
    }

    pidController.p =
        ramp_up(p_gain * 2, p_gain,
                (travelled_distance - start_travelled_distance) / 0.2);
    pidController.i = i_gain;
    pidController.d = d_gain;

    double atan_dir =
        std::atan2(cone_point_in_base_link.y(), cone_point_in_base_link.x());
    double driveAngle = pidController.calculate(atan_dir + lineShift);

    // Viz lineShift
    visualization_msgs::Marker offset_marker;
    offset_marker.header.frame_id = "base_link";
    offset_marker.ns = "cone";
    offset_marker.id = 127;
    offset_marker.type = visualization_msgs::Marker::POINTS;
    offset_marker.action = visualization_msgs::Marker::ADD;
    offset_marker.scale.x = 0.2;
    offset_marker.scale.y = 0.2;
    offset_marker.scale.z = 0.2;
    offset_marker.color.b = 1;
    offset_marker.color.a = 1;
    geometry_msgs::Point offset_point;
    offset_point.x = 1.5;
    offset_point.y = lineShift / 5;
    offset_marker.points.push_back(offset_point);
    viz_pub.publish(offset_marker);

    // Viz steering
    visualization_msgs::Marker steering_marker;
    steering_marker.header.frame_id = "base_link";
    steering_marker.ns = "cone";
    steering_marker.id = 128;
    steering_marker.type = visualization_msgs::Marker::POINTS;
    steering_marker.action = visualization_msgs::Marker::ADD;
    steering_marker.scale.x = 0.2;
    steering_marker.scale.y = 0.2;
    steering_marker.scale.z = 0.2;
    steering_marker.color.g = 1;
    steering_marker.color.a = 1;
    geometry_msgs::Point steering_point;
    steering_point.x = 1;
    steering_point.y = driveAngle / 5;
    steering_marker.points.push_back(steering_point);
    viz_pub.publish(steering_marker);

    if (far_end_of_row) {
      geometry_msgs::Twist driveCommand;
      driveCommand.linear.x = speed;
      driveCommand.angular.z = 0;
      drive_pub.publish(driveCommand);
    } else {
      geometry_msgs::Twist driveCommand;
      driveCommand.linear.x = speed;
      driveCommand.angular.z = driveAngle;
      drive_pub.publish(driveCommand);
    }
  } else {
    ROS_ERROR("No cloud");
    geometry_msgs::Twist driveCommand;
    drive_pub.publish(driveCommand);
  }
}

void CrawlRowAction::lidarCb(const sensor_msgs::LaserScan::ConstPtr& msg) {
  if (as.isActive()) {
    processLidar(*msg);
  }
}

void CrawlRowAction::cloudCb(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg) {
  cloudFront = msg;
}

void CrawlRowAction::odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
  if (got_initial_pos) {
    double dx = last_pos.x - msg->pose.pose.position.x;
    double dy = last_pos.y - msg->pose.pose.position.y;
    double d = std::sqrt(dx * dx + dy * dy);
    travelled_distance += d;
  }
  last_pos = msg->pose.pose.position;
  got_initial_pos = true;
}

void CrawlRowAction::dynamicParamCb(
    const fre_row_navigation::CrawlRowParamConfig& config, uint32_t level) {
  this->use_speed_control = config.use_speed_control;
  this->end_line_frame_count_threshold = config.end_line_frame_count_threshold;
  this->p_gain = config.p_gain;
  this->i_gain = config.i_gain;
  this->d_gain = config.d_gain;
  this->ray_length = config.ray_length;
  this->min_cone_width = config.min_cone_width;
  this->max_cone_width = config.max_cone_width;
  this->multi_cone_enabled = config.multi_cone_enabled;
  this->obstacle_angle = config.obstacle_angle;
  this->min_obs_density = config.min_obs_density;
  this->speed_low = config.speed_low;
  this->speed_high = config.speed_high;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "crawl_row_node");

  ros::NodeHandle nh;
  CrawlRowAction crawlRowAction;

  ros::spin();

  return 0;
}
