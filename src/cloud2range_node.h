#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace cloud2range {

class Cloud2RangeNode {
 public:
  explicit Cloud2RangeNode(const ros::NodeHandle& pnh);

  void CloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

 private:
  ros::NodeHandle pnh_;
  ros::Subscriber sub_cloud_;
  ros::Publisher pub_image_;

  size_t n_beams_;
  size_t rpm_;
  double min_angle_, max_angle_;
  double max_range_;
};

}  // namespace cloud2range
