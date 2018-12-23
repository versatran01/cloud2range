#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace cloud2range {

class Range2CloudNode {
 public:
  explicit Range2CloudNode(const ros::NodeHandle& pnh);

  void ImageCb(const sensor_msgs::ImageConstPtr& image_msg);

 private:
  ros::NodeHandle pnh_;
  ros::Subscriber sub_image_;
  ros::Publisher pub_cloud_;

  size_t rpm_;
  double max_range_;
  double min_angle_, max_angle_;
};

}  // namespace cloud2range
