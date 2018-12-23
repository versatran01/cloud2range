#include "utils.h"

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

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

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

Range2CloudNode::Range2CloudNode(const ros::NodeHandle& pnh) : pnh_(pnh) {
  sub_image_ =
      pnh_.subscribe("range/image", 1, &Range2CloudNode::ImageCb, this);
  pub_cloud_ = pnh_.advertise<CloudT>("cloud2", 1);

  max_range_ = 100;
  min_angle_ = Rad_Deg(-15);
  max_angle_ = Rad_Deg(15);
  rpm_ = 600;
}

void Range2CloudNode::ImageCb(const sensor_msgs::ImageConstPtr& image_msg) {
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(image_msg);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  const auto range_image = cv_ptr->image;
  ROS_INFO("range image %dx%d", range_image.rows, range_image.cols);

  const size_t n_beams = range_image.rows;
  const double f = (n_beams - 1) / (max_angle_ - min_angle_);
  const double delta_azimuth = Rad_Deg(2 * 60.0 / rpm_);

  // Create a point cloud and fill in points from range image
  CloudT cloud;
  cloud.points.reserve(range_image.rows * range_image.cols);

  for (int r = 0; r < range_image.rows; ++r) {
    const auto row_ptr = range_image.ptr<ushort>(r);
    for (int c = 0; c < range_image.cols; ++c) {
      const ushort range_encoded = row_ptr[c];

      if (range_encoded == 0) {
        continue;
      }

      const double range =
          (max_range_ * range_encoded) / std::numeric_limits<ushort>::max();

      const auto altitude = r / f + min_angle_;
      const auto azimuth = c * delta_azimuth;

      PointT point;
      point.x = std::cos(altitude) * std::cos(azimuth) * range;
      point.y = std::cos(altitude) * std::sin(azimuth) * range;
      point.z = std::sin(altitude) * range;
      cloud.points.push_back(point);
    }
  }

  ROS_INFO("number of points %zu", cloud.size());

  pcl_conversions::toPCL(image_msg->header, cloud.header);
  pub_cloud_.publish(cloud);
}

}  // namespace cloud2range

int main(int argc, char** argv) {
  ros::init(argc, argv, "range2cloud");

  cloud2range::Range2CloudNode node(ros::NodeHandle("~"));

  ros::spin();
}
