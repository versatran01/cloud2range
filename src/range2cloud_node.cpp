#include "utils.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

namespace cloud2range {

using PointT = pcl::PointXYZ;

class Range2CloudNode {
 public:
  explicit Range2CloudNode(const ros::NodeHandle& pnh);

  void CameraCb(const sensor_msgs::ImageConstPtr& image_msg,
                const sensor_msgs::CameraInfoConstPtr& cinfo_msg);

 private:
  ros::NodeHandle pnh_;
  ros::Publisher pub_cloud_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_camera_;
};

Range2CloudNode::Range2CloudNode(const ros::NodeHandle& pnh)
    : pnh_(pnh), it_(pnh) {
  sub_camera_ =
      it_.subscribeCamera("range/image", 1, &Range2CloudNode::CameraCb, this);
  pub_cloud_ = pnh_.advertise<pcl::PointCloud<PointT>>("cloud_ordered", 1);
}

void Range2CloudNode::CameraCb(
    const sensor_msgs::ImageConstPtr& image_msg,
    const sensor_msgs::CameraInfoConstPtr& cinfo_msg) {
  // Convert to image
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(image_msg);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  const cv::Mat range_image = cv_ptr->image;

  // Extract params from cinfo
  const double min_angle = cinfo_msg->K[0];
  //  const double max_angle = cinfo_msg->K[1];
  const double min_range = cinfo_msg->K[2];
  const double max_range = cinfo_msg->K[3];
  const double d_azimuth = cinfo_msg->K[4];
  const double d_altitude = cinfo_msg->K[5];

  // Create a point cloud and fill in points from range image
  pcl::PointCloud<PointT> cloud;
  cloud.points.reserve(range_image.rows * range_image.cols);

  for (int r = 0; r < range_image.rows; ++r) {
    const auto row_ptr = range_image.ptr<ushort>(r);
    for (int c = 0; c < range_image.cols; ++c) {
      const ushort range_encoded = row_ptr[c];

      // skip points with 0 range
      if (range_encoded == 0) {
        continue;
      }

      const double range_norm = static_cast<double>(range_encoded - 1) /
                                (std::numeric_limits<ushort>::max() - 1);
      const double range = range_norm * (max_range - min_range) + min_range;

      const auto altitude = r * d_altitude + min_angle;
      const auto azimuth = c * d_azimuth;

      PointT point;
      point.x = std::cos(altitude) * std::cos(azimuth) * range;
      point.y = std::cos(altitude) * std::sin(azimuth) * range;
      point.z = std::sin(altitude) * range;
      cloud.points.push_back(point);
    }
  }

  ROS_DEBUG("num restored points %zu", cloud.size());

  pcl_conversions::toPCL(image_msg->header, cloud.header);
  pub_cloud_.publish(cloud);
}

}  // namespace cloud2range

int main(int argc, char** argv) {
  ros::init(argc, argv, "range2cloud");

  cloud2range::Range2CloudNode node(ros::NodeHandle("~"));

  ros::spin();
}
