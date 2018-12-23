#include "utils.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

namespace cloud2range {

class Cloud2RangeNode {
 public:
  explicit Cloud2RangeNode(const ros::NodeHandle& pnh);

  void CloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

 private:
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  ros::Subscriber sub_cloud_;
  image_transport::CameraPublisher pub_camera_;

  int n_beams_;
  int rpm_;
  int sample_freq_;
  double min_angle_, max_angle_;
  double max_range_;

  double d_azimuth_, d_altitude_;
  int n_cols_;

  // We are going to use camera info to store all the above params for decoding
  sensor_msgs::CameraInfo cinfo_;
};

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

Cloud2RangeNode::Cloud2RangeNode(const ros::NodeHandle& pnh)
    : pnh_(pnh), it_(pnh) {
  sub_cloud_ = pnh_.subscribe("cloud", 1, &Cloud2RangeNode::CloudCb, this);
  pub_camera_ = it_.advertiseCamera("range/image", 1);

  // Read params
  n_beams_ = pnh_.param("n_beams", 0);
  ROS_ASSERT(n_beams_ > 0);
  rpm_ = pnh_.param("rpm", 0);
  ROS_ASSERT(rpm_ > 0);
  sample_freq_ = pnh_.param("sample_freq", 0);
  ROS_ASSERT(sample_freq_ > 0);

  min_angle_ = pnh_.param("min_angle", 0.0);
  max_angle_ = pnh_.param("max_angle", 0.0);
  ROS_ASSERT(min_angle_ < max_angle_);

  max_range_ = pnh_.param("max_range", 0.0);
  ROS_ASSERT(max_range_ > 0.0);

  ROS_INFO("n_beams: %d, rpm: %d, angle(deg): [%0.2f, %0.2f], max range: %0.2f",
           n_beams_, rpm_, Deg_Rad(min_angle_), Deg_Rad(max_angle_),
           max_range_);

  n_cols_ = sample_freq_ * 60 / rpm_;
  ROS_INFO("range image shape (%d, %d)", n_beams_, n_cols_);

  d_azimuth_ = Rad_Deg(360.0 / n_cols_);
  d_altitude_ = (max_angle_ - min_angle_) / (n_beams_ - 1);
  ROS_INFO("angular resolution(deg) horizontal: %0.2f, vertical: %0.2f",
           Deg_Rad(d_azimuth_), Deg_Rad(d_altitude_));

  // Fill in cinfo
}

void Cloud2RangeNode::CloudCb(
    const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  // convert to point cloud
  CloudT cloud;
  pcl::fromROSMsg(*cloud_msg, cloud);
  ROS_INFO("number of points %zu", cloud.size());

  cv::Mat range_image = cv::Mat::zeros(n_beams_, n_cols_, CV_16UC1);

  for (size_t i = 0; i < cloud.size(); ++i) {
    // calculate altitude and azimuth and put into range image
    const auto& point = cloud[i];
    const auto range = PointRange(point);
    const auto altitude = PointAltitude(point);
    const auto azimuth = PointAzimuth(point);

    const int row = (altitude - min_angle_) / d_altitude_ + 0.5;
    // const int col = static_cast<int>(azimuth / d_azimuth_ + 0.5) % n_cols_;
    // int round towards zero
    const int col = azimuth / d_azimuth_ + 0.5;
    // make sure valid
    ROS_ASSERT(row >= 0 && row < n_beams_);
    ROS_ASSERT(col >= 0 && col < n_cols_);

    range_image.at<ushort>(row, col) =
        std::min(range / max_range_, 1.0) * std::numeric_limits<ushort>::max();
  }

  // update header
  cinfo_.header = cloud_msg->header;
  cv_bridge::CvImage cv_image(cloud_msg->header, "mono16", range_image);
  pub_camera_.publish(*cv_image.toImageMsg(), cinfo_);
}

}  // namespace cloud2range

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud2range");
  cloud2range::Cloud2RangeNode node(ros::NodeHandle("~"));
  ros::spin();
}
