#include "utils.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

namespace cloud2range {

using PointT = pcl::PointXYZI;

class Cloud2RangeNode {
 public:
  explicit Cloud2RangeNode(const ros::NodeHandle& pnh);

  void CloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

 private:
  ros::NodeHandle pnh_;
  ros::Subscriber sub_cloud_;
  image_transport::ImageTransport it_;
  image_transport::CameraPublisher pub_camera_;

  int n_beams_;
  int rpm_;
  int sample_freq_;
  double min_angle_, max_angle_;
  double min_range_, max_range_;

  double d_azimuth_, d_altitude_;
  int n_cols_;

  // We are going to use camera info to store all the above params for decoding
  sensor_msgs::CameraInfo cinfo_;
};

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

  min_range_ = pnh_.param("min_range", 0.0);
  max_range_ = pnh_.param("max_range", 0.0);
  ROS_ASSERT(min_range_ < max_range_ && min_range_ >= 0.0);

  const auto model = pnh_.param<std::string>("model", "");
  ROS_INFO("lidar model: %s", model.c_str());

  ROS_INFO(
      "n_beams: %d, rpm: %d, angle(deg): [%0.2f, %0.2f], range: [%0.2f, %0.2f]",
      n_beams_, rpm_, Deg_Rad(min_angle_), Deg_Rad(max_angle_), min_range_,
      max_range_);

  n_cols_ = sample_freq_ * 60 / rpm_;
  ROS_INFO("range image shape (%d, %d)", n_beams_, n_cols_);

  d_azimuth_ = Rad_Deg(360.0 / n_cols_);
  d_altitude_ = (max_angle_ - min_angle_) / (n_beams_ - 1);
  ROS_INFO("angular resolution(deg) horizontal: %0.2f, vertical: %0.2f",
           Deg_Rad(d_azimuth_), Deg_Rad(d_altitude_));

  // Fill in cinfo
  cinfo_.height = n_beams_;
  cinfo_.width = n_cols_;
  cinfo_.distortion_model = model;
  cinfo_.K[0] = min_angle_;
  cinfo_.K[1] = max_angle_;
  cinfo_.K[2] = min_range_;
  cinfo_.K[3] = max_range_;
  cinfo_.K[4] = d_azimuth_;
  cinfo_.K[5] = d_altitude_;
}

void Cloud2RangeNode::CloudCb(
    const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  // convert to point cloud
  pcl::PointCloud<PointT> cloud;
  pcl::fromROSMsg(*cloud_msg, cloud);

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

    if (range < min_range_ || range > max_range_) {
      // skip invalid range
      continue;
    }

    // normalize range to [0, 1]
    const double range_normalized =
        (range - min_range_) / (max_range_ - min_range_);

    range_image.at<ushort>(row, col) =
        range_normalized * std::numeric_limits<ushort>::max();
  }

  ROS_DEBUG("num points %zu, num pixels %d", cloud.size(),
            cv::countNonZero(range_image));

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
