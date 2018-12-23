#include "cloud2range_node.h"
#include "utils.h"

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/timer/timer.hpp>

namespace cloud2range {
using namespace sensor_msgs;

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

Cloud2RangeNode::Cloud2RangeNode(const ros::NodeHandle &pnh) : pnh_(pnh) {
  sub_cloud_ = pnh_.subscribe("cloud", 1, &Cloud2RangeNode::CloudCb, this);
  pub_image_ = pnh_.advertise<sensor_msgs::Image>("range", 1);
  n_beams_ = 16;
  rpm_ = 600;
  min_angle_ = Rad_Deg(-15);
  max_angle_ = Rad_Deg(15);
  max_range_ = 100;
}

void Cloud2RangeNode::CloudCb(const PointCloud2ConstPtr &cloud_msg) {
  CloudT cloud;
  pcl::fromROSMsg(*cloud_msg, cloud);
  ROS_INFO("number of points %zu, dense %d", cloud.size(), (int)cloud.is_dense);

  const double delta_azimuth = Rad_Deg(2 * 60.0 / rpm_);
  const size_t cols = 2 * M_PI / delta_azimuth;
  cv::Mat range_image =
      cv::Mat::zeros(n_beams_, cols, cv_bridge::getCvType("mono16"));
  ROS_INFO("range image with %dx%d", range_image.rows, range_image.cols);

  {
    boost::timer::auto_cpu_timer t;

    //#pragma omp parallel for
    for (size_t i = 0; i < cloud.size(); ++i) {
      // calculate theta (altitude) and phi (azimuth) and put into range image
      const auto &point = cloud[i];

      const auto range = PointRange(point);

      //      if (range < 1e-5) {
      //        ROS_WARN("range is close to zero %f", range);
      //      }
      const auto altitude = PointAltitude(point);
      const auto azimuth = PointAzimuth(point);

      const size_t row =
          Altitude2Row(altitude, n_beams_, min_angle_, max_angle_);
      const size_t col = azimuth / delta_azimuth;

      //      if (range_image.at<ushort>(row, col) > 0) {
      //        ROS_WARN("duplicate");
      //      }

      range_image.at<ushort>(row, col) =
          static_cast<ushort>(std::min(range, max_range_) / max_range_ *
                              std::numeric_limits<ushort>::max());
    }
  }

  ROS_INFO("valid pixels %d", cv::countNonZero(range_image));

  cv_bridge::CvImage cv_image(cloud_msg->header, "mono16", range_image);
  pub_image_.publish(cv_image.toImageMsg());
}

}  // namespace cloud2range
