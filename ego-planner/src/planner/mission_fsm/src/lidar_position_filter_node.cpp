#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <cmath>
#include <mutex>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class LidarPositionFilterNode {
public:
  LidarPositionFilterNode(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh) {
    pnh_.param<std::string>("input_topic", input_topic_, "/livox/lidar");
    pnh_.param<std::string>("output_topic", output_topic_, "/livox/lidar_register");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/unity_odom");
    pnh_.param<double>("filter_radius", filter_radius_, 0.5);
    pnh_.param<int>("queue_size", queue_size_, 5);

    if (filter_radius_ < 0.0) {
      ROS_WARN_STREAM("[lidar_position_filter] filter_radius must be >= 0. Reset to 0.5");
      filter_radius_ = 0.5;
    }
    if (queue_size_ < 1) {
      ROS_WARN_STREAM("[lidar_position_filter] queue_size must be >= 1. Reset to 5");
      queue_size_ = 5;
    }

    odom_sub_ = nh_.subscribe(odom_topic_, queue_size_, &LidarPositionFilterNode::odomCallback, this,
                              ros::TransportHints().tcpNoDelay(true));
    cloud_sub_ = nh_.subscribe(input_topic_, queue_size_, &LidarPositionFilterNode::cloudCallback, this,
                               ros::TransportHints().tcpNoDelay(true));
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, queue_size_);

    ROS_INFO_STREAM("[lidar_position_filter] Started. Input: " << input_topic_ << ", Output: " << output_topic_
                                                                << ", Odom: " << odom_topic_
                                                                << ", FilterRadius: " << filter_radius_);
  }

private:
  void odomCallback(const nav_msgs::OdometryConstPtr &msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_x_ = msg->pose.pose.position.x;
    odom_y_ = msg->pose.pose.position.y;
    odom_z_ = msg->pose.pose.position.z;
    has_odom_ = true;
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *input_cloud);

    if (input_cloud->empty()) {
      cloud_pub_.publish(*msg);
      return;
    }

    double center_x = 0.0;
    double center_y = 0.0;
    double center_z = 0.0;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (!has_odom_) {
        ROS_WARN_STREAM_THROTTLE(2.0, "[lidar_position_filter] Waiting odometry from " << odom_topic_
                                                                                         << ", pass-through input cloud.");
        cloud_pub_.publish(*msg);
        return;
      }
      center_x = odom_x_;
      center_y = odom_y_;
      center_z = odom_z_;
    }

    filtered_cloud->points.reserve(input_cloud->points.size());
    const double radius_sq = filter_radius_ * filter_radius_;

    for (const auto &pt : input_cloud->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
        continue;
      }
      const double dx = pt.x - center_x;
      const double dy = pt.y - center_y;
      const double dz = pt.z - center_z;
      if (dx * dx + dy * dy + dz * dz <= radius_sq) {
        continue;
      }
      filtered_cloud->points.push_back(pt);
    }

    filtered_cloud->width = static_cast<uint32_t>(filtered_cloud->points.size());
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    sensor_msgs::PointCloud2 out_msg;
    pcl::toROSMsg(*filtered_cloud, out_msg);
    out_msg.header = msg->header;
    cloud_pub_.publish(out_msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber cloud_sub_;
  ros::Publisher cloud_pub_;

  std::string input_topic_;
  std::string output_topic_;
  std::string odom_topic_;
  double filter_radius_;
  int queue_size_;

  std::mutex odom_mutex_;
  bool has_odom_ = false;
  double odom_x_ = 0.0;
  double odom_y_ = 0.0;
  double odom_z_ = 0.0;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_position_filter_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  LidarPositionFilterNode node(nh, pnh);
  ros::spin();
  return 0;
}
