#include <ros/ros.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/radius_outlier_removal.h>
// Message filter includes
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// Message includes
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

// 定义PCL点云类型
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class PointCloudFilter {
public:
    PointCloudFilter() {
        // 初始化ROS节点句柄
        nh_ = ros::NodeHandle("~");

        // --- 读取参数 ---
        nh_.param("filter_radius", filter_radius_, 0.1);
        ROS_INFO("Self-filter radius is set to: %f meters", filter_radius_);

        nh_.param("ror_radius", ror_radius_, 0.2);
        nh_.param("ror_min_neighbors", ror_min_neighbors_, 5);
        ROS_INFO("RadiusOutlierRemoval: search radius = %f, min neighbors = %d",
                 ror_radius_, ror_min_neighbors_);

        // --- 初始化订阅并发布 ---
        pc_sub_.subscribe(nh_, "/livox/lidar_ros", 10);
        odom_sub_.subscribe(nh_, "/drone_0_visual_slam/odom", 10);

        sync_.reset(new Sync(MySyncPolicy(10), pc_sub_, odom_sub_));
        sync_->registerCallback(boost::bind(&PointCloudFilter::callback, this, _1, _2));

        filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/livox/lidar_filtered", 10);
    }

private:
    // --- 类型定义 ---
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;

    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    boost::shared_ptr<Sync> sync_;

    void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                const nav_msgs::OdometryConstPtr& odom_msg) {
        PointCloud::Ptr input_cloud(new PointCloud);
        pcl::fromROSMsg(*cloud_msg, *input_cloud);

        if (input_cloud->points.empty()) {
            return;
        }

        // 从里程计获取机器人当前位置 (世界坐标系下)
        double x_r = odom_msg->pose.pose.position.x;
        double y_r = odom_msg->pose.pose.position.y;
        double z_r = odom_msg->pose.pose.position.z;

        // --- 阶段 1: 执行基于机器人位置的自滤波 ---
        PointCloud::Ptr self_filtered_cloud(new PointCloud);
        double radius_squared = filter_radius_ * filter_radius_;
        for (const auto& point : input_cloud->points) {
            double dx = point.x - x_r;
            double dy = point.y - y_r;
            double dz = point.z - z_r;
            double distance_squared = dx * dx + dy * dy + dz * dz;
            if (distance_squared > radius_squared) {
                self_filtered_cloud->points.push_back(point);
            }
        }

        if (self_filtered_cloud->points.empty()) {
            sensor_msgs::PointCloud2 output_msg;
            pcl::toROSMsg(*self_filtered_cloud, output_msg);
            output_msg.header = cloud_msg->header;
            filtered_pub_.publish(output_msg);
            return;
        }

        // --- 阶段 2: 执行半径异常点移除滤波 ---
        PointCloud::Ptr final_filtered_cloud(new PointCloud);
        pcl::RadiusOutlierRemoval<PointT> ror_filter;
        ror_filter.setInputCloud(self_filtered_cloud);
        ror_filter.setRadiusSearch(ror_radius_);
        ror_filter.setMinNeighborsInRadius(ror_min_neighbors_);
        ror_filter.filter(*final_filtered_cloud);

        // --- 发布最终结果 ---
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*final_filtered_cloud, output_msg);
        output_msg.header = cloud_msg->header;
        filtered_pub_.publish(output_msg);
    }

    // ROS 相关句柄与发布器
    ros::NodeHandle nh_;
    ros::Publisher filtered_pub_;

    // 滤波器参数
    double filter_radius_;
    double ror_radius_;
    int ror_min_neighbors_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_filter_node");
    PointCloudFilter filter;
    ros::spin();
    return 0;
}
