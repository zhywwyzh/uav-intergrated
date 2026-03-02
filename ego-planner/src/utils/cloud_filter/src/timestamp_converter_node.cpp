#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class TimestampConverter {
public:
    TimestampConverter() {
        ros::NodeHandle nh;

        // 订阅原始 Livox 点云
        sub_ = nh.subscribe("/livox/lidar", 10, &TimestampConverter::callback, this);

        // 发布带有 ROS 时间戳的点云
        pub_ = nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar_ros", 10);

        ROS_INFO("TimestampConverterNode started: /livox/lidar -> /livox/lidar_ros");
    }

private:
    void callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        sensor_msgs::PointCloud2 new_msg = *msg;

        // 替换时间戳为 ROS 当前时间
        new_msg.header.stamp = ros::Time::now();

        // 保持 frame_id 不变（通常是 lidar_frame 或 base_link）
        pub_.publish(new_msg);
    }

    ros::Subscriber sub_;
    ros::Publisher pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "timestamp_converter_node");
    TimestampConverter tc;
    ros::spin();
    return 0;
}
