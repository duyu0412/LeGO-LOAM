#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

// 全局变量初始化
static tf::StampedTransform last_transform;
static ros::Time last_time;

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "base_link_to_map_to_odom");
    ros::NodeHandle nh;

    // 定义 TF 监听器
    tf::TransformListener tfListener;

    // 定义里程计发布器
    ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry>("/odom", 50);

    ros::Rate rate(50); // 50Hz 发布频率

    bool first_transform_received = false;

    while (ros::ok()) {
        tf::StampedTransform current_transform;
        ros::Time current_time = ros::Time::now();

        try {
            // 获取 base_link -> map 的变换
            tfListener.lookupTransform("map", "base_link", ros::Time(0), current_transform);

            // 如果是第一次接收 TF，初始化变量
            if (!first_transform_received) {
                last_transform = current_transform;
                last_time = current_time;
                first_transform_received = true;
                rate.sleep();
                continue;
            }

            // 时间差
            double dt = (current_time - last_time).toSec();
            if (dt > 0.0) {
                // 计算线速度
                double dx = current_transform.getOrigin().x() - last_transform.getOrigin().x();
                double dy = current_transform.getOrigin().y() - last_transform.getOrigin().y();
                double dz = current_transform.getOrigin().z() - last_transform.getOrigin().z();

                // 计算角速度
                tf::Quaternion delta_q = current_transform.getRotation() * last_transform.getRotation().inverse();
                tf::Vector3 delta_euler = delta_q.getAxis() * delta_q.getAngle();

                // 更新上一次的时间和变换
                last_transform = current_transform;
                last_time = current_time;

                // 构造里程计消息
                nav_msgs::Odometry odom;
                odom.header.stamp = current_time;
                odom.header.frame_id = "map";       // 里程计的全局参考系
                odom.child_frame_id = "base_link"; // 机器人参考系

                // 设置位置信息
                odom.pose.pose.position.x = current_transform.getOrigin().x();
                odom.pose.pose.position.y = current_transform.getOrigin().y();
                odom.pose.pose.position.z = current_transform.getOrigin().z();

                // 设置姿态信息（四元数）
                odom.pose.pose.orientation.x = current_transform.getRotation().x();
                odom.pose.pose.orientation.y = current_transform.getRotation().y();
                odom.pose.pose.orientation.z = current_transform.getRotation().z();
                odom.pose.pose.orientation.w = current_transform.getRotation().w();

                // 设置线速度
                odom.twist.twist.linear.x = dx / dt;
                odom.twist.twist.linear.y = dy / dt;
                odom.twist.twist.linear.z = dz / dt;

                // 设置角速度
                odom.twist.twist.angular.x = delta_euler.x() / dt;
                odom.twist.twist.angular.y = delta_euler.y() / dt;
                odom.twist.twist.angular.z = delta_euler.z() / dt;

                // 发布里程计消息
                odomPub.publish(odom);
            }

        } catch (tf::TransformException& ex) {
            ROS_WARN("Could not get transform from map to base_link: %s", ex.what());
        }

        rate.sleep();
    }

    return 0;
}
