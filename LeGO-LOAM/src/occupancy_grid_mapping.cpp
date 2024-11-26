#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <tf/transform_listener.h>

// 地图参数
const int MAP_WIDTH = 400;      // 栅格地图的宽度 (单位：格)
const int MAP_HEIGHT = 400;     // 栅格地图的高度 (单位：格)
const float RESOLUTION = 0.05;  // 每个栅格对应的物理大小 (单位：米)
const float ORIGIN_X = -10.0;   // 地图的原点 x (米)
const float ORIGIN_Y = -10.0;   // 地图的原点 y (米)
const int LOG_ODDS_FREE = -2;   // 空闲空间的更新权重
const int LOG_ODDS_OCCUPIED = 2; // 占据空间的更新权重
const int LOG_ODDS_MAX = 100;    // 最大占据值
const int LOG_ODDS_MIN = -100;   // 最小占据值

template<typename T>
T clamp(T value, T min, T max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
// 全局变量
nav_msgs::OccupancyGrid occupancyGrid;
std::vector<int> logOdds(MAP_WIDTH * MAP_HEIGHT, 0);
Eigen::Vector2f robotPos;

// 辅助函数：将世界坐标转换为地图坐标
Eigen::Vector2i worldToMap(const Eigen::Vector2f& point) {
    int x_index = static_cast<int>((point.x() - ORIGIN_X) / RESOLUTION);
    int y_index = static_cast<int>((point.y() - ORIGIN_Y) / RESOLUTION);
    return Eigen::Vector2i(x_index, y_index);
}

// 辅助函数：将地图索引转换为 1D 数组索引
int getIndex(int x, int y) {
    return y * MAP_WIDTH + x;
}

// 辅助函数：Bresenham 算法
void bresenham(int x0, int y0, int x1, int y1, std::vector<Eigen::Vector2i>& points) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    while (true) {
        points.emplace_back(x0, y0);
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

// 激光扫描回调函数
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        float range = scan->ranges[i];
        if (range < scan->range_min || range > scan->range_max) continue;

        float angle = scan->angle_min + i * scan->angle_increment;
        Eigen::Vector2f end(range * cos(angle), range * sin(angle));
        Eigen::Vector2i startGrid = worldToMap(robotPos);
        Eigen::Vector2i endGrid = worldToMap(end);

        std::vector<Eigen::Vector2i> points;
        bresenham(startGrid[0], startGrid[1], endGrid[0], endGrid[1], points);

        // 更新自由空间
        for (size_t j = 0; j < points.size() - 1; ++j) {
            int index = getIndex(points[j][0], points[j][1]);
            logOdds[index] = std::max(LOG_ODDS_MIN, logOdds[index] + LOG_ODDS_FREE);
        }

        // 更新占据空间
        int index = getIndex(endGrid[0], endGrid[1]);
        logOdds[index] = std::min(LOG_ODDS_MAX, logOdds[index] + LOG_ODDS_OCCUPIED);
    }
}

// 全局点云回调函数（初始化地图背景）
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloudMsg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloudMsg, *cloud);

    for (const auto& point : cloud->points) {
        Eigen::Vector2i gridPos = worldToMap(Eigen::Vector2f(point.x, point.y));
        int index = getIndex(gridPos[0], gridPos[1]);
        logOdds[index] = std::min(LOG_ODDS_MAX, logOdds[index] + LOG_ODDS_OCCUPIED);
    }
}

// 发布栅格地图
void publishOccupancyGrid(ros::Publisher& pub) {
    occupancyGrid.data.clear();
    for (int logOdd : logOdds) {
        int value = 50 + logOdd * 50 / LOG_ODDS_MAX; // 转换为 [0,100]
        occupancyGrid.data.push_back(clamp(value, 0, 100));
    }
    pub.publish(occupancyGrid);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "occupancy_grid_mapping");
    ros::NodeHandle nh;
    tf::StampedTransform transform;
    tf::TransformListener listener;  // 确保在全局作用域中声明

    // 初始化栅格地图
    occupancyGrid.header.frame_id = "map";
    occupancyGrid.info.resolution = RESOLUTION;
    occupancyGrid.info.width = MAP_WIDTH;
    occupancyGrid.info.height = MAP_HEIGHT;
    occupancyGrid.info.origin.position.x = ORIGIN_X;
    occupancyGrid.info.origin.position.y = ORIGIN_Y;

    // 订阅话题
    ros::Subscriber laserSub = nh.subscribe("/laser_scan", 10, laserScanCallback);
    ros::Subscriber pointCloudSub = nh.subscribe("/laser_without_ground_map", 10, pointCloudCallback);

    // 发布栅格地图
    ros::Publisher gridPub = nh.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid", 10);

    ros::Rate rate(10); // 10Hz
    while (ros::ok()) {
        try {
            listener.lookupTransform("map", "base_link", ros::Time(0), transform);
            robotPos.x() = transform.getOrigin().x();
            robotPos.y() = transform.getOrigin().y();
            }
            catch (tf::TransformException& ex) {
                ROS_WARN("Failed to get transform: %s", ex.what());
        }
        occupancyGrid.header.stamp = ros::Time::now();
        publishOccupancyGrid(gridPub);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
