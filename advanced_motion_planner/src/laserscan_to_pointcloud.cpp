#include <advanced_motion_planner/laserscan_to_pointcloud.h>

bool LaserScanToPointCloud::filter(float range, float angle) {
  constexpr auto max_range = 2.0f;
  constexpr auto min_range = 0.3f;
  constexpr auto angle_range = 0.5f;

  if (range > max_range ||
      range < min_range) {
      return false;
  }

  if (angle < -angle_range ||
      angle > angle_range) {
      return false;
  }

  return true;
}

pcl::PointCloud<pcl::PointXYZ> LaserScanToPointCloud::scanToCloud(const sensor_msgs::LaserScan &scan, bool insideFilter) {

    constexpr float pi = atan(1.0f) * 4.0f;
    pcl::PointXYZ point(0.0f, 0.0f, 0.0f);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.reserve(1081); //check this thing out

    for (uint32_t i = 0; i < scan.ranges.size(); ++i) {

        float r = scan.ranges[i];

        // Offset of the lidar is 90 degrees
        constexpr float offset = 0.5f * pi;
        float theta = scan.angle_min + static_cast<float>(i) * scan.angle_increment + offset;

        if (filter(r, theta) == insideFilter) {
          point.x = r * cos(theta);
          point.y = r * sin(theta);
          point.z = 0.0f;

          cloud.push_back(point);
        }
    }

    return cloud;
}
