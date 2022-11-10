#include <math.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/registration/transforms.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/ndt.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/io/pcd_io.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv/cv.h>

#define PI 3.14159265358979323846

using namespace std;

typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> Pointcloud;
typedef sensor_msgs::Imu msgs_Imu;
typedef sensor_msgs::PointCloud2 msgs_Point;
typedef sensor_msgs::NavSatFix msgs_Nav;
typedef nav_msgs::Odometry msgs_Odom;

typedef Eigen::Vector2f Vector2f;
typedef Eigen::Vector3f Vector3f;
typedef Eigen::Vector4f Vector4f;
typedef Eigen::Matrix3f Matrix3f;
typedef Eigen::Matrix4f Matrix4f;

/*extern const int N_SCAN = 16;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 0.2;
extern const float ang_res_y = 2.0;
extern const float ang_bottom = 15.0;
extern const int groundScanInd = 7;
extern const float sensorMountAngle = 0.0;*/

struct position
{
    double x;
    double y;
    double z;
};

struct PointXYZMAP
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint32_t index_;
    float position_x;
    float position_y;
    float position_z;
    float latitude;
    float longitude;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZMAP,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (uint32_t, index_, index_)
                                   (float, position_x, position_x)
                                   (float, position_y, position_y)
                                   (float, position_z, position_z)
                                   (float, latitude, latitude)
                                   (float, longitude, longitude)
                                   )
PCL_INSTANTIATE_PointCloud(PointXYZMAP);

typedef PointXYZMAP MapInfo;
typedef pcl::PointCloud<MapInfo> PointMap;