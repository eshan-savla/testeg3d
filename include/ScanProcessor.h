#include "RawCloud.h"
#include "EdgeCloud.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/LaserScan.h>
#include <processit_msgs/ScanData.h>
#include <feature_line_tracing_test/CloudData.h>

class ScanProcessor
{
private:
    ros::NodeHandle nh;
    bool valid;
    ros::Subscriber cloud_data_sub;
    pcl::PointCloud<pcl::PointXYZ>::Ptr edge_points;
    /* data */
public:
    ScanProcessor(ros::NodeHandle* nh);
    void initSubscribers();
    void msgCallBack(const feature_line_tracing_test::CloudData& cloud_data);
    pcl::PointCloud<pcl::PointXYZ> getEdgePoints(const sensor_msgs::PointCloud2Ptr cloud_in);
    pcl::PointCloud<pcl::PointXYZ> getSegments(const pcl::PointCloud<pcl::PointXYZ>& edge_points);
    ~ScanProcessor();
};