#pragma once

#include "RawCloud.h"
#include "EdgeCloud.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/LaserScan.h>
#include <processit_msgs/ScanData.h>
#include <testeg3d/CloudData.h>

class ScanProcessor
{
private:
    ros::NodeHandle nh;
    bool valid;
    int raw_cloud_count;
    ros::Subscriber cloud_data_sub;
    pcl::PCLPointCloud2::Ptr raw_cl2;
    Eigen::Vector3f dir_vec;
    EdgeCloud edge_cloud;

    static Eigen::Vector3f getDirectionVector(const testeg3d::CloudData &cloud_data);

public:
    ScanProcessor(ros::NodeHandle* nh);
    void initSubscribers();
    void msgCallBack(const testeg3d::CloudData& cloud_data);
    pcl::PointCloud<pcl::PointXYZ> getEdgePoints(const sensor_msgs::PointCloud2Ptr cloud_in);
    pcl::PointCloud<pcl::PointXYZ> getSegments(const pcl::PointCloud<pcl::PointXYZ>& raw_cl2);
    ~ScanProcessor();
};