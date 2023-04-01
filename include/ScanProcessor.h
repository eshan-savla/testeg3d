#pragma once

#include "RawCloud.h"
#include "EdgeCloud.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/LaserScan.h>
#include <testeg3d/CloudData.h>

class ScanProcessor
{
private:
    ros::NodeHandle nh;
    bool first;
    bool valid;
    int raw_cloud_count;
    double cloud_gap_count, cloud_gap_reuse;
    std::vector<int> first_ind, last_ind, reuse_ind_start, reuse_ind_end;
    std::vector<unsigned int> segment_sizes;
    ros::Subscriber cloud_data_sub;
    pcl::PCLPointCloud2::Ptr raw_cl2;
    pcl::PCLPointCloud2::Ptr reuse_cl2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cl, reuse_cl, downsampled_cl;
    Eigen::Vector3f dir_vec, reuse_vec;
    pcl::VoxelGrid<pcl::PointXYZ> vg_sampler;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outrem;
    EdgeCloud edge_cloud;
    testeg3d::CloudData first_message, last_message, reuse_message;

    static Eigen::Vector3f getDirectionVector(const testeg3d::CloudData &cloud_data);
    static void ClearCloud(pcl::PCLPointCloud2::Ptr &cloud2);

public:
    ScanProcessor(ros::NodeHandle* nh);
    void initSubscribers();
    void msgCallBack(const testeg3d::CloudData& cloud_data);
    pcl::PointCloud<pcl::PointXYZ> getEdgePoints(const sensor_msgs::PointCloud2Ptr cloud_in);
    pcl::PointCloud<pcl::PointXYZ> getSegments(const pcl::PointCloud<pcl::PointXYZ>& raw_cl2);
    ~ScanProcessor();
};