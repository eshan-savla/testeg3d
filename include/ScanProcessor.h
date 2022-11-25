#include "RawCloud.h"
#include "EdgeCloud.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/LaserScan.h>
#include <processit_msgs/ScanData.h>

class ScanProcessor
{
private:
    /* data */
public:
    ScanProcessor(ros::NodeHandle* nh);
    void initSubscribers();
    ~ScanProcessor();
};
