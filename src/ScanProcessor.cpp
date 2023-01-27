#include "ScanProcessor.h"
#include <signal.h>


ScanProcessor::ScanProcessor(ros::NodeHandle* nh) : nh(*nh),raw_cl2(new pcl::PCLPointCloud2) {
    ROS_INFO("Initializing processor");
    dir_vec.setZero(3);
    raw_cloud_count = 0;
    initSubscribers();
} 

void ScanProcessor::initSubscribers() {
    ROS_INFO("Subscribing to publiser");
    cloud_data_sub = nh.subscribe("LaserValues", 1000, &ScanProcessor::msgCallBack, this);

}

void ScanProcessor::ClearCloud() {
    pcl::PointCloud<pcl::PointXYZ> empty_cloud;
    empty_cloud.clear();
    pcl::toPCLPointCloud2(empty_cloud, *raw_cl2);
}
void ScanProcessor::msgCallBack(const testeg3d::CloudData& cloud_data) {

    Eigen::Vector3f current_vec = getDirectionVector(cloud_data);
    // ROS_INFO("Calculated direction vector");
    pcl::PCLPointCloud2::Ptr new_input (new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(cloud_data.cloud, *new_input);
    if (raw_cloud_count < 150 && !cloud_data.last)
    {
        *raw_cl2 += *new_input;
        // ROS_INFO_STREAM("Raw cloud size: " << raw_cl2->height * raw_cl2->width);
        dir_vec += current_vec;
        // ROS_INFO("Appended raw input");
        raw_cloud_count++;
    }
    else
    {
        if(cloud_data.last)
            ROS_WARN("Last point recieved");
        *raw_cl2 += *new_input;
        dir_vec += current_vec;
        ROS_INFO("segmenting edges");
        // ROS_INFO_STREAM_ONCE("Angle btw scan direction vectors: " << std::acos(std::abs(dir_vec.dot(current_vec))/(dir_vec.norm() * current_vec.norm())) << std::endl);
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cl1 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*raw_cl2, *raw_cl1);
        // pcl::io::savePCDFileASCII("/home/chl-es/TestEG3D/src/testeg3d/data/raw_cloud.pcd", *raw_cl1);
        RawCloud raw_cloud;
        raw_cloud.LoadInCloud(raw_cl1);
        raw_cloud.StatOutlierRemoval(50, 0.01);
        raw_cloud.VoxelDownSample(0.0001f);
        ROS_INFO("Filtered raw cloud");
        pcl::PointCloud<pcl::PointXYZ>::Ptr edges (new pcl::PointCloud<pcl::PointXYZ>);
        *edges = raw_cloud.FindEdgePoints(200, M_PI_2);
        ROS_INFO("Calculated edge points");
        // pcl::io::savePCDFileASCII("/home/chl-es/TestEG3D/src/testeg3d/data/edge_points.pcd", *edges);
        edge_cloud.SetScanDirection(dir_vec);
        edge_cloud.AddPoints(edges);
        ROS_INFO("Appended edge points");
        edge_cloud.ComputeVectors(50, 0.01, false);
        ROS_INFO("Computed point vectors");
        edge_cloud.RemoveFalseEdges(0.002);
        ROS_INFO("Tagged false edges");
        edge_cloud.ApplyRegionGrowing(30, 10.0 / 180.0 * M_PI, true);
        ROS_INFO("Segmented edges");

        if (cloud_data.last)
        {
            ROS_INFO("Saving edge points");
            edge_cloud.SaveCloud("/home/eshan/TestEG3D/src/testeg3d/data/edge_points.pcd");
            edge_cloud.AssembleRegions();
            ROS_INFO("Assembled segments");
            edge_cloud.CreateColouredCloud("/home/eshan/TestEG3D/src/testeg3d/data/segments.pcd");
            ROS_INFO("Saved segments");
            // ros::shutdown();
        }
        ClearCloud();
        dir_vec.setZero(3);
        raw_cloud_count = 0;

    }
}

Eigen::Vector3f ScanProcessor::getDirectionVector(const testeg3d::CloudData &cloud_data) {
    Eigen::Vector3f scan_dir(cloud_data.sensor_y[0], cloud_data.sensor_y[1], cloud_data.sensor_y[2]);
    return scan_dir;
}

ScanProcessor::~ScanProcessor() {
    edge_cloud.AssembleRegions();
    edge_cloud.CreateColouredCloud("/home/chl-es/TestEG3D/src/testeg3d/data/segments.pcd");
    ROS_INFO("Saved segmented cloud.");
}

/* void killProcessor(ScanProcessor& processor_object, const int signum) {
    ROS_WARN("Process kill with code: %i", signum);
    processor_object.~ScanProcessor();
    exit(signum);
} */

void sigint_handler(int signum) {
    ros::shutdown();
    exit(signum);
}


int main(int argc, char ** argv) {
    ros::init(argc, argv, "scan_processor", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ROS_INFO("Start scan processor node");
    ScanProcessor scan_processor(&nh);
    signal(SIGINT, sigint_handler);
    while (ros::ok)
    {
            ros::spin();
            if (ros::isShuttingDown)
                scan_processor.~ScanProcessor();
            

    }    
    return 0;
}