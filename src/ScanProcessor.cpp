#include "ScanProcessor.h"
#include <signal.h>
#include <numeric>

ScanProcessor::ScanProcessor(ros::NodeHandle* nh) : nh(*nh),raw_cl2(new pcl::PCLPointCloud2), reuse_cl2(new pcl::PCLPointCloud2) {
    ROS_INFO("Initializing processor");
    dir_vec.setZero(3);
    raw_cloud_count = 0;
    edge_cloud.SetSensorSpecs(0.0, 0.290, 0.460);
    edge_cloud.SetDownsampling(true, 0.0001f);
    initSubscribers();
} 

void ScanProcessor::initSubscribers() {
    ROS_INFO("Subscribing to publiser");
    cloud_data_sub = nh.subscribe("LaserValues", 1000, &ScanProcessor::msgCallBack, this);

}

void ScanProcessor::ClearCloud(pcl::PCLPointCloud2::Ptr &cloud2) {
    pcl::PointCloud<pcl::PointXYZ> empty_cloud;
    empty_cloud.clear();
    pcl::toPCLPointCloud2(empty_cloud, *cloud2);
}
void ScanProcessor::msgCallBack(const testeg3d::CloudData& cloud_data) {

    Eigen::Vector3f current_vec = getDirectionVector(cloud_data);
    // ROS_INFO("Calculated direction vector");
    pcl::PCLPointCloud2::Ptr new_input (new pcl::PCLPointCloud2);
    int n = static_cast<int>(0.004/cloud_data.gap) - 1;
    int reuse_count = static_cast<int>(0.0001/cloud_data.gap);
    int false_seg_count = static_cast<int>(0.0001/cloud_data.gap);
    ROS_INFO_ONCE("No. of scans needed: %i", n);
    pcl_conversions::toPCL(cloud_data.cloud, *new_input);
    if (raw_cloud_count < n && !cloud_data.last) 
    {
        if (raw_cloud_count == 0){
            first_message = cloud_data;
            // pcl::PointCloud<pcl::PointXYZ>::Ptr first_seg (new pcl::PointCloud<pcl::PointXYZ>);
            // pcl::fromPCLPointCloud2(*new_input, *first_seg);
            // first_ind.resize(first_seg->size());
            // std::iota(first_ind.begin(), first_ind.end(), 0);
        }
        *raw_cl2 += *new_input;
        segment_sizes.push_back(new_input->width * new_input->height);
        // ROS_INFO_STREAM("Raw cloud size: " << raw_cl2->height * raw_cl2->width);
        dir_vec += current_vec;
        // ROS_INFO("Appended raw input");
        if (raw_cloud_count >= n - reuse_count) {
            reuse_message = cloud_data;
            *reuse_cl2 += *new_input;
        }
        raw_cloud_count++;
    }
    else
    {
        if(cloud_data.last)
            ROS_WARN("Last point recieved");
        *raw_cl2 += *new_input;
        *reuse_cl2 += *new_input;
        dir_vec += current_vec;
        segment_sizes.push_back(new_input->width * new_input->height);
        last_message = cloud_data;
        ROS_INFO("segmenting edges");
        // ROS_INFO_STREAM_ONCE("Angle btw scan direction vectors: " << std::acos(std::abs(dir_vec.dot(current_vec))/(dir_vec.norm() * current_vec.norm())) << std::endl);
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cl1 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*raw_cl2, *raw_cl1);
        unsigned int first_size = 0, last_size = 0, reuse_size = 0;
        for (size_t i = 0; i < false_seg_count; i++)
        {
            first_size += segment_sizes.at(i);
            last_size += segment_sizes.at(segment_sizes.size() - false_seg_count + i);
            reuse_size += segment_sizes.at(segment_sizes.size() - reuse_count + i);

        }
        first_ind.resize(first_size);
        last_ind.resize(last_size);
        std::iota(first_ind.begin(), first_ind.end(), 0);
        std::iota(last_ind.begin(), last_ind.end(),raw_cl1->size() - last_size);
        // pcl::io::savePCDFileASCII("/home/chl-es/TestEG3D/src/testeg3d/data/raw_cloud.pcd", *raw_cl1);
        RawCloud raw_cloud;
        raw_cloud.LoadInCloud(raw_cl1);
        raw_cloud.SetFirstInd(first_ind);
        raw_cloud.SetLastInd(last_ind);
        raw_cloud.SetFilterCriteria(true, true);
        // raw_cloud.StatOutlierRemoval(50, 0.01);
        // raw_cloud.VoxelDownSample(0.0001f);
        ROS_INFO("Filtered raw cloud");
        pcl::PointCloud<pcl::PointXYZ>::Ptr edges (new pcl::PointCloud<pcl::PointXYZ>);
        *edges = raw_cloud.FindEdgePoints(200, M_PI_2);
        ROS_INFO("Calculated edge points");
        pcl::io::savePCDFileASCII("/home/eshan/TestEG3D/src/testeg3d/data/edge_points.pcd", *edges);
        edge_cloud.SetScanDirection(dir_vec);
        edge_cloud.SetSensorCoords(first_message.sensor_x, first_message.sensor_y, first_message.sensor_z, first_message.sensor_position, "first");
        edge_cloud.SetSensorCoords(last_message.sensor_x, last_message.sensor_y, last_message.sensor_z, last_message.sensor_position, "last");
        edge_cloud.AddPoints(edges);
        ROS_INFO("Appended edge points");
        edge_cloud.ComputeVectors(20, 0.01, reuse_size, false);
        ROS_INFO("Computed point vectors");
        // edge_cloud.RemoveFalseEdges(0.001, true);
        // ROS_INFO("Tagged false edges");
        edge_cloud.ApplyRegionGrowing(20, 20.0 / 180.0 * M_PI, true);
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
        ClearCloud(raw_cl2);
        *raw_cl2 += *reuse_cl2;
        dir_vec.setZero(3);
        ClearCloud(reuse_cl2);
        raw_cloud_count = reuse_count;
        first_message = reuse_message;
        ROS_INFO_STREAM("segment_sizes vec size: " << segment_sizes.size());
        segment_sizes.erase(segment_sizes.begin(), segment_sizes.end() - reuse_count);
        ROS_INFO_STREAM("Segment_siezes vec size: " << segment_sizes.size());
        // first_ind = reuse_ind;
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