#include "ScanProcessor.h"
#include <signal.h>
#include <numeric>

ScanProcessor::ScanProcessor(ros::NodeHandle* nh) : nh(*nh),raw_cl2(new pcl::PCLPointCloud2), reuse_cl2(new pcl::PCLPointCloud2), 
                    raw_cl(new pcl::PointCloud<pcl::PointXYZ>), reuse_cl(new pcl::PointCloud<pcl::PointXYZ>) {
    ROS_INFO("Initializing processor");
    dir_vec.setZero(3);
    raw_cloud_count = 0;
    edge_cloud.SetSensorSpecs(0.0, 0.290, 0.460);
    edge_cloud.SetDownsampling(false);
    edge_cloud.SetStatOutRem(true, 50, 1.5);
    raw_cl->clear();
    float leaf_size = 0.0001f;
    vg_sampler.setLeafSize(leaf_size, leaf_size, leaf_size);
    outrem.setMeanK(20);
    outrem.setStddevMulThresh(1.0);
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
void ScanProcessor::msgCallBack (const testeg3d::CloudData& cloud_data) {

    Eigen::Vector3f current_vec = getDirectionVector(cloud_data);
    // ROS_INFO("Calculated direction vector");
    pcl::PCLPointCloud2::Ptr new_input_cl2 (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_input (new pcl::PointCloud<pcl::PointXYZ>);
    int n = static_cast<int>(0.004/cloud_data.gap) - 1;
    // int n = 1195;
    int reuse_count = static_cast<int>(0.0008/cloud_data.gap);
    int false_seg_count = static_cast<int>(0.0001/cloud_data.gap);
    ROS_INFO_ONCE("No. of scans needed: %i", n);
    pcl_conversions::toPCL(cloud_data.cloud, *new_input_cl2);
    pcl::fromPCLPointCloud2(*new_input_cl2, *new_input);
    if (raw_cloud_count < n && !cloud_data.last) 
    {
        if (raw_cloud_count == 0){
            first_message = cloud_data;
        }
        // vg_sampler.setInputCloud(new_input);
        // vg_sampler.filter(*new_input);
        // outrem.setInputCloud(new_input);
        // outrem.filter(*new_input);
        *raw_cl += *new_input;
        segment_sizes.push_back(new_input->size());
        dir_vec += current_vec;
        if (raw_cloud_count >= n - reuse_count) {
            reuse_message = cloud_data;
            reuse_vec += current_vec;
            *reuse_cl += *new_input;
        }
        raw_cloud_count++;
    }
    else
    {
        if(cloud_data.last)
            ROS_WARN("Last point recieved");
        // vg_sampler.setInputCloud(new_input);
        // vg_sampler.filter(*new_input);
        // outrem.setInputCloud(new_input);
        // outrem.filter(*new_input);
        *raw_cl += *new_input;

        *reuse_cl += *new_input;
        dir_vec += current_vec;
        segment_sizes.push_back(new_input->size());
        last_message = cloud_data;
        ROS_INFO("segmenting edges");
        unsigned int first_size = 0, last_size = 0, reuse_size_start = 0, reuse_size_end = 0;
        for (size_t i = 0; i < false_seg_count; i++)
        {
            first_size += segment_sizes.at(i);
            last_size += segment_sizes.at(segment_sizes.size() - false_seg_count + i);
        }
        for (size_t i = 0; i < reuse_count; i++)
        {
            reuse_size_start += segment_sizes.at(i);
            reuse_size_end += segment_sizes.at(segment_sizes.size() - reuse_count + i);
        }
        first_ind.resize(first_size);
        last_ind.resize(last_size);
        reuse_ind_start.resize(reuse_size_start);
        reuse_ind_end.resize(reuse_size_end);
        std::iota(first_ind.begin(), first_ind.end(), 0);
        std::iota(last_ind.begin(), last_ind.end(),raw_cl->size() - last_size);
        std::iota(reuse_ind_start.begin(), reuse_ind_start.end(), 0);
        std::iota(reuse_ind_end.begin(),reuse_ind_end.end(), raw_cl->size() - reuse_size_end);
        RawCloud raw_cloud;
        raw_cloud.LoadInCloud(raw_cl);
        raw_cloud.SetDownSample(true, 0.0001f);
        raw_cloud.SetStatOutRem(false, 10, 1.2);
        raw_cloud.SetFirstInd(first_ind);
        raw_cloud.SetLastInd(last_ind);
        // pcl::PointCloud<pcl::PointXYZ> cl;
        // pcl::copyPointCloud(*raw_cl, last_ind, cl);
        // pcl::io::savePCDFileASCII("/home/eshan/TestEG3D/src/testeg3d/data/last_inds.pcd", cl);
        // if(cloud_data.first)
        //     raw_cloud.SetFilterCriteria(false, true);
        // else if(cloud_data.last)
        //     raw_cloud.SetFilterCriteria(true, false);
        // else
        raw_cloud.SetFilterCriteria(true, true);
        // raw_cloud.VoxelDownSample(0.0001f);
        // raw_cloud.StatOutlierRemoval(20, 1.0);
        ROS_INFO("Filtered raw cloud");
        pcl::PointCloud<pcl::PointXYZ>::Ptr edges (new pcl::PointCloud<pcl::PointXYZ>);
        *edges = raw_cloud.FindEdgePoints(200, M_PI_2);
        raw_cloud.CorrectIndices(reuse_ind_end);
        // pcl::PointCloud<pcl::PointXYZ>::Ptr reuse_cl_start (new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::PointCloud<pcl::PointXYZ>::Ptr reuse_cl_end (new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::copyPointCloud(*edges, reuse_ind_start, *reuse_cl_start);
        // pcl::copyPointCloud(*edges, reuse_ind_end, *reuse_cl_end);
        // pcl::io::savePCDFileASCII("/home/eshan/TestEG3D/src/testeg3d/data/reuse_cl_start.pcd", *reuse_cl_start);
        // pcl::io::savePCDFileASCII("/home/eshan/TestEG3D/src/testeg3d/data/reuse_cl_end.pcd", *reuse_cl_end);
        ROS_INFO("Calculated edge points");
        pcl::io::savePCDFileASCII("/home/eshan/TestEG3D/src/testeg3d/data/edge_points.pcd", *edges);
        edge_cloud.SetScanDirection(dir_vec);
        edge_cloud.SetSensorCoords(first_message.sensor_x, first_message.sensor_y, first_message.sensor_z, first_message.sensor_position, "first");
        edge_cloud.SetSensorCoords(last_message.sensor_x, last_message.sensor_y, last_message.sensor_z, last_message.sensor_position, "last");
        edge_cloud.SetEndIndices(reuse_ind_end);
        edge_cloud.AddPoints(edges);
        ROS_INFO("Appended edge points");
        edge_cloud.ComputeVectors(20, 0.01, false);
        ROS_INFO("Computed point vectors");
        // edge_cloud.RemoveFalseEdges(0.001, true);
        // ROS_INFO("Tagged false edges");
        edge_cloud.ApplyRegionGrowing(20, 11.0 / 180.0 * M_PI, false);
        ROS_INFO("Segmented edges");

        if (cloud_data.last)
        {
            ROS_INFO("Saving edge points");
            edge_cloud.SaveCloud("/home/eshan/TestEG3D/src/testeg3d/data/edge_points.pcd");
            edge_cloud.AssembleRegions();
            ROS_INFO("Assembled segments");
            edge_cloud.CreateColouredCloud("/home/eshan/TestEG3D/src/testeg3d/data/segments.pcd");
            ROS_INFO("Saved segments");
        }
        raw_cl->clear();
        *raw_cl += *reuse_cl;
        reuse_cl->clear();
        dir_vec.setZero(3);
        dir_vec += reuse_vec;
        reuse_vec.setZero(3);
        raw_cloud_count = reuse_count;
        // first_message = reuse_message;
        segment_sizes.erase(segment_sizes.begin(), segment_sizes.end() - reuse_count);
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