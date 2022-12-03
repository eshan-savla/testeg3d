#include "ScanProcessor.h"


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

void ScanProcessor::msgCallBack(const testeg3d::CloudData& cloud_data) {
    std::vector<float> quartenion = cloud_data.quartenion;
    std::vector<float> point_vector = cloud_data.point_vector;
    Eigen::Vector3f current_vec = getDirectionVector(quartenion, point_vector);
    ROS_INFO("Calculated direction vector");
    pcl::PCLPointCloud2::Ptr new_input (new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(cloud_data.cloud, *new_input);
    if (raw_cloud_count < 8)
    {
        *raw_cl2 += *new_input;
        dir_vec += current_vec;
        ROS_INFO("Appended raw input");
        raw_cloud_count++;
    }
    else
    {
        ROS_INFO("segmenting edges");
        ROS_DEBUG_STREAM_ONCE("Angle btw scan direction vectors: " << std::abs(dir_vec.dot(current_vec))/(dir_vec.norm() * current_vec.norm()) << std::endl);
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cl1 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*raw_cl2, *raw_cl1);
        RawCloud raw_cloud;
        raw_cloud.LoadInCloud(raw_cl1);
        raw_cloud.StatOutlierRemoval(50, 0.01);
        raw_cloud.VoxelDownSample(0.0005f);
        ROS_INFO("Filtered raw cloud");
        pcl::PointCloud<pcl::PointXYZ>::Ptr edges (new pcl::PointCloud<pcl::PointXYZ>);
        *edges = raw_cloud.FindEdgePoints(50, 10.0 / 180.0 * M_PI);
        ROS_INFO("Calculated edge points");
        edge_cloud.SetScanDirection(dir_vec);
        edge_cloud.AddPoints(edges);
        ROS_INFO("Appended edge points");
        edge_cloud.ComputeVectors(50, 0.01, false);
        ROS_INFO("Computed point vectors");
        edge_cloud.ApplyRegionGrowing(50, 0.01, true);
        ROS_INFO("Segmented edges");

        dir_vec.setZero(3);
        raw_cloud_count = 0;

    }
}

Eigen::Vector3f ScanProcessor::getDirectionVector(std::vector<float>& quartenion, std::vector<float>& start_point) {
    Eigen::Quaternion<float> quat(quartenion[0], quartenion[1], quartenion[2], quartenion[3]);
    Eigen::Vector3f scan_dir(start_point.data());
    quat.normalize();
    Eigen::Matrix3f rot_mat;
    rot_mat = quat.toRotationMatrix();
    scan_dir = rot_mat * scan_dir;
    return scan_dir;
}

ScanProcessor::~ScanProcessor() {
    edge_cloud.AssembleRegions();
    edge_cloud.CreateColouredCloud("data/segments.pcd");
}
int main(int argc, char ** argv) {
    ros::init(argc, argv, "scan_processor");
    ros::NodeHandle nh;
    ROS_INFO("Start scan processor node");
    ScanProcessor scan_processor(&nh);

    ros::spin();
    return 0;
}