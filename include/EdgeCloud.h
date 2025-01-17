//
// Created by eshan on 11.11.22.
//

#ifndef EG3D_EDGECLOUD_H
#define EG3D_EDGECLOUD_H

#pragma once
#include "BaseCloud.h"
#include "BoundingBox.h"
#include "Region2D.h"


#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>


class EdgeCloud : public BaseCloud{
private:
    std::vector<int> edge_points_indices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_points;
    pcl::search::Search<pcl::PointXYZ>::Ptr tree;
    std::vector<std::vector<int>> clusters;
    std::vector<unsigned int> num_pts_in_segment;
    std::vector<int> point_labels;
    std::vector<int> previous_seeds;
    Eigen::Vector3f first_coord[3], second_coord[3];

    float seg_tag_thresh;
    int total_num_of_segmented_pts;
    int total_num_of_segments;
    unsigned int previous_size;
    float angle_thresh;
    bool is_appended;
    bool override_cont;

    std::unordered_map<unsigned long, pcl::Indices> neighbours_map;
    std::unordered_map<unsigned long, Eigen::Vector3f> vectors_map;
    std::unordered_map<int, Eigen::Vector3f> segment_vectors;
    std::unordered_map<int, bool> finished_segments;
    std::unordered_map<unsigned long, bool> false_edges;

    Eigen::Vector3f scan_direction;

    void Init();

    int GrowSegment(const int &initial_seed, const int &segment_id, const int &neighbours_k,
                    Eigen::Vector3f &segment_vector);
    bool CheckPoint(const int &current_seed, const int &neighbour, bool &is_a_seed);
    int ExtendSegment(const int &new_point, const int &neighbour, const int &segment_id, const int &neighbours_k,
                      Eigen::Vector3f &segment_vector);
    bool IsFinished(const int &label);

public:
    EdgeCloud();
    EdgeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    EdgeCloud(const std::vector<int> &edge_indices, const pcl::PointCloud<pcl::PointXYZ>::Ptr& parent_cloud);

    void SegmentEdges(const int &neighbours_K, const float &dist_thresh, const float &angle_thresh, const bool &sort,
                      const bool &override_cont);
    void ComputeVectors(const int &neighbours_K, const float &dist_thresh, const bool &override);
    void RemoveFalseEdges(float region_width);
    void
    ApplyRegionGrowing(const int &neighbours_k, const float &angle_thresh, const bool &sort);
    void CreateColouredCloud(const std::string &path);
    void AddPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &new_points);
    void SetTagThresh(const float &seg_tag_thresh);
    void SetScanDirection(const Eigen::Vector3f &scan_direction);
    void SetBoundingCoordinates(const Eigen::Vector3f first[0], const Eigen::Vector3f second[3]);
    void AssembleRegions();

};

inline bool Compare(std::pair<unsigned long, int> i, std::pair<unsigned long, int> j) {
    return (i.first > j.first);
};


#endif //EG3D_EDGECLOUD_H
