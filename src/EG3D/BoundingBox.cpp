//
// Created by chl-es on 10.12.22.
//

#include <pcl/point_cloud.h>
#include "BoundingBox.h"

BoundingBox::BoundingBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_section) {
    
    Eigen::Vector4f pca_centroid;
    pcl::compute3DCentroid(*cloud_section, pca_centroid);
    // Eigen::Matrix3f covariance;
    // pcl::computeCovarianceMatrixNormalized(*cloud_section, pca_centroid, covariance);
    // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    // Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();
    // eigen_vectors.col(2) = eigen_vectors.col(0).cross(eigen_vectors.col(1));

    pcl::PointCloud<pcl::PointXYZ>::Ptr pca_projection (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud_section);
    pca.project(*cloud_section, *pca_projection);
    Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();

    // Eigen::MatrixX4f transformer(Eigen::Matrix4f::Identity());
    // transformer.block<3,3>(0,0) = eigen_vectors.transpose();
    // transformer.block<3,1>(0,3) = -1.f * (transformer.block<3,3>(0,0) * pca_centroid.head<3>());
    // pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::transformPointCloud(*cloud_section, *projected_cloud, transformer);
    pcl::io::savePCDFileASCII("/home/chl-es/TestEG3D/src/testeg3d/data/projected_cloud.pcd", *pca_projection);
    pcl::PointXYZ min, max;
    pcl::getMinMax3D(*pca_projection, min, max);
    const Eigen::Vector3f diagonal = 0.5f*(max.getVector3fMap() + min.getVector3fMap());

    const Eigen::Quaternionf b_box_quaternion(eigen_vectors);
    const Eigen::Vector3f b_box_translation = eigen_vectors * diagonal + pca_centroid.head<3>();
    // const Eigen::Quaternionf b_box_quat_norm = b_box_quaternion.normalized();
    const Eigen::Matrix3f rotation_matrix = b_box_quaternion.toRotationMatrix();

    Eigen::Vector3f depth_vec(max.x - min.x, 0.0, 0.0), width_vec(0.0, max.y - min.y, 0.0), height_vec(0.0, 0.0, max.z - min.z);
    // depth_vec = b_box_translation + (rotation_matrix * depth_vec);
    // width_vec = b_box_translation + (rotation_matrix * width_vec);
    // height_vec = b_box_translation + (rotation_matrix * height_vec);
    Eigen::Vector3f P1_vec = b_box_translation + (rotation_matrix * min.getVector3fMap()), P8_vec = b_box_translation + (rotation_matrix * max.getVector3fMap()),
                    P2_vec = b_box_translation + (rotation_matrix * (min.getVector3fMap() + depth_vec)), P3_vec = b_box_translation + (rotation_matrix * (min.getVector3fMap() + width_vec)),
                    P4_vec = b_box_translation + (rotation_matrix * (min.getVector3fMap() + depth_vec + width_vec));
    depth = P8.x - P1.x; width = P8.y - P1.y; height = P8.z - P1.z;

                    // P5_vec = b_box_translation + (rotation_matrix * (min.getVector3fMap() + height_vec)),
                    // P6_vec = b_box_translation + (rotation_matrix * (max.getVector3fMap() - width_vec )), P7_vec = b_box_translation + (rotation_matrix * (max.getVector3fMap() - depth_vec));
    P1 = {P1_vec.x(), P1_vec.y(), P1_vec.z()};
    P2 = {P2_vec.x(), P2_vec.y(), P2_vec.z()};
    P3 = {P3_vec.x(), P3_vec.y(), P3_vec.z()};
    P4 = {P4_vec.x(), P4_vec.y(), P4_vec.z()};
    P5 = {P1_vec.x(), P1_vec.y(), P1_vec.z() + height};
    P6 = {P2_vec.x(), P2_vec.y(), P2_vec.z() + height};
    P7 = {P3_vec.x(), P3_vec.y(), P3_vec.z() + height};
    P8 = {P8_vec.x(), P8_vec.y(), P8_vec.z()};
    //Dr Subramaniam Swami

    // depth = rotation_matrix * depth; width = rotation_matrix * width; height = rotation_matrix * height;
    // Eigen::Vector3f P1_vec = rotation_matrix * ();
    // pcl::PointXYZ P1(P1_vec.x(), P1_vec.y(), P1_vec.z());

    // pcl::MomentOfInertiaEstimation<pcl::PointXYZ> bbox_extractor;
    // bbox_extractor.setInputCloud(cloud_section);
    // // bbox_extractor.setNormalizePointMassFlag(true);
    // bbox_extractor.compute();

    // pcl::PointXYZ min_p, max_p, position_p;
    // Eigen::Matrix3f rotation_matrix;

    // bbox_extractor.getOBB(min_p, max_p, position_p, rotation_matrix);
    // rotation_matrix.normalize();

    // Eigen::Vector3f depth(max_p.x - min_p.x, 0.0, 0.0), width(0.0, max_p.y - min_p.y, 0.0), height(0.0, 0.0, max_p.z - min_p.z);
    // // Eigen::Vector3f depth_adjusted = rotation_matrix * depth, width_adjusted = rotation_matrix * width, height_adjusted = rotation_matrix * height;
    // Eigen::Vector3f P1_vec = rotation_matrix * (position_p.getVector3fMap() + min_p.getVector3fMap());
    // Eigen::Vector3f P8_vec = rotation_matrix * (position_p.getVector3fMap() + max_p.getVector3fMap());
    // P1 = {P1_vec.x(), P1_vec.y(), P1_vec.z()};
    // P8 = {P8_vec.x(), P8_vec.y(), P8_vec.z()};

    
    // pcl::visualization::PCLVisualizer::Ptr visu (new pcl::visualization::PCLVisualizer());
    // int mesh_vp_1, mesh_vp_2, mesh_vp_3, mesh_vp_4;
    // visu->createViewPort (0.0, 0.5, 0.5, 1.0,  mesh_vp_1);
    // visu->createViewPort (0.5, 0.5, 1.0, 1.0,  mesh_vp_2);
    // visu->createViewPort (0.0, 0, 0.5, 0.5,  mesh_vp_3);
    // visu->createViewPort (0.5, 0, 1.0, 0.5, mesh_vp_4);
    // visu->addPointCloud(cloud_section, "bboxedCloud", mesh_vp_3);
    // visu->addCube(b_box_transform, b_box_quaternion, depth, width, height, "bbox", mesh_vp_3);
    // while(!visu->wasStopped()) 
    // {
    //     visu->spinOnce(100);
    // }

    InitPoints();
}

void BoundingBox::InitPoints() {
    // P2 = {P1.x + depth, P1.y, P1.z};
    // P3 = {P1.x, P1.y + width, P1.z};
    // P4 = {P1.x + depth, P1.y + width, P1.z};
    // P5 = {P1.x, P1.y, P1.z + height};
    // P6 = {P1.x + depth, P1.y, P1.z + height};
    // P7 = {P1.x, P1.y + width, P1.z + height};

    Eigen::Vector3f P1_vec, P2_vec, P3_vec, P4_vec, P5_vec, P6_vec, P7_vec, P8_vec;
    P1_vec = P1.getVector3fMap();
    P2_vec = P2.getVector3fMap();
    P3_vec = P3.getVector3fMap();
    P4_vec = P4.getVector3fMap();
    P5_vec = P5.getVector3fMap();
    P6_vec = P6.getVector3fMap();
    P7_vec = P7.getVector3fMap();
    P8_vec = P8.getVector3fMap();

    float ang_p1_p2_p3 = std::acos(std::abs((P2_vec - P1_vec).dot(P3_vec - P1_vec) / ((P2_vec - P1_vec).norm() * (P3_vec - P1_vec).norm()))) * 180.0 / M_PI;
    float ang_p5_p6_p7 = std::acos(std::abs((P6_vec - P5_vec).dot(P7_vec - P5_vec) / ((P6_vec - P5_vec).norm() * (P7_vec - P5_vec).norm()))) * 180.0 / M_PI;
    float ang_p7_p8_p5 = std::acos(std::abs((P8_vec - P7_vec).dot(P7_vec - P5_vec) / ((P8_vec - P7_vec).norm() * (P7_vec - P5_vec).norm()))) * 180.0 / M_PI;

    std::cout << "Angle between p2p1 and p3p1: " << ang_p1_p2_p3 << std::endl;
    std::cout << "Angle between p6p5 and p7p5: " << ang_p5_p6_p7 << std::endl;
    std::cout << "Angle between p7p5 and p7p8: " << ang_p7_p8_p5 << std::endl;
}

void BoundingBox::GetPoints(pcl::PointXYZ *points) {
    pcl::PointXYZ points_ [8] = {P1, P2, P3, P4, P5, P6, P7, P8};
    for (int i = 0; i < 8; i++)
        *(points + i) = points_[i];
    
}