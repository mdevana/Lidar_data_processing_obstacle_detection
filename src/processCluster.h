// PCL lib Functions for processing point clouds 

#ifndef PROCESSCLUSTER_H_
#define PROCESSCLUSTER_H_


#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
/*
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"*/
#include "kdtree.h"

template<typename PointT>
class ProcessCluster {
public:
    KdTree* tree;
    typename pcl::PointCloud<PointT>::Ptr cloud_local;

    //constructor
    ProcessCluster(typename pcl::PointCloud<PointT>::Ptr);
    //deconstructor
    ~ProcessCluster();

   
    void proximity(int n,std::vector<bool>& track_pro_pts,std::vector<int>& cluster_one, float distanceTol);


    std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(float,int,int);
    std::vector<float> PointT2vector(PointT p);
  
};
#endif /* PROCESSCLUSTER_H_ */