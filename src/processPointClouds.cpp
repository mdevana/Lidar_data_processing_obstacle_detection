// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <stdio.h>
#include <unordered_set>
//#include "kdtree.h"
#include "processCluster.h"
#include "processCluster.cpp"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> voxgrid;
    voxgrid.setInputCloud(cloud);
    voxgrid.setLeafSize(filterRes,filterRes,filterRes);
    voxgrid.filter(*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloud_roi(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> ROI(true);
    ROI.setMin(minPoint);
    ROI.setMax(maxPoint);
    ROI.setInputCloud(cloud_filtered);
    ROI.filter(*cloud_roi);

    std::vector<int> box_indices;
    pcl::CropBox<PointT> roof_box(true);
    roof_box.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roof_box.setMax(Eigen::Vector4f(3,1.7,-0.4,1));
    roof_box.setInputCloud(cloud_roi);
    roof_box.filter(box_indices);

    

    pcl::PointIndices::Ptr inliers_box(new pcl::PointIndices);
    typename pcl::PointCloud<PointT>::Ptr cloud_roof_box(new pcl::PointCloud<PointT>);
    inliers_box->indices = box_indices;

    pcl::ExtractIndices<PointT> flip;
    flip.setInputCloud(cloud_roi);
    flip.setIndices(inliers_box);
    //Trap box for car
    flip.setNegative(false);
    flip.filter(*cloud_roof_box);
    car_box = BoundingBox(cloud_roof_box);

    // extract all points except top of car
    flip.setNegative(true);
    flip.filter(*cloud_roi);

    
    


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_roi;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    
    for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZI point = cloud->points[index];
		if ( std::count(inliers->indices.begin(),inliers->indices.end(),index) )
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);

    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices);
    
    std::unordered_set<int> inliersResult;
	srand(time(NULL));
  
  	while(maxIterations--){
      
      std::unordered_set<int> inliers;
	  while (inliers.size()<3){
		
			inliers.insert((rand())%(cloud->points.size()));

		}
      	float x1;
		float x2;
        float x3;
		float y1;
		float y2;
      	float y3;
		float z1;
		float z2;
		float z3;
      
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;

		itr++;

		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
      
        itr++;
      
        x3 = cloud->points[*itr].x;
	    y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

		float A = (y2-y1) * (z3-z1) - (z2-z1) * (y3-y1);
		float B = (z2-z1) * (x3-x1) - (x2-x1) * (z3-z1);
		float C = (x2-x1) * (y3-y1) - (y2-y1) * (x3-x1);
      	float D = -1 * (A * x1 + B * y1 + C * z1);
        
      for(int i=0;i<cloud->points.size();i++){
        if(inliers.count(i)>0)
          continue;
        
        float x=cloud->points[i].x;
		float y=cloud->points[i].y;
        float z=cloud->points[i].z;
        
        float dist = std::fabs(A * x + B * y + C * z + D  ) / std::sqrt( A * A + B * B + C * C) ;
        
        if (dist <= distanceThreshold)

          inliers.insert(i);
        
        
        
      }
      
      if (inliers.size() > inliersResult.size()){
        inliersResult = inliers;
      }

    }

    
    std::vector<int> inlier_vect(inliersResult.size());
    inlier_vect.assign(inliersResult.begin(),inliersResult.end());
	inliers_ptr->indices=inlier_vect;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers_ptr,cloud);

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    std::cout << " starting clustering" << std::endl;
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    
    //create Process cluster object and call clustering
    ProcessCluster<pcl::PointXYZI> cluster_ops(cloud); 
    clusters = cluster_ops.euclideanCluster(clusterTolerance,minSize,maxSize);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

