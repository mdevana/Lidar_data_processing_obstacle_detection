// PCL lib Functions for processing point clouds 



#include "processCluster.h"



    //constructor
    template<typename PointT>
    ProcessCluster<PointT>::ProcessCluster(typename pcl::PointCloud<PointT>::Ptr cloud) {
        tree = new KdTree;
        cloud_local = cloud;
    
        for (int i=0; i<cloud->points.size(); i++) 
    	    tree->insert(PointT2vector(cloud->points[i]),i); 
    }

    //deconstructor
    template<typename PointT>
    ProcessCluster<PointT>::~ProcessCluster(){};

template<typename PointT>
std::vector<float> ProcessCluster<PointT>::PointT2vector(PointT p){

	std::vector<float> vect_point;
	vect_point.push_back(p.x);
	vect_point.push_back(p.y);
	vect_point.push_back(p.z);
	vect_point.push_back(1);
	
	return(vect_point);

}
   

    
template<typename PointT>
void ProcessCluster<PointT>::proximity(int n,std::vector<bool>& track_pro_pts,std::vector<int>& cluster_one, float distanceTol){

	
	

	track_pro_pts[n] = true;
	cluster_one.push_back(n);
	std::vector<int> nearby_pts = tree->search(PointT2vector(cloud_local->points[n]),distanceTol);

	for(int id : nearby_pts){

		if(!track_pro_pts[id]) {

			proximity(id,track_pro_pts,cluster_one,distanceTol);

		}

	}


	

}
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessCluster<PointT>::euclideanCluster(float distanceTol, int min_size, int max_size)
{

	

	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

	std::vector<bool> track_processed(cloud_local->points.size(),false);
    
	for(int i = 0;i<cloud_local->points.size();i++){

		if (track_processed[i]){
			continue;
		}
		
		std::vector<int> single_cluster;
		

		proximity(i,track_processed,single_cluster,distanceTol);

		

		typename pcl::PointCloud<PointT>::Ptr pcl_cluster(new pcl::PointCloud<PointT>());

		for(int point_id:single_cluster){
			pcl::PointXYZI point = cloud_local->points[point_id];
			pcl_cluster->points.push_back(point);
		}
        
		int cloud_size = pcl_cluster->points.size();
        if (cloud_size > min_size && cloud_size <=max_size)
		clusters.push_back(pcl_cluster);
		

	}
    
	return clusters;

}
    
    
  

