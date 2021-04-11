// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

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


/* Templatized function for handling PCD Downsampling for any kind of input PCD data - XYZ PCD, XYZI PCD ... 
   The function implements this using the Voxel Grid Filter from PCL library which selects the centroid from a Grid region. Grid Size is defined using the filterRes
   Next 
*/
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    //Voxel Gridy
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloud_filtered);

    // Region of Interest Derivation
    typename pcl::PointCloud<PointT>::Ptr cloud_cropped(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> cropbox(true);
    cropbox.setMin(minPoint);
    cropbox.setMax(maxPoint);
    cropbox.setInputCloud(cloud_filtered);
    cropbox.filter(*cloud_cropped);



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr& inliers, typename pcl::PointCloud<PointT>::Ptr& cloud) 
{
    typename pcl::PointCloud<PointT>::Ptr Obstacle_Point_Cloud = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr Road_Point_Cloud(new pcl::PointCloud<PointT> ());
    pcl::ExtractIndices<PointT> extract;

    // inliers doesn't store the actual cloud points, but it just stores the entries/indexes of the point cloud which are on the same plane
    // the measurement for the same plane are done based on SAC segmentation which checks the  x and z measurments to assign the points in the input point cloud to 
    // the same plane based on a particular distance tolerance.
    for(int index:inliers->indices){
        Road_Point_Cloud->points.push_back(cloud->points[index]);
    }

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);  // setting the paramter to true informs the PCL extract function to extract only those points which are not inliers,  thus obstacle points
    extract.filter(*Obstacle_Point_Cloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(Obstacle_Point_Cloud, Road_Point_Cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
    ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    //define the Segmentation object
    pcl::SACSegmentation<PointT> seg;
    
    //Set the Model Hyperparameters
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // define the input cloud for the segmentation
    seg.setInputCloud(cloud);

    //List of inliers and coefficients that relevant to a plane in the point cloud
    seg.segment(*inliers, *coefficients);

    if((inliers->indices).size() == 0){
        std::cout << "couldn't find a planar segmentation for the input point cloud "<< std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    //Now that we have the set of inliers, we would use it to separate the point cloud
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    typename pcl::search::KdTree<PointT>::Ptr Obstacle_search_tree(new pcl::search::KdTree<PointT>);

    Obstacle_search_tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> clusterIndices; //Each clusterindex holds the indices of the original point cloud that belong to a particular cluster
    pcl::EuclideanClusterExtraction<PointT> obs_clus;
    obs_clus.setClusterTolerance(clusterTolerance); // allowed Distance between each point to belong to a cluster
    obs_clus.setMinClusterSize(minSize);  // Minimum points for the clusters
    obs_clus.setMaxClusterSize(maxSize);  // Maximum points allowed for the clusters
    obs_clus.setSearchMethod(Obstacle_search_tree);
    obs_clus.setInputCloud(cloud);
    obs_clus.extract(clusterIndices);

    for(auto cluster_index:clusterIndices){//Loop through all the clusters
        typename pcl::PointCloud<PointT>::Ptr Obs_cloud(new pcl::PointCloud<PointT>); // create a point cluster for each cluster object
        for(auto cloud_point:cluster_index.indices){
            Obs_cloud->points.push_back(cloud->points[cloud_point]);
        }
        Obs_cloud->width = Obs_cloud->points.size();
        Obs_cloud->height = 1;
        Obs_cloud->is_dense = true;

        clusters.push_back(Obs_cloud);
    }

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