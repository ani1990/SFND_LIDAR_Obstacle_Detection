/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

/* Function for rendering the point cloud detection pipeline on a simulated point cloud data 
   Here the PCD data is in the form of XYZ
*/
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    
    Lidar lidar(cars, 0);   // creating LIDAR object on stack
    //Lidar *lidar = new Lidar(cars,0);  // creating LIDAR object on heap
    pcl::PointCloud<pcl::PointXYZ>::Ptr Pointcloud = lidar.scan();

    // Vect3 origin for renderRays will be the position at which the LIDAR object is created, since all the rays are casted from this position
    // this is setup during the creation of LIDAR object above
    //renderRays(viewer, lidar.position, Pointcloud);
    //renderPointCloud(viewer,Pointcloud, "PointCloud");
    ProcessPointClouds<pcl::PointXYZ> *pointProcessor = new ProcessPointClouds<pcl::PointXYZ>;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segresult =  pointProcessor->SegmentPlane(Pointcloud, 100, 0.2);
    //segresult.first : Obstacles, segresult.second : Road
    renderPointCloud(viewer,segresult.second, "Road", Color(1,1,1));
    //renderPointCloud(viewer,segresult.first, "Obstacle", Color(1,0,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> CloudClusters = pointProcessor->Clustering(segresult.first, 1.0, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for(auto cloud:CloudClusters){
        std::cout << "Found "<< cloud->points.size() << " points in cluster " << clusterId << std::endl;
        //renderPointCloud(viewer,cloud, "ObstCloud" + std::to_string(clusterId), colors[clusterId]);
        
        // here we are passing by value the pointer cloud, but it wont have problems with dangling pointer
        // since the pcl::PointCloud<PointT>::Ptr is a shared pointer..thus reference counting is done for the memory region
        // and the memory is freed at the end when reference counting goes to zero, i.e all the cloud pointers go out
        // of scope
        Box box = pointProcessor->BoundingBox(cloud);
        renderBox(viewer, box, clusterId, colors[clusterId]);
        clusterId++;
    }
  

}


/* function to implement the LIDAR point cloud detection pipeline on a Real PCD data from a car
   Here the PCD data is in the form of XYZI 
 */
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr &pointcloud){
    //renderPointCloud(viewer, streetPCD, "PCD");
    pcl::PointCloud<pcl::PointXYZI>::Ptr streetPCD_filtered = pointProcessor->FilterCloud(pointcloud, 0.2f, Eigen::Vector4f(0.0f, 0.0f, 0.0f,1.0f), Eigen::Vector4f(20.0f, 20.0f, 20.0f,1.0f));
    //renderPointCloud(viewer, streetPCD_filtered, "StreetPCD_Filtered");

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segresult =  pointProcessor->SegmentPlane(streetPCD_filtered, 50, 0.2);
    renderPointCloud(viewer,segresult.second, "Road", Color(1,1,1));
    //renderPointCloud(viewer,segresult.first, "Obstacle", Color(1,0,0));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> CloudClusters = pointProcessor->Clustering(segresult.first, 1.0, 20, 100);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for(auto cloud:CloudClusters){
        std::cout << "Found "<< cloud->points.size() << " points in cluster " << clusterId << std::endl;
        //renderPointCloud(viewer,cloud, "ObstCloud" + std::to_string(clusterId), colors[clusterId]);
        
        // here we are passing by value the pointer cloud, but it wont have problems with dangling pointer
        // since the pcl::PointCloud<PointT>::Ptr is a shared pointer..thus reference counting is done for the memory region
        // and the memory is freed at the end when reference counting goes to zero, i.e all the cloud pointers go out
        // of scope
        Box box = pointProcessor->BoundingBox(cloud);
        renderBox(viewer, box, clusterId, colors[clusterId]);
        clusterId++;
    }

}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI> *pointProcessor = new ProcessPointClouds<pcl::PointXYZI>;
    std::vector<boost::filesystem::path> streamPCD = pointProcessor->streamPcd("../src/sensors/data/pcd/data_1/");
    
    if(!streamPCD.empty()){
        while (!viewer->wasStopped ()){
            for(auto PCD:streamPCD){
                viewer->removeAllPointClouds();
                viewer->removeAllShapes();
                pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZI>);
                pointCloud = pointProcessor->loadPcd(PCD.string());
                cityBlock(viewer, pointProcessor, pointCloud);
                viewer->spinOnce ();
            }            
            
        } 
    }
}