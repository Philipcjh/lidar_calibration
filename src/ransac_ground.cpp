#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>

int  main(int argc, char *argv[])
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_low(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_high(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/philip/catkin_ws/src/lidar_calibration/data/two_lidars/lg.pcd", *cloud_low) == -1)
    {
        PCL_ERROR("Error loading cloud.pcd.\n");
        return -1;
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/philip/catkin_ws/src/lidar_calibration/data/two_lidars/hg.pcd", *cloud_high) == -1)
    {
        PCL_ERROR("Error loading cloud.pcd.\n");
        return -1;
    }

    for (int i = 0; i < cloud_low->size();++i)
    {
        cloud->push_back((*cloud_low)[i]);
    }

    for (int i = 0; i < cloud_high->size(); ++i)
    {
        cloud->push_back((*cloud_high)[i]);
    }

    std::vector<int> inliers;

    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud)); 
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane); 
    ransac.setDistanceThreshold(0.05);  
    ransac.computeModel();
    ransac.getInliers(inliers);

    Eigen::VectorXf coefficient;
	ransac.getModelCoefficients(coefficient);
    cout << "共有" << cloud->size() << "个点云，拟合后平面点云为" << inliers.size() << "个" << endl;
    cout << "拟合平面方程为:" << coefficient[0] << "x + " << coefficient[1] << "y + " << coefficient[2] << "z + " << coefficient[3] << " = 0" << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ransac(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &idx : inliers)
    {
        cloud_ransac->push_back((*cloud)[idx]);
    }

    pcl::PointXYZ min_p, max_p;
    pcl::getMinMax3D(*cloud_ransac, min_p, max_p);
    cout << "z_min = " << min_p.z << ", z_max = " << max_p.z << endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("result"));
    viewer->addCoordinateSystem(); // R :X, G:Y, B:Z

    viewer->addPointCloud<pcl::PointXYZ>(cloud, "origin");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "origin"); // white
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "origin");

    viewer->addPointCloud<pcl::PointXYZ>(cloud_ransac, "ransac");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "ransac"); // green
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "ransac");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return 0;
}