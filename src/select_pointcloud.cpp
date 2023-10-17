#include <string>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

int selected_num = 0; // 在pcl_viewer中选中的点的数量

struct CallbackArgs
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void pickPointCallback(const pcl::visualization::PointPickingEvent &event, void *args)
{
    CallbackArgs *data = (CallbackArgs *)args;
    if (event.getPointIndex() == -1)
        return;
    selected_num++;
    pcl::PointXYZ current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);
    data->clicked_points_3d->points.push_back(current_point);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(data->clicked_points_3d, 255, 0, 0);
    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10,
                                                      "clicked_points");
    cout << "Point" << selected_num<<" : "<< current_point.x << "," << current_point.y << "," << current_point.z << endl;
}

void plot_and_choose(string &path)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_strong(new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(path, *cloud) == -1)
    {
        PCL_ERROR("Error loading pcd.\n");
        return;
    }

    for (int i = 0; i < cloud->size();++i)
    {
        if ((*cloud)[i].intensity>150)
        {
            cloud_strong->push_back((*cloud)[i]);
        }
    }

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("select"));
    viewer->addCoordinateSystem(); // R :X, G:Y, B:Z

    viewer->addPointCloud<pcl::PointXYZI>(cloud, "origin");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "origin"); // white
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2, "origin");

    viewer->addPointCloud<pcl::PointXYZI>(cloud_strong, "strong");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "strong"); // green
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "strong");

    CallbackArgs cb_args;
    pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d(new pcl::PointCloud<pcl::PointXYZ>);
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
    viewer->registerPointPickingCallback(pickPointCallback, (void *)&cb_args);
    cout << "Shift+click to select points, press q to quit" << endl;

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}


int  main(int argc, char *argv[])
{
    // 待手动筛选点的原始点云文件路径,根据需要修改
    string pcd_path = "/home/philip/catkin_ws/src/lidar_calibration/data/high_lidar/10/board.pcd";

    plot_and_choose(pcd_path);
    return 0;
}