#ifndef CALIBRATION_H
#define CALIBRATION_H

// ROS
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

// C++
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

//Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;
using namespace cv;

class lidar_calibration
{
public:
    /// @brief 类的construct函数
    /// @param
    /// @return void
    lidar_calibration(ros::NodeHandle &_nh);

    /// @brief 读取存放雷达坐标系下的XYZ数据txt文件
    /// @param
    /// @return void
    void read_lidar_txt();

    /// @brief 读取存放东北天坐标系下的XYZ数据txt文件
    /// @brief 每行第1个数字为N方向坐标（Y），第2个数字为E方向坐标（X），第3个数字为H方向坐标（Z）
    /// @param
    /// @return void
    void read_neh_txt();

    /// @brief 通过Kabsch算法求解3D点对匹配问题，得到从激光雷达坐标系到东北天坐标系的齐次变换矩阵
    /// @param
    /// @return void
    void pose_estimation_3d3d();

    /// @brief 计算匹配结果误差
    /// @param
    /// @return void
    void compute_error();

    /// @brief 将匹配结果可视化并提供在pcl_viewer中选取点获取坐标的功能
    /// @param
    /// @return void
    void match_viewer();

    /// @brief 订阅2.4m处激光雷达的点云的回调函数，利用外参矩阵将2.4m处激光雷达点云变换到东北天坐标系下
    /// @param
    /// @return void
    void low_callback(const sensor_msgs::PointCloud2ConstPtr &input);

    /// @brief 订阅7m处激光雷达的点云的回调函数，利用外参矩阵将7m处激光雷达点云变换到东北天坐标系下
    /// @param
    /// @return void
    void high_callback(const sensor_msgs::PointCloud2ConstPtr &input);

    /// @brief 将旋转矩阵转化为欧拉角并检查计算是否正确
    /// @param
    /// @return void
    void RotationMatrix2RPY();

private:
    ros::NodeHandle nh; // ROS句柄
    ros::Publisher pub_low; // 发布2.4m处激光雷达变换后的点云信息
    ros::Publisher pub_high;// 发布7m处激光雷达变换后的点云信息
    ros::Subscriber sub_low; // 订阅2.4m处激光雷达点云信息
    ros::Subscriber sub_high;// 订阅7m处激光雷达点云信息

    vector<pcl::PointXYZ> global_points;// 东北天坐标系下点的坐标
    vector<pcl::PointXYZ> lidar_points; // 雷达坐标系下点的坐标

    Eigen::Matrix4f transformation_matrix; // Kabsch匹配得到的齐次变换矩阵
    Eigen::Matrix3f R_;  // Kabsch匹配得到的旋转矩阵
    Eigen::Vector3f t_;// Kabsch匹配得到的平移向量

    Eigen::Matrix4f transformation_matrix_low_to_global; // 从2.4m处雷达坐标系变换到东北天(全局)坐标系的齐次变换矩阵
    Eigen::Matrix4f transformation_matrix_high_to_global; // 从7m处雷达坐标系变换到东北天(全局)坐标系的齐次变换矩阵
    Eigen::Vector3f euler;                                //欧拉角Y->euler[0],P->euler[1],R->euler[2],

    string neh_txt = "/home/philip/catkin_ws/src/lidar_calibration/data/high_lidar/neh.txt";     // 待读取的东北天(全局)坐标系下点的坐标
    string lidar_txt = "/home/philip/catkin_ws/src/lidar_calibration/data/high_lidar/lidar.txt"; // 待读取的雷达坐标系下点的坐标
};

#endif