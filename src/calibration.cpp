#include "lidar_calibration/calibration.h"

lidar_calibration::lidar_calibration(ros::NodeHandle &_nh)
{
    nh = _nh;

    transformation_matrix_low_to_global << 0.976111, 0.0187844, 0.216462, -8.84625,
                                                                                        -0.020873, 0.999755, 0.00736652, 8.25426,
                                                                                        -0.216271, -0.0117087, 0.976264, 3.23757,
                                                                                        0, 0, 0, 1;

    // euler angle = [R : 3.1296 P: 2.9236 Y : 3.12021]
    // t = [- 8.84625; 8.25426,3.23757];

    transformation_matrix_high_to_global << 0.96271, 0.0440072, 0.266931, -7.169,
                                                                                        -0.0378209, 0.998884, -0.0282752, 8.61001,
                                                                                        -0.267878, 0.0171252, 0.963301, 9.68827,
                                                                                        0, 0, 0, 1;

    // euler angle = [R : -3.12382 P: 2.8704 Y : 3.10233]
    // t = [- 7.169; 8.61001,9.68827];

    pub_low = nh.advertise<sensor_msgs::PointCloud2>("/low/transformed_points", 1);
    pub_high = nh.advertise<sensor_msgs::PointCloud2>("/high/transformed_points", 1);

    sub_low = nh.subscribe("/low/rslidar_points", 1, &lidar_calibration::low_callback, this);
    sub_high = nh.subscribe("/high/rslidar_points", 1, &lidar_calibration::high_callback, this);
}

void lidar_calibration::read_lidar_txt()
{
    ifstream fin(lidar_txt);
    if (!fin)
    {
        cerr << " !!!!!!!!!!!!!!!!!!!!!!No Lidar Data!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
        return;
    }

    string line;
    pcl::PointXYZ point;
    vector<string> points_string;
    while (getline(fin, line))
    {
        istringstream sin(line);
        vector<string> fields;
        string field;
        while (getline(sin, field, ','))
        {
            fields.push_back(field);
        }

        stringstream ss;
        ss << fields[0];
        ss >> point.x;
        ss.clear();

        ss << fields[1];
        ss >> point.y;
        ss.clear();

        ss << fields[2];
        ss >> point.z;
        ss.clear();
        lidar_points.push_back(point);
    }

    // cout << "Lidar points:"<< endl;
    // for (vector<pcl::PointXYZ>::iterator iter = lidar_points.begin(); iter != lidar_points.end(); iter++)
    // {
    //     printf(" x = %f, y = %f, z = %f\n", (*iter).x, (*iter).y, (*iter).z);
    // }
    // cout << "******************************************************" << endl;
}

void lidar_calibration::read_neh_txt()
{
    ifstream fin(neh_txt);
    if (!fin)
    {
        cerr << " !!!!!!!!!!!!!!!!!!!!!!No Neh Data!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
        return;
    }

    string line;
    pcl::PointXYZ point;
    vector<string> points_string;
    while (getline(fin, line))
    {
        istringstream sin(line);
        vector<string> fields;
        string field;
        while (getline(sin, field, ','))
        {
            fields.push_back(field);
        }

        stringstream ss;
        ss << fields[0];
        ss >> point.y;
        ss.clear();

        ss << fields[1];
        ss >> point.x;
        ss.clear();

        ss << fields[2];
        ss >> point.z;
        ss.clear();

        global_points.push_back(point);
    }

    // cout << "Global points:" << endl;
    // for (vector<pcl::PointXYZ>::iterator iter = global_points.begin(); iter != global_points.end(); iter++)
    // {
    //     printf(" x = %f, y = %f, z = %f\n", (*iter).x, (*iter).y, (*iter).z);
    // }
    // cout << "******************************************************" << endl;
}

void lidar_calibration::pose_estimation_3d3d()
{
    vector<Point3f> pts1, pts2;
    for (int i = 0; i < global_points.size();++i)
    {
        pts1.push_back(Point3f(global_points[i].x,global_points[i].y,global_points[i].z));
        pts2.push_back(Point3f(lidar_points[i].x, lidar_points[i].y, lidar_points[i].z));
    }

    Point3f p1, p2; // center of mass
    int N = pts1.size();
    for (int i = 0; i < N; i++)
    {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = Point3f(Vec3f(p1) / N);
    p2 = Point3f(Vec3f(p2) / N);
    vector<Point3f> q1(N), q2(N); // remove the center
    for (int i = 0; i < N; i++)
    {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1*q2^T
    Matrix3f W = Matrix3f::Zero();
    for (int i = 0; i < N; i++)
    {
        W += Vector3f(q2[i].x, q2[i].y, q2[i].z) * Vector3f(q1[i].x, q1[i].y, q1[i].z).transpose();
    }

    // SVD on W
    JacobiSVD<Matrix3f> svd(W, ComputeFullU | ComputeFullV);
    Matrix3f U = svd.matrixU();
    Matrix3f V = svd.matrixV();

    Matrix3f E;
    E << 1, 0, 0, 0, 1, 0, 0, 0, (V * (U.transpose())).determinant();

    R_ = V * E * (U.transpose());

    t_ = Vector3f(p1.x, p1.y, p1.z) - R_ * Vector3f(p2.x, p2.y, p2.z);

    Isometry3f T = Isometry3f::Identity();
    T.rotate(R_);
    T.pretranslate(t_);
    transformation_matrix =  T.matrix();

    cout << "transformation_matrix : " << endl;
    cout << transformation_matrix << endl;
    cout << "******************************************************" << endl;
}

void lidar_calibration::compute_error()
{
    vector<Point3f> pts1, pts2;
    for (int i = 0; i < global_points.size(); ++i)
    {
        pts1.push_back(Point3f(global_points[i].x, global_points[i].y, global_points[i].z));
        pts2.push_back(Point3f(lidar_points[i].x, lidar_points[i].y, lidar_points[i].z));
    }

    float error = 0;
    for (int i = 0; i < pts1.size(); i++)
    {
        Eigen::Vector3f pts(pts2[i].x, pts2[i].y, pts2[i].z);
        pts = R_ * pts + t_;
        Point3f dp(pts1[i].x - pts[0], pts1[i].y - pts[1], pts1[i].z - pts[2]);
        error += sqrt(dp.x * dp.x + dp.y * dp.y + dp.z * dp.z);
    }

    error /= pts1.size();
    cout << "compute error : "<< error << endl;
    cout << "******************************************************" << endl;
}

void lidar_calibration::match_viewer()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < lidar_points.size(); ++i)
    {
        cloud_in->push_back(lidar_points[i]);
    }

    for (int i = 0; i < global_points.size(); ++i)
    {
        cloud_out->push_back(global_points[i]);
    }

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("result"));
    viewer->addCoordinateSystem(); // R :X, G:Y, B:Z

    viewer->addPointCloud<pcl::PointXYZ>(cloud_in, "origin");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "origin"); // white
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "origin");

    pcl::transformPointCloud(*cloud_in, *cloud_in, transformation_matrix);

    cout << "Transform from Lidar Coordinate to Global Coordinate:" << endl;
    for (int i = 0; i < cloud_in->size(); ++i)
    {
        cout << "x = " << (*cloud_in)[i].x << ","
             << "y = " << (*cloud_in)[i].y << ","
             << "z = " << (*cloud_in)[i].z << endl;
    }
    cout << "******************************************************" << endl;

    viewer->addPointCloud<pcl::PointXYZ>(cloud_in, "transformed");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "transformed"); // red
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed");

    viewer->addPointCloud<pcl::PointXYZ>(cloud_out, "target");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "target"); // green
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

void lidar_calibration::low_callback(const sensor_msgs::PointCloud2ConstPtr &input)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*input, *cloud);
    pcl::transformPointCloud(*cloud, *cloud, transformation_matrix_low_to_global);

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = "rslidar";
    pub_low.publish(msg);
    ROS_INFO("Transform Low Lidar Successfully");

    // ROS_INFO("low lidar stamp value is: %f", input->header.stamp.toSec());
}

void lidar_calibration::high_callback(const sensor_msgs::PointCloud2ConstPtr &input)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*input, *cloud);
    pcl::transformPointCloud(*cloud, *cloud, transformation_matrix_high_to_global);

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = "rslidar";
    pub_high.publish(msg);
    ROS_INFO("Transform High Lidar Successfully");

    // ROS_INFO("high lidar stamp value is: %f", input->header.stamp.toSec());
}

void lidar_calibration::RotationMatrix2RPY()
{
    Isometry3f T;
    T.matrix() = transformation_matrix_low_to_global;
    Vector3f euler = T.rotation().eulerAngles(2, 1, 0);

    AngleAxisf rollAngle(AngleAxisf(euler(2), Vector3f::UnitX()));
    AngleAxisf pitchAngle(AngleAxisf(euler(1), Vector3f::UnitY()));
    AngleAxisf yawAngle(AngleAxisf(euler(0), Vector3f::UnitZ()));
    Matrix3f rotation_matrix;
    rotation_matrix = yawAngle * pitchAngle * rollAngle;

    cout << "Rotation Matrix: " << endl;
    cout << rotation_matrix << endl;
    cout << "******************************************************" << endl;
    cout << "Low to Global Euler Angle = [R : " << euler[2]
         << " P: " << euler[1] << " Y : " << euler[0] << "]" << endl;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lidar_calibration");
    ros::NodeHandle nh;

    lidar_calibration L(nh);
    ros::spin();

    // 求齐次变换矩阵时用
    // L.read_lidar_txt();
    // L.read_neh_txt();
    // L.pose_estimation_3d3d();
    // L.compute_error();
    // L.match_viewer();

    // 求欧拉角用
    // L.RotationMatrix2RPY();

    return 0;
}