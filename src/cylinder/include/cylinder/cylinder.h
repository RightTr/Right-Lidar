#pragma once

#include <iostream>                            // 引入输入输出流库
#include <pcl/ModelCoefficients.h>             // 引入模型系数类的头文件
#include <pcl/io/pcd_io.h>                     // 引入PCD文件输入输出的头文件
#include <pcl/point_types.h>                   // 引入点云类型定义的头文件
#include <pcl/sample_consensus/method_types.h> // 引入模型定义的头文件
#include <pcl/sample_consensus/model_types.h>  // 引入随机参数估计方法的头文件
#include <pcl/segmentation/sac_segmentation.h> // 引入基于采样一致性分割的类的头文件
#include <pcl/features/normal_3d.h>            // 引入法线估计的头文件
#include <pcl/filters/passthrough.h>           // 引入直通滤波器的头文件
#include <pcl/filters/extract_indices.h>       // 引入点提取的头文件
#include <pcl/search/kdtree.h>


#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose2D.h>

#define PLANECOUNT 12 //分割平面的数量

using namespace std;

class PclCylinder
{
    private:
        ros::Subscriber sub_point_cloud;
        ros::Publisher pub_cylinder_pos;
        ros::Publisher pub_cylinder_pc;

        void subCallBack(const sensor_msgs::PointCloud2ConstPtr &in_cloud); //回调函数

        void Normals_Estimate(const pcl::PointCloud<pcl::PointXYZ>::Ptr in, const pcl::PointCloud<pcl::Normal>::Ptr out); //法线估计

        void Plane_Seg(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, const pcl::PointCloud<pcl::Normal>::Ptr in_normals, 
                             pcl::ModelCoefficients::Ptr coefficients_plane, pcl::PointIndices::Ptr inliers_plane); //平面分割

        void Cylinder_Seg(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, const pcl::PointCloud<pcl::Normal>::Ptr in_normals,
                                pcl::ModelCoefficients::Ptr coefficients_cylinder, pcl::PointIndices::Ptr inliers_cylinder); //圆柱分割

        void Plane_Get(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, pcl::PointIndices::Ptr inliers_plane,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr out); //获取平面

        void Cylinder_Get(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, pcl::PointIndices::Ptr inliers_cylinder,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr out); //获取圆柱

        void Cylinder_Remove(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, const pcl::PointIndices::Ptr inliers_cylinder,
                          pcl::PointCloud<pcl::Normal>::Ptr in_normals); //去除圆柱及其法线

        void Plane_Remove(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, const pcl::PointIndices::Ptr inliers_plane,
                          pcl::PointCloud<pcl::Normal>::Ptr in_normals); //去除平面及其法线

        void Plane_Filter(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud); //去除输入点云的所有平面



    public:
        PclCylinder(ros::NodeHandle& nh);
        ~PclCylinder();
};