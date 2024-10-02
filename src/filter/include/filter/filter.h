#pragma once

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/filters/voxel_grid.h> //体素滤波
#include <pcl/filters/passthrough.h> //直通滤波

#include <sensor_msgs/PointCloud2.h>

#define MIN_DISTANCE 0.1 //截掉雷达自身周围2.4米内的点
#define MAX_DISTANCE 5 //截掉雷达4米外的点
#define RADIAL_DIVIDER_ANGLE 0.7 //雷达的水平光束发散间隔
#define SENSOR_HEIGHT 0.46 //雷达高度

#define concentric_divider_distance_ 0.01 //分割出同心圆半径
#define min_height_threshold_ 0.05 //最小高度阈值
#define local_max_slope_ 8   //同条射线上邻近两点的坡度阈值
#define general_max_slope_ 5 //整个地面的坡度阈值
#define reclass_distance_threshold_ 0.2 //与前一个点距离的阈值

#define LEAFSIZE 0.05f
#define PASSLOW 0.5
#define PASSHIGH 1.5


class PclFilter
{

  private:
    ros::Subscriber sub_point_cloud;
    ros::Publisher pub_vg, pub_no_ground, pub_ground;

    struct PointXYZRTColor
  {
    pcl::PointXYZ point;
    float radius; //表示点在XY平面上的圆柱坐标系中的径向距离（从原点到点的距离）。
    float theta;  //表示点在XY平面上的角度（以度为单位），用于描述点在圆柱坐标系中的角度位置。
    size_t radial_div;     //表示该点所属的径向分区的索引。径向分区是将点云按径向距离分成的若干部分。
    size_t concentric_div; //表示该点所属的同心圆分区的索引。同心圆分区是将点云按同心圆环分成的若干部分。
    size_t original_index; //表示该点在原始点云中的索引，用于跟踪点在源点云数据中的位置。
  };

  typedef std::vector<PointXYZRTColor> PointCloudXYZRTColor;

  size_t radial_dividers_num_; //径向分区的数量
  size_t concentric_dividers_num_; //同心圆分区的数量

  void Vg_pc(float leafsize, const pcl::PointCloud<pcl::PointXYZ>::Ptr in,
                          const pcl::PointCloud<pcl::PointXYZ>::Ptr out);

  void pass_pc(float low, float high, const pcl::PointCloud<pcl::PointXYZ>::Ptr in, 
                            const pcl::PointCloud<pcl::PointXYZ>::Ptr out);

  void clip_above(double clip_height, const pcl::PointCloud<pcl::PointXYZ>::Ptr in, const pcl::PointCloud<pcl::PointXYZ>::Ptr out); //用于裁剪点云中高于指定高度

  void remove_close_pc(double min_distance, const pcl::PointCloud<pcl::PointXYZ>::Ptr in, const pcl::PointCloud<pcl::PointXYZ>::Ptr out); //用于移除距离小于指定最小距离

  void remove_distant_pc(double max_distance, const pcl::PointCloud<pcl::PointXYZ>::Ptr in, const pcl::PointCloud<pcl::PointXYZ>::Ptr out); //用于移除距离大于指定最大距离
 
  void XYZ_to_RTZColor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,
                        PointCloudXYZRTColor &out_organized_points,
                        std::vector<pcl::PointIndices> &out_radial_divided_indices,
                        std::vector<PointCloudXYZRTColor> &out_radial_ordered_clouds); //将输入的点云从XYZI格式转换为RTZColor格式（即圆柱坐标系）

  void classify_pc(std::vector<PointCloudXYZRTColor> &in_radial_ordered_clouds,
                   pcl::PointIndices &out_ground_indices,
                   pcl::PointIndices &out_no_ground_indices); //将输入的有序点云分为地面点和非地面点

  void publish_cloud(const ros::Publisher &in_publisher,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr,
                     const std_msgs::Header &in_header); //用于发布点云数据

  void subCallBack(const sensor_msgs::PointCloud2ConstPtr &in_cloud);

  public:
    PclFilter(ros::NodeHandle &nh);
    ~PclFilter();
};
