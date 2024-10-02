#include "cylinder.h"


PclCylinder::PclCylinder(ros::NodeHandle &nh)
{
    sub_point_cloud = nh.subscribe("/filtered_points_no_ground", 10, &PclCylinder::subCallBack, this);
    
    pub_cylinder_pc = nh.advertise<sensor_msgs::PointCloud2>("cylinder_pc_node", 10);
    pub_cylinder_pos = nh.advertise<geometry_msgs::Pose2D>("cylinder_pos_node", 10);

    ros::spin();
}

PclCylinder::~PclCylinder(){}

void PclCylinder::Normals_Estimate(const pcl::PointCloud<pcl::PointXYZ>::Ptr in, const pcl::PointCloud<pcl::Normal>::Ptr out) //法线估计
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>); //kdtree
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne; //法线估计对象
    ne.setSearchMethod(tree); //设置搜索方法

    ne.setInputCloud(in); //设置输入点云

    ne.setKSearch(15); //设置领域搜索点数，这个参数很关键，太大了就识别不出来了

    ne.compute(*out); //计算法线

}

void PclCylinder::Cylinder_Seg(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, const pcl::PointCloud<pcl::Normal>::Ptr in_normals,   
                                     pcl::ModelCoefficients::Ptr coefficients_cylinder, pcl::PointIndices::Ptr inliers_cylinder)  //圆柱分割
{

    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg; //创建分割对象 

	seg.setOptimizeCoefficients(true);         //优化模型参数
	seg.setModelType(pcl::SACMODEL_CYLINDER);  //设置分割模型为圆柱
	seg.setMethodType(pcl::SAC_RANSAC);        //使用RANSAC方法  
	// seg.setNormalDistanceWeight(0.1);          //设置法线权重
	seg.setMaxIterations(10000);               //最大迭代次数
	seg.setDistanceThreshold(0.05);             //距离阈值
	seg.setRadiusLimits(0.1, 0.3);               //设置圆柱半径范围
	seg.setInputCloud(in_cloud);               //设置输入点云
    seg.setInputNormals(in_normals);           //设置输入法线

    seg.segment(*inliers_cylinder, *coefficients_cylinder); //开始分割，获取圆柱内点和模型系数

    if(inliers_cylinder->indices.empty())
    {
        cerr << "Fault" << endl;
    }
} 

void PclCylinder::Plane_Seg(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, const pcl::PointCloud<pcl::Normal>::Ptr in_normals, 
                    pcl::ModelCoefficients::Ptr coefficients_plane, pcl::PointIndices::Ptr inliers_plane) //平面分割
{
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg; //创建分割对象 

    seg.setOptimizeCoefficients(true);      //优化模型参数
	seg.setModelType(pcl::SACMODEL_PLANE);  //设置分割模型为平面
    seg.setNormalDistanceWeight(0.1);       //设置法线权重
    seg.setMaxIterations(10000);            //最大迭代次数
	seg.setMethodType(pcl::SAC_RANSAC);     //使用RANSAC方法 
	seg.setDistanceThreshold(0.01);         //距离阈值
	seg.setInputCloud(in_cloud);            //设置输入点云
	seg.setInputNormals(in_normals);         //设置输入法线

	seg.segment(*inliers_plane, *coefficients_plane); //开始分割，获取平面内点和模型系数
}

void PclCylinder::Cylinder_Get(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, pcl::PointIndices::Ptr inliers_cylinder, 
                                pcl::PointCloud<pcl::PointXYZ>::Ptr out) //获取圆柱
{
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    extract.setInputCloud(in_cloud); //设置输入点云
    extract.setIndices(inliers_cylinder); //设置要提取的圆柱内点索引
    extract.setNegative(false); //提取索引点

    extract.filter(*out); //输出点云

	if (out->points.empty())
    {
		cerr << "Can't find the cylindrical component." << endl;
        return ;
    }
}

void PclCylinder::Plane_Get(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, pcl::PointIndices::Ptr inliers_plane,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr out) //获取平面
{
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    extract.setInputCloud(in_cloud); //设置输入点云
    extract.setIndices(inliers_plane); //设置要提取的平面内点索引
    extract.setNegative(false); //提取索引点

    extract.filter(*out); //输出点云

}

void PclCylinder::Cylinder_Remove(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, const pcl::PointIndices::Ptr inliers_cylinder,
                               pcl::PointCloud<pcl::Normal>::Ptr in_normals) //去除圆柱及其法线
{
    pcl::ExtractIndices<pcl::PointXYZ> extract; //圆柱提取
    pcl::ExtractIndices<pcl::Normal> extract_normals; //法线提取

    extract.setInputCloud(in_cloud); //设置输入点云
    extract.setIndices(inliers_cylinder); //设置要去除的圆柱内点索引
    extract.setNegative(true); //去除索引点，得到去除圆柱的点云数据
    extract.filter(*in_cloud); //提取非圆柱部分

    extract_normals.setInputCloud(in_normals); //设置输入法线
    extract_normals.setIndices(inliers_cylinder); //设置要去除的圆柱内点法线索引
    extract_normals.setNegative(true); //去除索引点，得到除去该圆柱的法线信息
    extract_normals.filter(*in_normals); //提取非圆柱部分的法线信息
}

void PclCylinder::Plane_Remove(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, const pcl::PointIndices::Ptr inliers_plane,
                               pcl::PointCloud<pcl::Normal>::Ptr in_normals)
{
    pcl::ExtractIndices<pcl::PointXYZ> extract; //平面提取
    pcl::ExtractIndices<pcl::Normal> extract_normals; //法线提取

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>()); //储存结果
    // pcl::PointCloud<pcl::Normal>::Ptr normals_filtered(new pcl::PointCloud<pcl::Normal>());  //储存结果

    extract.setInputCloud(in_cloud); //设置输入点云
    extract.setIndices(inliers_plane); //设置要去除的平面内点索引
    extract.setNegative(true); //去除索引点，得到去除平面的点云数据
    extract.filter(*in_cloud); //提取非平面部分

    extract_normals.setInputCloud(in_normals); //设置输入法线
    extract_normals.setIndices(inliers_plane); //设置要去除的平面内点法线索引
    extract_normals.setNegative(true); //去除索引点，得到除去该平面的法线信息
    extract_normals.filter(*in_normals); //提取非平面部分

    // in_cloud->swap(*cloud_filtered);
    // in_normals->swap(*normals_filtered);
}

void PclCylinder::Plane_Filter(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud) //去除输入点云的所有平面
{
    pcl::PointCloud<pcl::Normal>::Ptr normal_ptr(new pcl::PointCloud<pcl::Normal>); //法线数据
    pcl::ModelCoefficients::Ptr coefficients_plane_ptr(new pcl::ModelCoefficients); //平面参数向量
    pcl::PointIndices::Ptr inliers_plane_ptr(new pcl::PointIndices); //平面内点向量

    Normals_Estimate(in_cloud, normal_ptr); //法线估计

    int plane_count = 0; //平面数量

    while(plane_count < PLANECOUNT) //没找到PLANTCOUNT个平面就执行循环
    { 
        Plane_Seg(in_cloud, normal_ptr, coefficients_plane_ptr, inliers_plane_ptr); //分割平面
        plane_count++;

        if(inliers_plane_ptr->indices.empty()) //没有平面了就退出循环
        {
            break;
        }

        Plane_Remove(in_cloud, inliers_plane_ptr, normal_ptr); //将这个平面移除
    }
    cerr << "Find " << plane_count << " planes." << endl;
}


void PclCylinder::subCallBack(const sensor_msgs::PointCloud2ConstPtr &in_cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>); //pcl XYZ数据
    pcl::PointCloud<pcl::Normal>::Ptr normal_ptr(new pcl::PointCloud<pcl::Normal>); //法线数据

    pcl::ModelCoefficients::Ptr coefficients_cylinder_ptr1(new pcl::ModelCoefficients); //圆柱1参数
    pcl::ModelCoefficients::Ptr coefficients_cylinder_ptr2(new pcl::ModelCoefficients); //圆柱2参数

    pcl::PointIndices::Ptr inliers_cylinder_ptr1(new pcl::PointIndices); //圆柱1内点
    pcl::PointIndices::Ptr inliers_cylinder_ptr2(new pcl::PointIndices); //圆柱2内点

    pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_pc_ptr1(new pcl::PointCloud<pcl::PointXYZ>); //圆柱1点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_pc_ptr2(new pcl::PointCloud<pcl::PointXYZ>); //圆柱2点云


    pcl::fromROSMsg(*in_cloud, *current_pc_ptr); //PointCloud2 to pcl

    // Plane_Filter(current_pc_ptr); //去除平面

    Normals_Estimate(current_pc_ptr, normal_ptr); //法线估计
    int cylinder_count = 0;

    //for debug
    // Cylinder_Seg(current_pc_ptr, normal_ptr, coefficients_cylinder_ptr1, inliers_cylinder_ptr1); //圆柱1分割
    // Cylinder_Get(current_pc_ptr, inliers_cylinder_ptr1, cylinder_pc_ptr1); //输出圆柱1 

    while(cylinder_count < 2)
    {
        if(cylinder_count == 0)
        {
            Cylinder_Seg(current_pc_ptr, normal_ptr, coefficients_cylinder_ptr1, inliers_cylinder_ptr1); //圆柱1分割
            Cylinder_Get(current_pc_ptr, inliers_cylinder_ptr1, cylinder_pc_ptr1); //输出圆柱1

            if (inliers_cylinder_ptr1->indices.empty())
            {
                continue; // 如果没有找到圆柱，重新开始循环
            }
            else
            {
                cerr << "Cylinder1 coefficients:"  
                     << "x1:" << coefficients_cylinder_ptr1->values[3] << ","
                     << "y1:" << coefficients_cylinder_ptr1->values[4] << ","
                     << "r1:" << coefficients_cylinder_ptr1->values[5] << endl;

                cylinder_count++;
                // Cylinder_Remove(current_pc_ptr, inliers_cylinder_ptr1, normal_ptr); //去除一个圆柱
            }
        }
        // else if(cylinder_count == 1)
        // {
        //     Cylinder_Seg(current_pc_ptr, normal_ptr, coefficients_cylinder_ptr2, inliers_cylinder_ptr2); //圆柱2分割
        //     Cylinder_Get(current_pc_ptr, inliers_cylinder_ptr2, cylinder_pc_ptr2); //输出圆柱2

        //     if (inliers_cylinder_ptr2->indices.empty())
        //         {
        //             continue; // 如果没有找到圆柱，重新开始循环
        //         }
        //         else
        //         {
        //             cerr << "Cylinder2 coefficients:"  
        //              << "x2:" << coefficients_cylinder_ptr2->values[3] << ","
        //              << "y2:" << coefficients_cylinder_ptr2->values[4] << ","
        //              << "r2:" << coefficients_cylinder_ptr2->values[5] << endl;

        //             cylinder_count++;
        //         }
        // }
        cylinder_count = 0;
        
    }      

    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>); 

    *merged_pc_ptr = *cylinder_pc_ptr1 + *cylinder_pc_ptr2; //合并两个圆柱点云

    sensor_msgs::PointCloud2 pcl2_msg;
    pcl::toROSMsg(*cylinder_pc_ptr1, pcl2_msg); //pcl to pointcloud2
    pcl2_msg.header = in_cloud->header; //头部消息（时间轴、框架ID）
    pub_cylinder_pc.publish(pcl2_msg);
}