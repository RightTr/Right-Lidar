#include "filter.h"

PclFilter::PclFilter(ros::NodeHandle &nh){
    sub_point_cloud = nh.subscribe("/livox/lidar",10, &PclFilter::subCallBack, this);
    
    pub_vg = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 10);
    pub_ground = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points_ground", 10);
    pub_no_ground = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points_no_ground", 10);

    ros::spin();
}

PclFilter::~PclFilter(){}


void PclFilter::Vg_pc(float leafsize, const pcl::PointCloud<pcl::PointXYZ>::Ptr in,
                            const pcl::PointCloud<pcl::PointXYZ>::Ptr out)
{
    pcl::VoxelGrid<pcl::PointXYZ> vg;

    vg.setInputCloud(in);
    vg.setLeafSize(leafsize, leafsize, leafsize);
    vg.filter(*out);

}

void PclFilter::pass_pc(float low, float high, const pcl::PointCloud<pcl::PointXYZ>::Ptr in, 
                            const pcl::PointCloud<pcl::PointXYZ>::Ptr out)
{
    pcl::PassThrough<pcl::PointXYZ> pass;	//创建对象
    pass.setInputCloud(in);				//设置输入点云
    pass.setFilterFieldName("z");			//设置过滤字段，这里对z轴上的点云进行过滤
    pass.setFilterLimits(low, high);			//设置过滤范围
    //pass.setFilterLimitsNegative (true);	//设置选择保留范围内的还是过滤掉范围内的点云
    pass.filter(*out);			//执行滤波，结果保存在cloud_filter中
}
 

void PclFilter::remove_close_pc(double min_distance, const pcl::PointCloud<pcl::PointXYZ>::Ptr in,
                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr out)
{
    pcl::ExtractIndices<pcl::PointXYZ> cliper; //设置提取器

    cliper.setInputCloud(in);
    pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices);; //移除点索引
    //TODO:考虑并行加速2
    for (size_t i = 0; i < in->points.size(); i++)
    {
        double distance = sqrt(in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y);

        if (distance < min_distance)
        {
            indices_ptr->indices.push_back(i);
        }
    }
    cliper.setIndices(indices_ptr);//告诉提取器要去除点的索引
    cliper.setNegative(true); //移除相应索引的点
    cliper.filter(*out);
}

void PclFilter::remove_distant_pc(double max_distance, const pcl::PointCloud<pcl::PointXYZ>::Ptr in,
                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr out)
{
    pcl::ExtractIndices<pcl::PointXYZ> cliper; //设置提取器

    cliper.setInputCloud(in);
    pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices);; //移除点索引
    //TODO:考虑并行加速2
    for (size_t i = 0; i < in->points.size(); i++)
    {
        double distance = sqrt(in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y);

        if (distance > max_distance)
        {
            indices_ptr->indices.push_back(i);
        }
    }
    cliper.setIndices(indices_ptr);//告诉提取器要去除点的索引
    cliper.setNegative(true); //移除相应索引的点
    cliper.filter(*out);
}

/*!
 * @param[in] in_cloud 输入点云，将被组织成径向段
 * @param[out] out_organized_points 自定义点云，填充了 XYZRTZColor 数据
 * @param[out] out_radial_divided_indices 原始点云中每个径向段的点的索引
 * @param[out] out_radial_ordered_clouds 点云的向量，每个元素将包含已排序的点
 */

void PclFilter::XYZ_to_RTZColor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,
                                   PointCloudXYZRTColor &out_organized_points,
                                   std::vector<pcl::PointIndices> &out_radial_divided_indices,
                                   std::vector<PointCloudXYZRTColor> &out_radial_ordered_clouds)
{
    out_organized_points.resize(in_cloud->points.size()); //调整 out_organized_points 的大小为输入点云的点数
    out_radial_divided_indices.clear(); //清空 out_radial_divided_indices
    out_radial_divided_indices.resize(radial_dividers_num_); //调整 out_radial_divided_indices 的大小为径向分割数
    out_radial_ordered_clouds.resize(radial_dividers_num_); //调整 out_radial_ordered_clouds 的大小为径向分割数

    for (size_t i = 0; i < in_cloud->points.size(); i++)
    {
        PointXYZRTColor new_point;
        auto radius = (float)sqrt(
            in_cloud->points[i].x * in_cloud->points[i].x + in_cloud->points[i].y * in_cloud->points[i].y); //极径
        auto theta = (float)atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / M_PI; //极角
        if (theta < 0)
        {
            theta += 360;
        }
        //角度的微分
        auto radial_div = (size_t)floor(theta / RADIAL_DIVIDER_ANGLE); //计算出该点落在哪个角度分割内，结果是一个整数索引
        //半径的微分
        auto concentric_div = (size_t)floor(fabs(radius / concentric_divider_distance_)); //计算该点距离原点的半径在同心分割中的索引位置

        new_point.point = in_cloud->points[i];
        new_point.radius = radius;
        new_point.theta = theta;
        new_point.radial_div = radial_div;
        new_point.concentric_div = concentric_div;
        new_point.original_index = i;

        out_organized_points[i] = new_point;

        out_radial_divided_indices[radial_div].indices.push_back(i); //将当前点的索引 i 添加到其对应的径向分割的索引列表中

        out_radial_ordered_clouds[radial_div].push_back(new_point); //将当前点 new_point 添加到其对应的径向分割的点云中

    } //end for

    //将同一根射线上的点按照半径（距离）排序
    //TODO:考虑并行加速3
    for (size_t i = 0; i < radial_dividers_num_; i++)
    {
        std::sort(out_radial_ordered_clouds[i].begin(), out_radial_ordered_clouds[i].end(),
                  [](const PointXYZRTColor &a, const PointXYZRTColor &b) { return a.radius < b.radius; });
    }
}

/*!
 * @param in_radial_ordered_clouds 
 * 这个参数包含了所有点，根据它们与原点的距离进行了排序
 * @param out_ground_indices
 * 这个参数用于返回分类为“地面”的点在原始点云中的索引
 * @param out_no_ground_indices 
 *  这个参数用于返回分类为“非地面”的点在原始点云中的索引
 */
void PclFilter::classify_pc(std::vector<PointCloudXYZRTColor> &in_radial_ordered_clouds,
                              pcl::PointIndices &out_ground_indices,
                              pcl::PointIndices &out_no_ground_indices)
{
    out_ground_indices.indices.clear(); //清空
    out_no_ground_indices.indices.clear(); //清空
    //TODO:考虑并行加速
    for (size_t i = 0; i < in_radial_ordered_clouds.size(); i++) //遍历每一根射线
    {
        float prev_radius = 0.f; //上一个点的半径距离
        float prev_height = -SENSOR_HEIGHT; //上一个点的高度
        bool prev_ground = false; //上一个点是否是地面点
        bool current_ground = false; //当前点是否是地面点
        for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); j++) //遍历射线中的每个点
        {
            float points_distance = in_radial_ordered_clouds[i][j].radius - prev_radius; //当前点和上一个点之间的距离
            float height_threshold = tan(DEG2RAD(local_max_slope_)) * points_distance; //当前点与前一点的高度阈值，由局部最大坡度计算
            float current_height = in_radial_ordered_clouds[i][j].point.z; //当前点的高度
            float general_height_threshold = tan(DEG2RAD(general_max_slope_)) * in_radial_ordered_clouds[i][j].radius; //基于当前点半径距离的整体高度阈值，由整体最大坡度计算

            //对于距离较近的点，如果高度阈值太小，则设置为最小高度阈值。防止高度阈值过小导致错误分类
            if (points_distance > concentric_divider_distance_ && height_threshold < min_height_threshold_)
            {
                height_threshold = min_height_threshold_;
            }

            //检查当前点高度与局部阈值的关系
            //检查当前点的高度是否在前一个点高度的范围内，即在 prev_height ± height_threshold 之间
            if (current_height <= (prev_height + height_threshold) && current_height >= (prev_height - height_threshold))
            {
                //使用整体几何关系再检查一次，如果前一个点不是地面点
                if (!prev_ground) //这个点前面的一个点不是地面点，采用整体阈值的判断
                {
                    if (current_height <= (-SENSOR_HEIGHT + general_height_threshold) && current_height >= (-SENSOR_HEIGHT - general_height_threshold))
                    {
                        current_ground = true;
                    }
                    else
                    {
                        current_ground = false;
                    }
                }
                else //与前一个地面点的距离在阈值内，则这个点就是地面点
                {
                    current_ground = true;
                }
            }
            else
            {
                //检查前一个点是否离当前点太远，如果是，则重新分类
                if (points_distance > reclass_distance_threshold_ &&
                    (current_height <= (-SENSOR_HEIGHT + height_threshold) && current_height >= (-SENSOR_HEIGHT - height_threshold))) 
                {
                    current_ground = true;
                }
                else
                {
                    current_ground = false;
                }
            }

            if (current_ground)
            {
                out_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                prev_ground = true;
            }
            else
            {
                out_no_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                prev_ground = false;
            }

            prev_radius = in_radial_ordered_clouds[i][j].radius; //迭代
            prev_height = in_radial_ordered_clouds[i][j].point.z; //迭代
        }
    }
}

void PclFilter::publish_cloud(const ros::Publisher &in_publisher,
                                const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr,
                                const std_msgs::Header &in_header)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg); //PCL to pointcloud2
    cloud_msg.header = in_header; //头部消息（时间轴、框架ID）
    in_publisher.publish(cloud_msg);
}

void PclFilter::subCallBack(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>); //PCL数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr vg_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>); //体素滤波后的数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr pass_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>); //过滤远点的数据

    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr); //pointcloud2 to PCL

    Vg_pc(LEAFSIZE, current_pc_ptr, vg_pc_ptr); //体素滤波
    pass_pc(PASSLOW, PASSHIGH, vg_pc_ptr, pass_pc_ptr); //过滤z轴的点云

    pcl::PointCloud<pcl::PointXYZ>::Ptr remove_close(new pcl::PointCloud<pcl::PointXYZ>); //过滤近点的数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr remove_distant(new pcl::PointCloud<pcl::PointXYZ>); //过滤远点的数据
    
    remove_close_pc(MIN_DISTANCE, pass_pc_ptr, remove_close); //过滤近点
    remove_distant_pc(MAX_DISTANCE, remove_close, remove_distant); //过滤远点

    PointCloudXYZRTColor organized_points; //圆柱坐标系数据
    std::vector<pcl::PointIndices> radial_division_indices; // 存储每个径向分割区中点的索引
    std::vector<pcl::PointIndices> closest_indices; //存储距离原点最近的点的索引
    std::vector<PointCloudXYZRTColor> radial_ordered_clouds; //存储每个径向分割区内按照半径排序的点云数据

    radial_dividers_num_ = ceil(360 / RADIAL_DIVIDER_ANGLE); //计算径向分割的数量

    XYZ_to_RTZColor(remove_distant, organized_points,
                     radial_division_indices, radial_ordered_clouds); //直角坐标系 to 圆柱坐标系

    pcl::PointIndices ground_indices, no_ground_indices; //地面点云索引、非地面点云索引

    classify_pc(radial_ordered_clouds, ground_indices, no_ground_indices); //分割地面与非地面点云

    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>); //创建共享指针，用于存储提取的地面点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>); //创建共享指针，用于存储提取的非地面点云

    pcl::ExtractIndices<pcl::PointXYZ> extract_ground; //用于从输入点云中提取特定的点
    extract_ground.setInputCloud(remove_distant); //传入近点点云索引

    pcl::PointIndices::Ptr ground_indices_ptr(new pcl::PointIndices(ground_indices)); //创建共享指针，用于存储提取的地面点云
    extract_ground.setIndices(ground_indices_ptr); //传入地面点云索引

    extract_ground.setNegative(false); //保留地面点云
    extract_ground.filter(*ground_cloud_ptr); //得到地面点云

    extract_ground.setNegative(true); //去除地面点云，即保留非地面点云
    extract_ground.filter(*no_ground_cloud_ptr); //得到非地面点云

    ////pub for debug
    // sensor_msgs::PointCloud2 pub_pc;
    // pcl::toROSMsg(*no_ground_cloud_ptr, pub_pc);

    // pub_pc.header = in_cloud_ptr->header;

    // pub_ground_.publish(pub_pc);

    publish_cloud(pub_vg, vg_pc_ptr, in_cloud_ptr->header); //发布体素滤波点云
    publish_cloud(pub_ground, ground_cloud_ptr, in_cloud_ptr->header); //发布地面点云
    publish_cloud(pub_no_ground, no_ground_cloud_ptr, in_cloud_ptr->header); //发布非地面点云
}
