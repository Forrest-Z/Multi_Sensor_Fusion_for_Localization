#include <lidar_camera_calibration/utils_pcl.h>

// 自定义格式转XYZ
pcl::PointCloud<pcl::PointXYZ> *toPointsXYZ(pcl::PointCloud<myPointXYZRID> point_cloud)
{
    pcl::PointCloud<pcl::PointXYZ> *new_cloud = new pcl::PointCloud<pcl::PointXYZ>();
    for (pcl::PointCloud<myPointXYZRID>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++)
    {
        new_cloud->push_back(pcl::PointXYZ(pt->x, pt->y, pt->z));
    }
    return new_cloud;
}

// 点云变换，传入平移+xyz欧拉角
pcl::PointCloud<myPointXYZRID> transform(pcl::PointCloud<myPointXYZRID> pc, float x, float y, float z, float rot_x, float rot_y, float rot_z)
{
    Eigen::Affine3f transf = pcl::getTransformation(x, y, z, rot_x, rot_y, rot_z);
    // std::cout << "初始变换矩阵(4x4):\n"
    //           << transf.matrix() << std::endl;
    pcl::PointCloud<myPointXYZRID> new_cloud;
    pcl::transformPointCloud(pc, new_cloud, transf);
    return new_cloud;
}

pcl::PointCloud<myPointXYZRID> normalizeIntensity(pcl::PointCloud<myPointXYZRID> point_cloud, float min, float max)
{
    float min_found = 10e6;
    float max_found = -10e6;

    for (pcl::PointCloud<myPointXYZRID>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++)
    {
        max_found = MAX(max_found, pt->intensity);
        min_found = MIN(min_found, pt->intensity);
    }

    for (pcl::PointCloud<myPointXYZRID>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++)
    {
        pt->intensity = (pt->intensity - min_found) / (max_found - min_found) * (max - min) + min;
    }
    return point_cloud;
}

pcl::PointCloud<myPointXYZRID> intensityByRangeDiff(pcl::PointCloud<myPointXYZRID> point_cloud, config_data config)
{
    // 恢复VLP16的按照线排列的点云
    std::vector<std::vector<myPointXYZRID *>> rings(16);
    // 计算点的距离
    for (auto pt = point_cloud.points.begin(); pt != point_cloud.points.end(); pt++)
    {
        pt->range = (pt->x * pt->x + pt->y * pt->y + pt->z * pt->z);
        rings[pt->ring].push_back(&(*pt));
    }
    // 按照ring遍历点云
    // TODO:这下面一堆对强度的操作在干啥
    for (auto ring = rings.begin(); ring != rings.end(); ring++)
    {
        myPointXYZRID *prev, *succ;
        if (ring->empty())
        {
            continue;
        }
        // 上一个点云强度
        float last_intensity = (*ring->begin())->intensity;
        // 当前点云强度
        float new_intensity;

        (*ring->begin())->intensity = 0;
        (*(ring->end() - 1))->intensity = 0;
        for (auto pt = ring->begin() + 1; pt < ring->end() - 1; pt++)
        {
            prev = *(pt - 1);
            succ = *(pt + 1);
            (*pt)->intensity = MAX(MAX(prev->range - (*pt)->range, succ->range - (*pt)->range), 0) * 10;
        }
    }
    // 强度正则化
    point_cloud = normalizeIntensity(point_cloud, 0.0, 1.0);

    pcl::PointCloud<myPointXYZRID> filtered;
    for (auto pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++)
    {
        // 保留点云强度>阈值
        if (pt->intensity > config.intensity_thresh)
        {
            // 保留感兴趣范围内的点云
            if (pt->x >= config.xyz_[0].first && pt->x <= config.xyz_[0].second && pt->y >= config.xyz_[1].first && pt->y <= config.xyz_[1].second && pt->z >= config.xyz_[2].first && pt->z <= config.xyz_[2].second)
            {
                filtered.push_back(*pt);
            }
        }
    }
    //pcl::io::savePCDFileASCII ("/home/lixiang/PCDs/filtered.pcd", *(toPointsXYZ(filtered)));
    return filtered;
}
