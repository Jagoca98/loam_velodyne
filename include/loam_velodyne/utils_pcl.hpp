#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "PointXYZRGBI.h"

namespace utils
{
    // Convertion Function
    inline pcl::PointCloud<pcl::PointXYZRGBL>::Ptr convertToPointXYZRGBL(const pcl::PointCloud<PointXYZRGBI>::Ptr &custom_pointcloud)
    {

        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZRGBL>);

        for (auto &pt : *custom_pointcloud)
        {
            pcl::PointXYZRGBL pcl_point;
            pcl_point.x = pt.x;
            pcl_point.y = pt.y;
            pcl_point.z = pt.z;
            pcl_point.r = pt.r; // Extract red component
            pcl_point.g = pt.g; // Extract green component
            pcl_point.b = pt.b; // Extract blue component
            pcl_point.label = pt.intensity;
            pcl_pointcloud->push_back(pcl_point);
        }

        return pcl_pointcloud;
    }

}