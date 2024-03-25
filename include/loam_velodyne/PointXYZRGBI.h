#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// Define the custom point type
struct PointXYZRGBI {
    PCL_ADD_POINT4D; // This adds the members x,y,z which are the PointXYZ definition
    PCL_ADD_RGB;     // This adds the member rgb
    // float intensity;
    PCL_ADD_INTENSITY;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// Macro to register the custom point type with PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBI,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (std::uint32_t, rgb, rgb)
                                  (float, intensity, intensity))