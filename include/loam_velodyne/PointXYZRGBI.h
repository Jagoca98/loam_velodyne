#ifndef POINTXYZRGBI_H
#define POINTXYZRGBI_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Define the custom point type
struct EIGEN_ALIGN16 _PointXYZRGBI
{
    PCL_ADD_POINT4D; // This adds the members x,y,z which are the PointXYZ definition
    PCL_ADD_RGB;     // This adds the member rgb
    PCL_ADD_INTENSITY;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
};

PCL_EXPORTS std::ostream &operator<<(std::ostream &os, const _PointXYZRGBI &p);
struct EIGEN_ALIGN16 PointXYZRGBI : public _PointXYZRGBI
{
    inline constexpr PointXYZRGBI(const _PointXYZRGBI &p) : PointXYZRGBI{p.x, p.y, p.z, p.r, p.g, p.b, p.intensity, p.a} {}

    inline constexpr PointXYZRGBI(std::uint32_t _intensity = 0) : PointXYZRGBI(0.f, 0.f, 0.f, 0, 0, 0, _intensity) {}

    inline constexpr PointXYZRGBI(std::uint8_t _r, std::uint8_t _g, std::uint8_t _b) : PointXYZRGBI(0.f, 0.f, 0.f, _r, _g, _b) {}

    inline constexpr PointXYZRGBI(float _x, float _y, float _z) : PointXYZRGBI(_x, _y, _z, 0, 0, 0) {}

    inline constexpr PointXYZRGBI(float _x, float _y, float _z,
                                  std::uint8_t _r, std::uint8_t _g, std::uint8_t _b,
                                  float _intensity = 0, std::uint8_t _a = 255) : _PointXYZRGBI{{{_x, _y, _z, 1.0f}}, {{{_b, _g, _r, _a}}}, _intensity} {}

    friend std::ostream &operator<<(std::ostream &os, const PointXYZRGBI &p);
    PCL_MAKE_ALIGNED_OPERATOR_NEW
};

 POINT_CLOUD_REGISTER_POINT_STRUCT (_PointXYZRGBI,
     (float, x, x)
     (float, y, y)
     (float, z, z)
     (std::uint32_t, rgba, rgba)
     (float, intensity, intensity)
 )
 POINT_CLOUD_REGISTER_POINT_WRAPPER(PointXYZRGBI, _PointXYZRGBI)

#endif // POINTXYZRGBI_H