/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_point.h
 * @brief PCL point datatype for use with ouster sensors
 */

#pragma once

#include <pcl/point_types.h>

namespace velodyne_ros {

// The default/original representation of the point cloud since the driver
// inception. This shouldn't be confused with Point_LEGACY which provides exact
// mapping of the fields of Ouster LidarScan of the Legacy Profile, copying
// the same order and using the same bit representation. For example, this Point
// struct uses float data type to represent intensity (aka signal); however, the
// sensor sends the signal channel either as UINT16 or UINT32 depending on the
// active udp lidar profile.
struct EIGEN_ALIGN16 _Point {
    PCL_ADD_POINT4D;
    float intensity;        // equivalent to signal
    uint16_t ring;          // equivalent to channel
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct Point : public _Point {

    inline Point(const _Point& pt)
    {
      x = pt.x; y = pt.y; z = pt.z; data[3] = 1.0f;
      intensity = pt.intensity;
      ring = pt.ring;
      time = pt.time;
    }

    inline Point()
    {
      x = y = z = 0.0f; data[3] = 1.0f;
      intensity = 0.0f;
      ring = 0;
      time = 0.0f;
    }

    inline const auto as_tuple() const {
        return std::tie(x, y, z, intensity, ring, time);
    }

    inline auto as_tuple() {
        return std::tie(x, y, z, intensity, ring, time);
    }

    template<size_t I>
    inline auto& get() {
        return std::get<I>(as_tuple());
    }
};

}   // namespace ouster_ros

// clang-format off

// DEFAULT/ORIGINAL
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (float, time, time)
)

// clang-format on
