#pragma once

#include <Eigen/Geometry>

namespace utils
{
    Eigen::Quaternionf enu_to_ned_via_quat_sandwich(const Eigen::Quaternionf &q_enu);

}
