#include "optitrack4px4/utils.hpp"

namespace utils
{

    Eigen::Quaternionf enu_to_ned_via_quat_sandwich(const Eigen::Quaternionf &q_enu)
    {
        const float s = static_cast<float>(M_SQRT1_2); // sqrt(2)/2
        const Eigen::Quaternionf p(0.f, s, s, 0.f);    // (w, x, y, z)

        // Similarity transform / basis change: q' = p * q * p^{-1}
        // For unit quaternions, inverse == conjugate.
        Eigen::Quaternionf q_ned = p * q_enu * p.conjugate();

        // Optional: normalize to fight drift if you're doing many ops
        q_ned.normalize();
        return q_ned;
    }

}
