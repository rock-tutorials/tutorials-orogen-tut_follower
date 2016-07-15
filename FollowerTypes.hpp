#ifndef TUT_FOLLOWER_TYPES_HPP
#define TUT_FOLLOWER_TYPES_HPP

#include <base/commands/Motion2D.hpp>
#include <base/Eigen.hpp>

namespace tut_follower {
    struct Debug {
        double d;
        double yaw_delta;
        bool forward_motion_allowed;

        base::Vector3d target_position;
        base::Vector3d current_position;
        base::Vector3d delta_position;
        base::Vector3d current_forward;
        base::commands::Motion2D result;
    };
}

#endif

