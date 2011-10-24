#ifndef TUT_FOLLOWER_TYPES_HPP
#define TUT_FOLLOWER_TYPES_HPP

#include <base/motion_command.h>
#include <base/eigen.h>

namespace tut_follower {
    struct Debug {
        double d;
        double yaw_delta;
        bool forward_motion_allowed;

        base::Vector3d target_position;
        base::Vector3d current_position;
        base::Vector3d delta_position;
        base::Vector3d current_forward;
        base::MotionCommand2D result;
    };
}

#endif

