/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace tut_follower;

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : TaskBase(name, engine, initial_state)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

// bool Task::configureHook()
// {
//     if (! TaskBase::configureHook())
//         return false;
//     return true;
// }
//
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    pid_distance.setPIDSettings(_pid_distance);
    pid_distance.reset();
    pid_heading.setPIDSettings(_pid_heading);
    pid_heading.reset();

    last_update = base::Time();
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    base::samples::RigidBodyState target;
    if (_target_position.readNewest(target) == RTT::NoData)
        return;

    base::samples::RigidBodyState current;
    if (_current_position.readNewest(current) == RTT::NoData)
        return;

    if (last_update.isNull())
        last_update = base::Time::now();
    base::Time now = base::Time::now();

    Eigen::Vector3d v = (target.position - current.position);
    double d = v.norm();

    // The resulting command
    base::MotionCommand2D cmd;
    // The component's internal structure for debugging purposes
    Debug debug;

    // This is the difference between the current orientation and the direction
    // towards the target
    //
    // We don't compute it when the target is too close, as the computation
    // would be ill defined
    base::Angle yaw_delta;
    Eigen::Vector3d forward = Eigen::Vector3d::Zero();
    if (d > _desired_distance / 10)
    {
        forward = current.orientation * base::Vector3d::UnitX();
        yaw_delta = base::Angle::vectorToVector(forward, v, base::Vector3d::UnitZ());
    }
    cmd.rotation = pid_heading.update(yaw_delta.getRad(), 0, (now - last_update).toSeconds());

    // Don't move if the difference in heading is too big, or we might do weird
    // things
    if (fabs(yaw_delta.getRad()) < M_PI / 4)
    {
        cmd.translation = pid_distance.update(_desired_distance, d, (now - last_update).toSeconds());
        debug.forward_motion_allowed = true;
    }
    else
    {
        cmd.translation = 0;
        debug.forward_motion_allowed = false;
    }

    // Write the 
    _cmd.write(cmd);

    debug.current_forward = forward;
    debug.yaw_delta = yaw_delta.getRad();
    debug.d = d;
    debug.target_position = target.position;
    debug.current_position = current.position;
    debug.delta_position = v;
    debug.result = cmd;
    _debug.write(debug);

    last_update = now;

}
// void Task::errorHook()
// {
//     TaskBase::errorHook();
// }
// void Task::stopHook()
// {
//     TaskBase::stopHook();
// }
// void Task::cleanupHook()
// {
//     TaskBase::cleanupHook();
// }

