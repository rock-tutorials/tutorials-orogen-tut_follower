/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace tut_follower;

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state)
{
    setDefaultValues();
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : TaskBase(name, engine, initial_state)
{
    setDefaultValues();
}

Task::~Task()
{
}


void Task::setDefaultValues()
{
    motor_controller::PIDSettings settings;
    settings.K = 0.05;
    settings.YMax = 0.5;
    _pid_distance.set(settings);
    settings.K = 1;
    settings.YMax = 1;
    settings.YMin = -1;
    _pid_heading.set(settings);
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
    rock_tutorial::BearingDistanceSensor sensor;
    if (_bearing_distance.readNewest(sensor) == RTT::NoData)
        return;

    if (last_update.isNull())
        last_update = base::Time::now();
    base::Time now = base::Time::now();

    // The resulting command
    base::MotionCommand2D cmd;
    // The component's internal structure for debugging purposes
    Debug debug;

    // This is the difference between the current orientation and the direction
    // towards the target
    //
    // We don't compute it when the target is too close, as the computation
    // would be ill defined
    cmd.rotation = pid_heading.update(0, sensor.bearing.getRad(), (now - last_update).toSeconds());

    // Don't move if the difference in heading is too big, or we might do weird
    // things
    if (fabs(sensor.bearing.getRad()) < M_PI / 4)
    {
        cmd.translation = pid_distance.update(_desired_distance, sensor.distance, (now - last_update).toSeconds());
        debug.forward_motion_allowed = true;
    }
    else
    {
        cmd.translation = 0;
        debug.forward_motion_allowed = false;
    }

    // Write the 
    _cmd.write(cmd);

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

