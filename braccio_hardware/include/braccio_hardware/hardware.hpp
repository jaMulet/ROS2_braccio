#ifndef HARDWARE_HPP
#define HARDWARE_HPP

#include <string>
#include <cmath>


class Joint
{
    public:

    std::string name = "";
    float cmd = 0.0;
    int pos = 0;
    float vel = 0.0;

    Joint() = default;

    Joint(const std::string &joint_name)
    {
        setup(joint_name);
    }

    void setup(const std::string &joint_name)
    {
        name = joint_name;
    }
};

class GripperJoint
{
    public:

    std::string name = "";

    float cmd = 0.0;
    int pos = 0;
    float vel = 0.0;

    GripperJoint() = default;

    GripperJoint(const std::string &gripper_joint_name)
    {
        setup(gripper_joint_name);
    }

    void setup(const std::string &gripper_joint_name)
    {
        name = gripper_joint_name;
    }
};

#endif
