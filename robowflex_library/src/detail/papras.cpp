/* Author: Kazuki Shin */

#include <robowflex_library/detail/papras.h>
#include <robowflex_library/io.h>
#include <robowflex_library/log.h>

using namespace robowflex;

const std::string  //
    PAPRASRobot::DEFAULT_URDF{"package://papras_simple_demo/urdf/simple_robot.urdf.xacro"};
const std::string  //
    PAPRASRobot::DEFAULT_SRDF{"package://papras_simple_moveit_config/config/simple_robot.srdf"};
const std::string  //
    PAPRASRobot::DEFAULT_LIMITS{"package://papras_simple_moveit_config/config/joint_limits.yaml"};
const std::string  //
    PAPRASRobot::DEFAULT_KINEMATICS{"package://papras_simple_moveit_config/config/kinematics.yaml"};
const std::string  //
    OMPL::PAPRAS_OMPLPipelinePlanner::DEFAULT_CONFIG{
        "package://papras_simple_moveit_config/config/ompl_planning.yaml"  //
    };
static const std::string GROUP = "arm1";

PAPRASRobot::PAPRASRobot() : Robot("PAPRAS")
{
}

bool PAPRASRobot::initialize(bool addVirtual)
{
    if (addVirtual)
        setSRDFPostProcessAddPlanarJoint("base_joint");

    bool success = false;
   
    RBX_INFO("Initializing PAPRAS with `papras_description`");
    success = Robot::initialize(DEFAULT_URDF, DEFAULT_SRDF, DEFAULT_LIMITS, DEFAULT_KINEMATICS);

    loadKinematics(GROUP);

    return success;
}

void PAPRASRobot::setBasePose(double x, double y, double z, double theta)
{
    if (hasJoint("base_joint/x") && hasJoint("base_joint/y") && hasJoint("base_joint/z") && hasJoint("base_joint/theta"))
    {
        const std::map<std::string, double> pose = {
            {"base_joint/x", x}, 
            {"base_joint/y", y}, 
            {"base_joint/z", z},
            {"base_joint/theta", theta}};

        scratch_->setVariablePositions(pose);
        scratch_->update();
    }
    else
        RBX_WARN("base_joint does not exist, cannot move base! You need to set addVirtual to true");
}

OMPL::PAPRAS_OMPLPipelinePlanner::PAPRAS_OMPLPipelinePlanner(const RobotPtr &robot, const std::string &name)
  : OMPLPipelinePlanner(robot, name)
{
}

bool OMPL::PAPRAS_OMPLPipelinePlanner::initialize(const Settings &settings,
                                              const std::vector<std::string> &adapters)
{
    return OMPLPipelinePlanner::initialize(DEFAULT_CONFIG, settings, DEFAULT_PLUGIN, adapters);
}
