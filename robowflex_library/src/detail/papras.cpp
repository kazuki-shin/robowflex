/* Author: Kazuki Shin */

#include <robowflex_library/detail/papras.h>
#include <robowflex_library/io.h>
#include <robowflex_library/log.h>

using namespace robowflex;

const std::string  //
    PAPRASRobot::DEFAULT_URDF{"package://papras_description/urdf/big_table.urdf.xacro"};
const std::string  //
    PAPRASRobot::DEFAULT_SRDF{"package://big_table_moveit_config/config/big_table.srdf"};
const std::string  //
    PAPRASRobot::DEFAULT_LIMITS{"package://big_table_moveit_config/config/joint_limits.yaml"};
const std::string  //
    PAPRASRobot::DEFAULT_KINEMATICS{"package://big_table_moveit_config/config/kinematics.yaml"};
const std::string  //
    OMPL::PAPRAS_OMPLPipelinePlanner::DEFAULT_CONFIG{
        "package://big_table_moveit_config/config/ompl_planning.yaml"  //
    };
static const std::string GROUP = "arm2";

PAPRASRobot::PAPRASRobot() : Robot("PAPRAS")
{
}

bool PAPRASRobot::initialize()
{
    bool success = false;
   
    RBX_INFO("Initializing PAPRAS with `papras_description`");
    success = Robot::initialize(DEFAULT_URDF, DEFAULT_SRDF, DEFAULT_LIMITS, DEFAULT_KINEMATICS);

    loadKinematics(GROUP);

    return success;
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
