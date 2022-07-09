/* Author: Kazuki Shin */

#include <robowflex_library/builder.h>
#include <robowflex_library/detail/papras.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>

using namespace robowflex;

/* \file papras_test.cpp
 * A simple script that demonstrates motion planning with the PAPRAS robot. Here,
 * two planners are created: the default OMPL planner, and the default OMPL
 * planner but with simplified solutions disabled.
 */

static const std::string GROUP = "arm1";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default PAPRAS robot.
    auto papras = std::make_shared<PAPRASRobot>();
    papras->initialize();

    // Create an empty scene.
    auto scene = std::make_shared<Scene>(papras);

    // Create the default planner for the PAPRAS.
    auto default_planner = std::make_shared<OMPL::PAPRAS_OMPLPipelinePlanner>(papras, "default");
    default_planner->initialize();

    // Create the a planner for the PAPRAS, and disable simplification.
    auto simple_planner = std::make_shared<OMPL::PAPRAS_OMPLPipelinePlanner>(papras, "simple");

    OMPL::Settings settings;
    settings.simplify_solutions = false;

    simple_planner->initialize(settings);

    // Run a motion plan for each planner.
    for (const auto &planner : {default_planner, simple_planner})
    {
        // Create a motion planning request with a pose goal.
        MotionRequestBuilder request(planner, GROUP);
        request.setStartConfiguration({0, 0, 0, 0, 0, 0});

        RobotPose pose = RobotPose::Identity();
        pose.translate(Eigen::Vector3d{0.0, 0.0, 0.0});
        Eigen::Quaterniond orn{0, 0, 1, 0};

        request.setGoalRegion("robot1/end_effector_link", "robot1/link1",               // links
                              pose, Geometry::makeSphere(10),  // position
                              orn, {10, 10, 10}           // orientation
        );

        // Do motion planning!
        planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
        if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
            return 1;
    }

    return 0;
}
