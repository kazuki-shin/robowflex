/* Author: Zachary Kingston */

#include <robowflex_library/builder.h>
#include <robowflex_library/detail/papras.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/log.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>

using namespace robowflex;

/* \file papras_visualization.cpp
 * A simple script that demonstrates how to use RViz with Robowflex with the
 * UR5 robot. See https://kavrakilab.github.io/robowflex/rviz.html for how to
 * use RViz visualization. Here, the scene, the pose goal, and motion plan
 * displayed in RViz.
 */

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default UR5 robot.
    auto papras = std::make_shared<PAPRASRobot>();
    papras->initialize();

    // Create an RViz visualization helper. Publishes all topics and parameter under `/robowflex` by default.
    IO::RVIZHelper rviz(papras);

    RBX_INFO("RViz Initialized! Press enter to continue (after your RViz is setup)...");
    std::cin.get();

    // Load a scene from a YAML file.
    auto scene = std::make_shared<Scene>(papras);
    // scene->fromYAMLFile("package://robowflex_library/yaml/test.yml");

    // Visualize the scene.
    rviz.updateScene(scene);

    // Create the default planner for the UR5.
    auto planner = std::make_shared<OMPL::PAPRAS_OMPLPipelinePlanner>(papras);
    planner->initialize();

    // Create a motion planning request with a pose goal.
    MotionRequestBuilder request(planner, "arm1");
    request.setStartConfiguration({0, 0, 0, 0, 0, 0});

    RobotPose pose = RobotPose::Identity();
    pose.translate(Eigen::Vector3d{0.4 , -0.3, 0.3});
    Eigen::Quaterniond orn{ 0.931, 0, 0,  -0.365}; // w x y z

    request.setGoalRegion("robot1/end_effector_link", "world",               // links
                                pose, Geometry::makeSphere(0.01),  // position
                                orn, {0.001, 0.001, 0.001}           // orientation
        );

    rviz.addGoalMarker("goal", request);
    rviz.updateMarkers();

    RBX_INFO("Scene and Goal displayed! Press enter to plan...");
    std::cin.get();

    // Do motion planning!
    planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Publish the trajectory to a topic to display in RViz
    rviz.updateTrajectory(res);

    RBX_INFO("Press enter to remove goal and scene.");
    std::cin.get();

    rviz.removeMarker("goal");
    rviz.updateMarkers();

    rviz.removeScene();

    RBX_INFO("Press enter to exit.");
    std::cin.get();

    return 0;
}
