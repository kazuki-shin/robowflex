/* Author: Kazuki Shin */

// Robowflex
#include <robowflex_library/builder.h>
#include <robowflex_library/detail/papras.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/log.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>

using namespace robowflex;

/* \file papras_scenes_visualize.cpp
 * Visualizes a set of scenes and motion plans from requests in these scenes. A
 * number of example scene and planning request pairs are included in
 * 'package://robowflex_library/yaml/fetch_scenes'. See the associated
 * `fetch_scenes_benchmark.cpp` for benchmarking over these scenes. See
 * https://kavrakilab.github.io/robowflex/rviz.html for RViz visualization.
 */

static const std::string GROUP = "arm1";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default Fetch robot.
    auto papras = std::make_shared<PAPRASRobot>();

    papras->initialize();

    // Sets the Fetch's base pose.
    papras->setBasePose(0, 0, 2.0, 0.5);

    // Create an RViz visualization helper.
    // Publishes all topics and parameter under `/robowflex` by default.
    IO::RVIZHelper rviz(papras);

    RBX_INFO("RViz Initialized! Press enter to continue (after your RViz is setup)...");
    std::cin.get();


    const auto &scene_file =
        "package://robowflex_library/yaml/test_papras.yml";
    // const auto &request_file = "package://robowflex_library/yaml/fetch_scenes/request.yaml";

    // Create an empty Scene.
    auto scene = std::make_shared<Scene>(papras);
    if (not scene->fromYAMLFile(scene_file))
    {
        RBX_ERROR("Failed to read file: %s for scene", scene_file);
    }

    // Create the default planner for the Fetch.
    auto planner = std::make_shared<OMPL::PAPRAS_OMPLPipelinePlanner>(papras, "default");
    planner->initialize();

    RBX_INFO("Before request");

    // Create an empty motion planning request.
    auto request = std::make_shared<robowflex::MotionRequestBuilder>(planner, GROUP);
    // if (not request->fromYAMLFile(request_file))
    // {
    //     RBX_ERROR("Failed to read file: %s for request", request_file);
    // }

    // Visualize the scene.
    rviz.updateScene(scene);
    rviz.updateMarkers();

    RBX_INFO("Scene displayed! Press enter to plan...");
    std::cin.get();

    // Do motion planning!
    planning_interface::MotionPlanResponse res = planner->plan(scene, request->getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Publish the trajectory to a topic to display in RViz
    rviz.updateTrajectory(res);

    RBX_INFO("Press enter to remove the scene.");
    std::cin.get();

    rviz.removeScene();
    
}
