/* Author: Zachary Kingston */
/* Modified by: Juan D. Hernandez */

#include <robowflex_library/benchmarking.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/detail/papras.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>

#include <robowflex_library/io/gnuplot.h>

using namespace robowflex;

/* \file papras_benchmark.cpp
 * A basic script that demonstrates benchmarking with the Fetch robot.
 * Benchmarking output is saved in the OMPL format. See
 * https://ompl.kavrakilab.org/benchmark.html for more information on the
 * benchmark data format and how to use. http://plannerarena.org/ can be used to
 * visualize results.
 * Note: This script requires GNUPlot for live visualization of timing data.
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
    auto planner = std::make_shared<OMPL::PAPRAS_OMPLPipelinePlanner>(papras, "default");
    planner->initialize();

    // Setup a benchmarking request for the joint and pose motion plan requests.
    Profiler::Options options;
    options.metrics = Profiler::WAYPOINTS | Profiler::CORRECT | Profiler::LENGTH | Profiler::SMOOTHNESS;
    Experiment experiment("unfurl",  // Name of experiment
                          options,   // Options for internal profiler
                          5.0,       // Timeout allowed for ALL queries
                          100);      // Number of trials

    // Create a motion planning request with a pose goal.
    auto request = std::make_shared<MotionRequestBuilder>(planner, GROUP);
    papras->setGroupState(GROUP, {0, 0, 0, 0, 0, 0});  // Stow
    request->setStartConfiguration(papras->getScratchState());

    papras->setGroupState(GROUP, {0.0677, -0.8235, 0.9860, -0.1624, 0.0678, 0.0});  // Unfurl
    request->setGoalConfiguration(papras->getScratchState());

    request->setConfig("RRTConnect");
    experiment.addQuery("rrtconnect", scene, planner, request->getRequest());

    request->setConfig("RRT");
    experiment.addQuery("rrt", scene, planner, request->getRequest());

    request->setConfig("PRM");
    experiment.addQuery("prm", scene, planner, request->getRequest());

    request->setConfig("KPIECE");
    experiment.addQuery("kpiece", scene, planner, request->getRequest());

    request->setConfig("BKPIECE");
    experiment.addQuery("bkpiece", scene, planner, request->getRequest());

    request->setConfig("LBKPIECE");
    experiment.addQuery("lbkpiece", scene, planner, request->getRequest());

    request->setConfig("EST");
    experiment.addQuery("est", scene, planner, request->getRequest());

    // Use the post-query callback to visualize the data live.
    IO::GNUPlotPlanDataSetOutputter plot("time");
    experiment.setPostQueryCallback(
        [&](PlanDataSetPtr dataset, const PlanningQuery &) { plot.dump(*dataset); });

    auto dataset = experiment.benchmark(4);

    OMPLPlanDataSetOutputter output("robowflex_papras_demo");
    output.dump(*dataset);

    return 0;
}
