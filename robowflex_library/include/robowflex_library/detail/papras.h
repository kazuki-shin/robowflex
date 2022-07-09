/* Author: Kazuki Shin */

#ifndef ROBOWFLEX_PAPRAS_
#define ROBOWFLEX_PAPRAS_

#include <robowflex_library/robot.h>
#include <robowflex_library/planning.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(PAPRASRobot);
    /* \endcond */

    /** \class robowflex::PAPRASRobotPtr
        \brief A shared pointer wrapper for robowflex::PAPRASRobot. */

    /** \class robowflex::PAPRASRobotConstPtr
        \brief A const shared pointer wrapper for robowflex::PAPRASRobot. */

    /** \brief Convenience class that describes the default setup for PAPRAS (with gripper and table)
     */
    class PAPRASRobot : public Robot
    {
    public:
        /** \brief Constructor.
         */
        PAPRASRobot();

        /** \brief Initialize the robot with manipulator kinematics.
         *  \return True on success, false on failure.
         */
        bool initialize();

    private:
        static const std::string DEFAULT_URDF;        ///< Default URDF
        static const std::string DEFAULT_SRDF;        ///< Default SRDF
        static const std::string DEFAULT_LIMITS;      ///< Default Limits
        static const std::string DEFAULT_KINEMATICS;  ///< Default kinematics
    };

    namespace OMPL
    {
        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(PAPRAS_OMPLPipelinePlanner);
        /* \endcond */

        /** \class robowflex::OMPL::PAPRAS_OMPLPipelinePlanner
            \brief A shared pointer wrapper for robowflex::OMPL::PAPRAS_OMPLPipelinePlanner. */

        /** \class robowflex::OMPL::PAPRAS_OMPLPipelinePlanner
            \brief A const shared pointer wrapper for robowflex::OMPL::PAPRAS_OMPLPipelinePlanner. */

        /** \brief Convenience class for the default motion planning pipeline for PAPRAS.
         */
        class PAPRAS_OMPLPipelinePlanner : public OMPLPipelinePlanner
        {
        public:
            /** \brief Constructor.
             *  \param[in] robot Robot to create planner for.
             *  \param[in] name Namespace of this planner.
             */
            PAPRAS_OMPLPipelinePlanner(const RobotPtr &robot, const std::string &name = "");

            /** \brief Initialize the planning context. All parameter provided are defaults.
             *  \param[in] settings Settings to set on the parameter server.
             *  \param[in] adapters Planning adapters to load.
             *  \return True on success, false on failure.
             */
            bool initialize(const Settings &settings = Settings(),
                            const std::vector<std::string> &adapters = DEFAULT_ADAPTERS);

        private:
            static const std::string DEFAULT_CONFIG;   ///< Default planning configuration.
        };
    }  // namespace OMPL
}  // namespace robowflex

#endif
