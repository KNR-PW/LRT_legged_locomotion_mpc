#include <legged_locomotion_mpc/trajectory_planners/ContactForceWrenchTrajectoryPlanner.hpp>

namespace legged_locomotion_mpc
{
  namespace planners
  {
    
    using namespace ocs2;
    using namespace floating_base_model;
    using namespace floating_base_model::access_helper_functions;

    ContactForceWrenchTrajectoryPlanner::ContactForceWrenchTrajectoryPlanner(
      FloatingBaseModelInfo modelInfo): modelInfo_(std::move(modelInfo)) {}
      
    void ContactForceWrenchTrajectoryPlanner::updateTargetTrajectory(
      std::vector<contact_flags_t> contactFlagsTrajectory,
      TargetTrajectories& targetTrajectories)
    {
      assert(contactFlagsTrajectory.size() == targetTrajectories.timeTrajectory.size());
      
      const size_t referenceSize = targetTrajectories.timeTrajectory.size();

      vector_t& firstInput = targetTrajectories.inputTrajectory[0];
      const contact_flags_t& firstFlags = contactFlagsTrajectory[0];
      utils::weightCompensatingAppendInput(firstInput, modelInfo_, firstFlags);
      for(size_t i = 1; i < referenceSize; ++i)
      {
        vector_t& currentInput = targetTrajectories.inputTrajectory[i];
        const contact_flags_t& currentFlags = contactFlagsTrajectory[i];
        const contact_flags_t& previousFlags = contactFlagsTrajectory[i - 1];
        if(currentFlags == previousFlags)
        {
          const vector_t& previousInput = targetTrajectories.inputTrajectory[i - 1];
          currentInput = previousInput;
        }
        else
        {
          utils::weightCompensatingAppendInput(currentInput, modelInfo_, currentFlags);
        }
      } 
    }
  } // namespace planners
} // namespace legged_locomotion_mp