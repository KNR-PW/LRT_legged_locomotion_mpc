#include <legged_locomotion_mpc/common/Utils.hpp>

#include <ocs2_core/misc/LinearInterpolation.h>


namespace legged_locomotion_mpc
{ 
  namespace utils
  {

    using namespace ocs2;
    using namespace floating_base_model;
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    std::pair<vector_t, vector_t> robotStateToOptimizationStateAndInput(
      const floating_base_model::FloatingBaseModelInfo& info,
      const vector_t& robotState)
    {
      const vector_t state = robotState.block(0, 0, info.stateDim, 1);
      vector_t input = vector_t::Zero(info.inputDim);
      input.block(3 * info.numThreeDofContacts + 6 * info.numSixDofContacts, 0,
        info.actuatedDofNum, 1) = robotState.block(info.stateDim, 0, info.actuatedDofNum, 1);
      return {state, input};
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    TargetTrajectories subsampleReferenceTrajectory(
      const TargetTrajectories& targetTrajectories,
      scalar_t initTime,
      scalar_t finalTime,
      scalar_t maximumReferenceSampleInterval)
    {
      // Need a copy to
      // 1. possibly overwrite joint references later (adapted with inverse kinematics)
      // 2. ensure a maximum interval between references points.
      // 3. unsure we have samples at start and end of the MPC horizon.
      if (targetTrajectories.empty()) 
      {
        throw std::runtime_error("[SwingTrajectoryPlanner] provided target "
          "trajectory cannot be empty.");
      }

      TargetTrajectories newTargetTrajectories;
      // Add first reference
      {
        const auto initInterpIndex = LinearInterpolation::timeSegment(
          initTime, targetTrajectories.timeTrajectory);

        newTargetTrajectories.timeTrajectory.push_back(initTime);

        newTargetTrajectories.stateTrajectory.push_back(
          LinearInterpolation::interpolate(initInterpIndex, targetTrajectories.stateTrajectory));

        newTargetTrajectories.inputTrajectory.push_back(
          LinearInterpolation::interpolate(initInterpIndex, targetTrajectories.inputTrajectory));
      }
      bool finalTimeFlag = true;

      for (int k = 0; k < targetTrajectories.timeTrajectory.size(); ++k) 
      {
        if (targetTrajectories.timeTrajectory[k] < initTime) 
        {
          continue; // Drop all samples before init time
        } 
        else if (targetTrajectories.timeTrajectory[k] > finalTime && finalTimeFlag) 
        {
          // Do it one time only!
          finalTimeFlag = false;

          // Add final time sample. Samples after final time are also kept for 
          // touchdowns after the horizon.
          const auto finalInterpIndex = LinearInterpolation::timeSegment(
            finalTime, targetTrajectories.timeTrajectory);

          newTargetTrajectories.timeTrajectory.push_back(finalTime);

          newTargetTrajectories.stateTrajectory.push_back(
            LinearInterpolation::interpolate(finalInterpIndex, targetTrajectories.stateTrajectory));

          newTargetTrajectories.inputTrajectory.push_back(
            LinearInterpolation::interpolate(finalInterpIndex, targetTrajectories.inputTrajectory));
        }

        // Check if we need to add extra intermediate samples
        while (newTargetTrajectories.timeTrajectory.back() + 
          maximumReferenceSampleInterval < targetTrajectories.timeTrajectory[k]) 
        {
          const scalar_t t = newTargetTrajectories.timeTrajectory.back() + maximumReferenceSampleInterval;

          const auto interpIndex = LinearInterpolation::timeSegment(t, targetTrajectories.timeTrajectory);

          newTargetTrajectories.timeTrajectory.push_back(t);

          newTargetTrajectories.stateTrajectory.push_back(
            LinearInterpolation::interpolate(interpIndex, targetTrajectories.stateTrajectory));

          newTargetTrajectories.inputTrajectory.push_back(
            LinearInterpolation::interpolate(interpIndex, targetTrajectories.inputTrajectory));
        }
        // Add the original reference sample
        newTargetTrajectories.timeTrajectory.push_back(targetTrajectories.timeTrajectory[k]);
        newTargetTrajectories.stateTrajectory.push_back(targetTrajectories.stateTrajectory[k]);
        newTargetTrajectories.inputTrajectory.push_back(targetTrajectories.inputTrajectory[k]);
      }
      return newTargetTrajectories;
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    size_t numberOfClosedContacts(const FloatingBaseModelInfo &info,
      const contact_flags_t &contactFlags) 
    {
      size_t numEndEffectors = info.numThreeDofContacts + info.numSixDofContacts;

      return contactFlags.count() > numEndEffectors ? numEndEffectors : contactFlags.count();
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    vector_t weightCompensatingInput(const FloatingBaseModelInfo &info, 
      const contact_flags_t &contactFlags)
    {
      const auto numStanceLegs = numberOfClosedContacts(info, contactFlags);
      size_t numEndEffectors = info.numThreeDofContacts + info.numSixDofContacts;
      vector_t input = vector_t::Zero(info.inputDim);
      if (numStanceLegs > 0) 
      {
        const scalar_t totalWeight = info.robotMass * PLUS_GRAVITY_VALUE;
        const vector3_t forceInInertialFrame(0.0, 0.0, totalWeight / numStanceLegs);
        for (size_t i = 0; i < numEndEffectors; i++) 
        {
          if (contactFlags[i]) {
            access_helper_functions::getContactForces(input, i, info) = forceInInertialFrame;
          }
        } 
      }
      return input;
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void weightCompensatingAppendInput(vector_t& input, const FloatingBaseModelInfo &info, 
      const contact_flags_t &contactFlags)
    {
      const auto numStanceLegs = numberOfClosedContacts(info, contactFlags);
      size_t numEndEffectors = info.numThreeDofContacts + info.numSixDofContacts;
      if (numStanceLegs > 0) 
      {
        const scalar_t totalWeight = info.robotMass * PLUS_GRAVITY_VALUE;
        const vector3_t forceInInertialFrame(0.0, 0.0, totalWeight / numStanceLegs);
        for (size_t i = 0; i < numEndEffectors; i++) 
        {
          if (contactFlags[i]) {
            access_helper_functions::getContactForces(input, i, info) = forceInInertialFrame;
          }
        } 
      }
    }
    
  } // namsespace utils
} // namespace legged_locomotiom_mpc