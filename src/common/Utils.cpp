#include <legged_locomotion_mpc/common/Utils.hpp>


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
      size_t stateSize = 12 + info.actuatedDofNum;
      vector_t state = robotState.block(0, 0, stateSize, 1);
      vector_t input = vector_t::Zero(3 * info.numThreeDofContacts + 6 * info.numSixDofContacts + info.actuatedDofNum);
      vector_t input.block(3 * info.numThreeDofContacts + 6 * info.numSixDofContacts, 0
        info.actuatedDofNum, 1) = robotState.block(stateSize, 0, info.actuatedDofNum, 1);
      return {state, input};
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    size_t numberOfClosedContacts(const contact_flags_t &contactFlags) 
    {
      size_t numStanceLegs = 0;
      for (auto legInContact: contactFlags) 
      {
        if (legInContact) 
        {
          ++numStanceLegs;
        }
      }
      return numStanceLegs;
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    vector_t weightCompensatingInput(const FloatingBaseModelInfo &info, 
      const contact_flags_t &contactFlags)
    {
      const auto numStanceLegs = numberOfClosedContacts(contactFlags);
      vector_t input = vector_t::Zero(info.inputDim);
      if (numStanceLegs > 0) 
      {
        const scalar_t totalWeight = info.robotMass * 9.81;
        const vector3_t forceInInertialFrame(0.0, 0.0, totalWeight / numStanceLegs);
        for (size_t i = 0; i < contactFlags.size(); i++) 
        {
          if (contactFlags[i]) {
            access_helper_functions::getContactForces(input, i, info) = forceInInertialFrame;
          }
        } 
      }
      return input;
    }
  }
}