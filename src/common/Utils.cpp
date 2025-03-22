#include "legged_locomotion_mpc/common/Utils.hpp"


using namespace floating_base_model;

namespace legged_locomotion_mpc
{ 
  
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
  ocs2::vector_t weightCompensatingInput(const FloatingBaseModelInfo &info, 
    const contact_flags_t &contactFlags)
  {
    const auto numStanceLegs = numberOfClosedContacts(contactFlags);
    ocs2::vector_t input = ocs2::vector_t::Zero(info.inputDim);
    if (numStanceLegs > 0) 
    {
      const ocs2::scalar_t totalWeight = info.robotMass * 9.81;
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