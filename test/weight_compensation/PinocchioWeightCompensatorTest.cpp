#include <gtest/gtest.h>

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/locomotion/GaitCommon.hpp>
#include <legged_locomotion_mpc/weight_compensation/PinocchioWeightCompensator.hpp>

#include "../include/definitions.hpp"

#include <floating_base_model/FactoryFunctions.hpp>
#include <floating_base_model/AccessHelperFunctions.hpp>
#include <floating_base_model/PinocchioFloatingBaseDynamics.hpp>

using namespace ocs2;
using namespace floating_base_model;
using namespace legged_locomotion_mpc;
using namespace access_helper_functions;

const scalar_t tolerance = 1e-9;
const size_t NUM_TEST = 100;

TEST(PinocchioWeightCompensator, getInput)
{
  std::vector<std::string> testThreeDofMeldogContacts(2);
  testThreeDofMeldogContacts[0] = meldog3DofContactNames[0];
  testThreeDofMeldogContacts[1] = meldog3DofContactNames[1];

  std::vector<std::string> testSixDofMeldogContacts(2);
  testSixDofMeldogContacts[0] = meldog3DofContactNames[2];
  testSixDofMeldogContacts[1] = meldog3DofContactNames[3];

  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(meldogWithBaseLinkUrdfFile, baseLink);
  auto info = createFloatingBaseModelInfo(interface, testThreeDofMeldogContacts, 
    testSixDofMeldogContacts);

  PinocchioFloatingBaseDynamics dynamics(info);
  dynamics.setPinocchioInterface(interface);

  FloatingBaseModelPinocchioMapping mapping(info);
  mapping.setPinocchioInterface(interface);

  PinocchioWeightCompensator compensator(interface, info);

  for(size_t i = 0; i < NUM_TEST; ++i)
  {
    vector_t state = vector_t::Random(info.stateDim);

    state.block<6, 1>(0, 0).setZero();

    size_t randomMode = std::rand() % 16;

    auto contactFlags = locomotion::modeNumber2ContactFlags(randomMode);
    if(contactFlags[2] == 0 && contactFlags[3] == 0) contactFlags[2] = 1;

    const vector_t q = mapping.getPinocchioJointPosition(state);

    const vector_t input = compensator.getInput(state, contactFlags);

    const vector_t acceleration = dynamics.getValue(0, state, input, vector6_t::Zero());

    EXPECT_TRUE((acceleration).norm() < tolerance);
  }
}

TEST(PinocchioWeightCompensator, appendInput)
{
  std::vector<std::string> testThreeDofMeldogContacts(2);
  testThreeDofMeldogContacts[0] = meldog3DofContactNames[0];
  testThreeDofMeldogContacts[1] = meldog3DofContactNames[1];

  std::vector<std::string> testSixDofMeldogContacts(2);
  testSixDofMeldogContacts[0] = meldog3DofContactNames[2];
  testSixDofMeldogContacts[1] = meldog3DofContactNames[3];

  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(meldogWithBaseLinkUrdfFile, baseLink);
  auto info = createFloatingBaseModelInfo(interface, testThreeDofMeldogContacts, 
    testSixDofMeldogContacts);

  PinocchioFloatingBaseDynamics dynamics(info);
  dynamics.setPinocchioInterface(interface);

  FloatingBaseModelPinocchioMapping mapping(info);
  mapping.setPinocchioInterface(interface);

  PinocchioWeightCompensator compensator(interface, info);

  for(size_t i = 0; i < NUM_TEST; ++i)
  {
    vector_t state = vector_t::Random(info.stateDim);

    state.block<6, 1>(0, 0).setZero();

    size_t randomMode = std::rand() % 16;

    auto contactFlags = locomotion::modeNumber2ContactFlags(randomMode);
    if(contactFlags[2] == 0 && contactFlags[3] == 0) contactFlags[2] = 1;

    const vector_t q = mapping.getPinocchioJointPosition(state);

    vector_t input = vector_t::Zero(info.inputDim);

    compensator.appendInput(state, input, contactFlags);
    
    const vector_t acceleration = dynamics.getValue(0, state, input, vector6_t::Zero());

    EXPECT_TRUE((acceleration).norm() < tolerance);
  }
}