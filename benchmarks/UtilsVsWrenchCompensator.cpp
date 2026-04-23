#include <benchmark/benchmark.h>

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/locomotion/GaitCommon.hpp>
#include <legged_locomotion_mpc/common/Utils.hpp>
#include <legged_locomotion_mpc/common/ModelHelperFunctions.hpp>
#include <legged_locomotion_mpc/weight_compensation/PinocchioWeightCompensator.hpp>

#include "../test/include/definitions.hpp"

#include <floating_base_model/FactoryFunctions.hpp>
#include <floating_base_model/AccessHelperFunctions.hpp>
#include <floating_base_model/ModelHelperFunctions.hpp>
#include <floating_base_model/PinocchioFloatingBaseDynamics.hpp>

using namespace ocs2;
using namespace floating_base_model;
using namespace legged_locomotion_mpc;
using namespace access_helper_functions;

static void UtilsInput(benchmark::State & state)
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

  size_t randomMode = std::rand() % 16;

  auto contactFlags = locomotion::modeNumber2ContactFlags(randomMode);
  if(contactFlags[2] == 0 && contactFlags[3] == 0) contactFlags[2] = 1;

  for (auto _ : state) 
  {
    benchmark::DoNotOptimize(utils::weightCompensatingInput(info, contactFlags));
  }
}
static void WrenchCompensatorInput(benchmark::State & state)
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

  const vector_t robotState = vector_t::Random(info.stateDim);

  size_t randomMode = std::rand() % 16;

  auto contactFlags = locomotion::modeNumber2ContactFlags(randomMode);
  if(contactFlags[2] == 0 && contactFlags[3] == 0) contactFlags[2] = 1;

  for (auto _ : state) 
  {
    benchmark::DoNotOptimize(compensator.getInput(robotState, contactFlags));
  }
}
BENCHMARK(UtilsInput);
BENCHMARK(WrenchCompensatorInput);
BENCHMARK_MAIN();