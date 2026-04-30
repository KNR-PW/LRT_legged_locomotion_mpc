#include <benchmark/benchmark.h>

#include <legged_locomotion_mpc/locomotion/GaitDynamicPhaseController.hpp>

using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::locomotion;
using namespace ocs2;

static void GaitDynamicPhaseController_COPY(benchmark::State & state)
{
  /* STANDING TROT */
  ocs2::scalar_t currentPhase = 3.5 / 7.0;

  GaitStaticParameters staticParams;
  staticParams.endEffectorNumber = 4;

  GaitDynamicParameters dynamicParams;
  dynamicParams.steppingFrequency = 1.0 / 0.7;
  dynamicParams.swingRatio = 3.0 / 7.0;
  
  dynamicParams.phaseOffsets = {-currentPhase , -currentPhase , 0};

  GaitDynamicPhaseController gaitController(currentPhase,
    0.0, staticParams, dynamicParams);

  scalar_t nextTime = 2.0;

  scalar_t trueNotNormalizedPhase = currentPhase + nextTime * dynamicParams.steppingFrequency;
  
  /* FLYING TROT */
  dynamicParams.swingRatio =  0.33 / 0.6;
  dynamicParams.steppingFrequency = 1.0 / 0.6;
  currentPhase = dynamicParams.swingRatio;

  dynamicParams.phaseOffsets = {-currentPhase + 0.03 / 0.6, -currentPhase + 0.03 / 0.6, 0};
  int i = 0;
  for (auto _ : state) {
    gaitController.update(10.0 + i, dynamicParams);
  }
}

static void GaitDynamicPhaseController_MOVE(benchmark::State & state)
{
  /* STANDING TROT */
  ocs2::scalar_t currentPhase = 3.5 / 7.0;

  GaitStaticParameters staticParams;
  staticParams.endEffectorNumber = 4;

  GaitDynamicParameters dynamicParams;
  dynamicParams.steppingFrequency = 1.0 / 0.7;
  dynamicParams.swingRatio = 3.0 / 7.0;
  
  dynamicParams.phaseOffsets = {-currentPhase , -currentPhase , 0};

  GaitDynamicPhaseController gaitController(currentPhase,
    0.0, staticParams, dynamicParams);

  scalar_t nextTime = 2.0;

  scalar_t trueNotNormalizedPhase = currentPhase + nextTime * dynamicParams.steppingFrequency;
  
  /* FLYING TROT */
  dynamicParams.swingRatio =  0.33 / 0.6;
  dynamicParams.steppingFrequency = 1.0 / 0.6;
  currentPhase = dynamicParams.swingRatio;

  dynamicParams.phaseOffsets = {-currentPhase + 0.03 / 0.6, -currentPhase + 0.03 / 0.6, 0};
  int i = 0;
  for (auto _ : state) {
    gaitController.update(10.0 + i, std::move(dynamicParams));
  }
}

BENCHMARK(GaitDynamicPhaseController_COPY);
BENCHMARK(GaitDynamicPhaseController_MOVE);


BENCHMARK_MAIN();