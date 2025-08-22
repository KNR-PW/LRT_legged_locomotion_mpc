#include <gtest/gtest.h>

#include <legged_locomotion_mpc/locomotion/GaitDynamicPhaseController.hpp>
#include <legged_locomotion_mpc/locomotion/GaitCommon.hpp>

using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::locomotion;
using namespace ocs2;

const scalar_t eps = 1e-6;

TEST(GaitDynamicPhaseController, getPhaseAtTime)
{
  /* STANDING TROT */
  ocs2::scalar_t currentPhase = 3.5 / 7.0;

  GaitStaticParameters staticParams;
  staticParams.endEffectorNumber = 4;
  staticParams.plannerFrequency = 2.0;
  staticParams.timeHorizion = 0.7;

  GaitDynamicParameters dynamicParams;
  dynamicParams.steppingFrequency = 1.0 / 0.7;
  dynamicParams.swingRatio = 3.0 / 7.0;
  
  dynamicParams.phaseOffsets = {-currentPhase , -currentPhase , 0};

  GaitDynamicPhaseController gaitController(currentPhase,
    0.0, staticParams, dynamicParams);
  
  scalar_t finalTime = 5.0;

  auto phases = gaitController.getPhasesAtTime(5.0);

  scalar_t trueNotNormalizedPhase = currentPhase + finalTime * dynamicParams.steppingFrequency;
  std::vector<scalar_t> truePhases{normalizePhase(trueNotNormalizedPhase), 
    normalizePhase(trueNotNormalizedPhase + dynamicParams.phaseOffsets [0]),
    normalizePhase(trueNotNormalizedPhase + dynamicParams.phaseOffsets [1]),
    normalizePhase(trueNotNormalizedPhase + dynamicParams.phaseOffsets [2])};
  
  for(size_t i = 0; i < phases.size(); ++i)
  {
    ASSERT_NEAR(phases[i], truePhases[i], eps);
  }
}

TEST(GaitDynamicPhaseController, update)
{
  /* STANDING TROT */
  ocs2::scalar_t currentPhase = 3.5 / 7.0;

  GaitStaticParameters staticParams;
  staticParams.endEffectorNumber = 4;
  staticParams.plannerFrequency = 2.0;
  staticParams.timeHorizion = 0.7;

  GaitDynamicParameters dynamicParams;
  dynamicParams.steppingFrequency = 1.0 / 0.7;
  dynamicParams.swingRatio = 3.0 / 7.0;
  
  dynamicParams.phaseOffsets = {-currentPhase , -currentPhase , 0};

  GaitDynamicPhaseController gaitController(currentPhase,
    0.0, staticParams, dynamicParams);

  scalar_t nextTime = 2.0;

  scalar_t trueNotNormalizedPhase = currentPhase + nextTime * dynamicParams.steppingFrequency;
  
  /* FLYING TROT */
  staticParams.timeHorizion = 1.2;
  dynamicParams.swingRatio =  0.33 / 0.6;
  dynamicParams.steppingFrequency = 1.0 / 0.6;
  currentPhase = dynamicParams.swingRatio;

  dynamicParams.phaseOffsets = {-currentPhase + 0.03 / 0.6, -currentPhase + 0.03 / 0.6, 0};
  
  gaitController.update(2.0, dynamicParams);

  scalar_t finalTime = 5.0;

  auto phases = gaitController.getPhasesAtTime(finalTime);

  trueNotNormalizedPhase += (finalTime - nextTime) * dynamicParams.steppingFrequency;

  std::vector<scalar_t> truePhases{normalizePhase(trueNotNormalizedPhase), 
    normalizePhase(trueNotNormalizedPhase + dynamicParams.phaseOffsets[0]),
    normalizePhase(trueNotNormalizedPhase + dynamicParams.phaseOffsets[1]),
    normalizePhase(trueNotNormalizedPhase + dynamicParams.phaseOffsets[2])};

  for(size_t i = 0; i < phases.size(); ++i)
  {
    ASSERT_NEAR(phases[i], truePhases[i], eps);
  }


  /* STANDING TROT AGAIN */
  currentPhase = 3.5 / 7.0;

  staticParams.endEffectorNumber = 4;
  staticParams.plannerFrequency = 2.0;
  staticParams.timeHorizion = 0.7;

  dynamicParams.steppingFrequency = 1.0 / 0.7;
  dynamicParams.swingRatio = 3.0 / 7.0;
  
  dynamicParams.phaseOffsets = {-currentPhase , -currentPhase , 0};

  nextTime = 2.0;

  trueNotNormalizedPhase = currentPhase + nextTime * dynamicParams.steppingFrequency;
  
  phases = gaitController.getPhasesAtTime(nextTime);

  
  truePhases = {normalizePhase(trueNotNormalizedPhase), 
    normalizePhase(trueNotNormalizedPhase + dynamicParams.phaseOffsets[0]),
    normalizePhase(trueNotNormalizedPhase + dynamicParams.phaseOffsets[1]),
    normalizePhase(trueNotNormalizedPhase + dynamicParams.phaseOffsets[2])};

  for(size_t i = 0; i < phases.size(); ++i)
  {
    ASSERT_NEAR(phases[i], truePhases[i], eps);
  }

}

TEST(GaitDynamicPhaseController, remove)
{
  /* STANDING TROT */
  ocs2::scalar_t currentPhase = 3.5 / 7.0;

  GaitStaticParameters staticParams;
  staticParams.endEffectorNumber = 4;
  staticParams.plannerFrequency = 2.0;
  staticParams.timeHorizion = 0.7;

  GaitDynamicParameters dynamicParams;
  dynamicParams.steppingFrequency = 1.0 / 0.7;
  dynamicParams.swingRatio = 3.0 / 7.0;
  
  dynamicParams.phaseOffsets = {-currentPhase , -currentPhase , 0};

  GaitDynamicPhaseController gaitController(currentPhase,
    0.0, staticParams, dynamicParams);

  scalar_t nextTime = 2.0;

  scalar_t trueNotNormalizedPhase = currentPhase + nextTime * dynamicParams.steppingFrequency;
  
  /* FLYING TROT */
  staticParams.timeHorizion = 1.2;
  dynamicParams.swingRatio =  0.33 / 0.6;
  dynamicParams.steppingFrequency = 1.0 / 0.6;
  currentPhase = dynamicParams.swingRatio;

  dynamicParams.phaseOffsets = {-currentPhase + 0.03 / 0.6, -currentPhase + 0.03 / 0.6, 0};
  
  gaitController.update(2.0, dynamicParams);

  scalar_t finalTime = 5.0;

  gaitController.remove(4.5);

  auto phases = gaitController.getPhasesAtTime(finalTime);

  trueNotNormalizedPhase += (finalTime - nextTime) * dynamicParams.steppingFrequency;

  std::vector<scalar_t> truePhases{normalizePhase(trueNotNormalizedPhase), 
    normalizePhase(trueNotNormalizedPhase + dynamicParams.phaseOffsets [0]),
    normalizePhase(trueNotNormalizedPhase + dynamicParams.phaseOffsets [1]),
    normalizePhase(trueNotNormalizedPhase + dynamicParams.phaseOffsets [2])};

  for(size_t i = 0; i < phases.size(); ++i)
  {
    ASSERT_NEAR(phases[i], truePhases[i], eps);
  }

}

TEST(GaitDynamicPhaseController, getContactFlagsAtTime)
{
  /* STANDING TROT */
  ocs2::scalar_t currentPhase = 3.5 / 7.0;

  GaitStaticParameters staticParams;
  staticParams.endEffectorNumber = 4;
  staticParams.plannerFrequency = 2.0;
  staticParams.timeHorizion = 0.7;

  GaitDynamicParameters dynamicParams;
  dynamicParams.steppingFrequency = 1.0 / 0.7;
  dynamicParams.swingRatio = 3.0 / 7.0;
  
  dynamicParams.phaseOffsets = {-currentPhase , -currentPhase , 0};

  GaitDynamicPhaseController gaitController(currentPhase,
    0.0, staticParams, dynamicParams);

  scalar_t nextTime = 2.0;

  scalar_t trueNotNormalizedPhase = currentPhase + nextTime * dynamicParams.steppingFrequency;
  
  /* FLYING TROT */
  staticParams.timeHorizion = 1.2;
  dynamicParams.swingRatio =  0.33 / 0.6;
  dynamicParams.steppingFrequency = 1.0 / 0.6;
  currentPhase = dynamicParams.swingRatio;

  dynamicParams.phaseOffsets = {-currentPhase + 0.03 / 0.6, -currentPhase + 0.03 / 0.6, 0};
  
  gaitController.update(2.0, dynamicParams);

  scalar_t finalTime = 5.0;

  auto contacts = gaitController.getContactFlagsAtTime(finalTime);

  trueNotNormalizedPhase += (finalTime - nextTime) * dynamicParams.steppingFrequency;

  contact_flags_t trueContacts;
  if(normalizePhase(trueNotNormalizedPhase) >= dynamicParams.swingRatio)
  {
    trueContacts[0] = true;
  }
  for(size_t i = 1; i < staticParams.endEffectorNumber; ++i)
  {
    if(normalizePhase(trueNotNormalizedPhase + dynamicParams.phaseOffsets[i - 1]) >= dynamicParams.swingRatio)
    {
      trueContacts[i] = true;
    }
  }

  ASSERT_TRUE(contacts == trueContacts);

}


TEST(GaitDynamicPhaseController, getDynamicParametersAtTime)
{
  /* STANDING TROT */
  ocs2::scalar_t currentPhase = 3.5 / 7.0;

  GaitStaticParameters staticParams1;
  staticParams1.endEffectorNumber = 4;
  staticParams1.plannerFrequency = 2.0;
  staticParams1.timeHorizion = 0.7;

  GaitDynamicParameters dynamicParams1;
  dynamicParams1.steppingFrequency = 1.0 / 0.7;
  dynamicParams1.swingRatio = 3.0 / 7.0;
  
  dynamicParams1.phaseOffsets = {-currentPhase , -currentPhase , 0};

  GaitDynamicPhaseController gaitController(currentPhase,
    0.0, staticParams1, dynamicParams1);

  scalar_t nextTime = 2.0;
  
  /* FLYING TROT */
  GaitDynamicParameters dynamicParams2;
  dynamicParams2.swingRatio =  0.33 / 0.6;
  dynamicParams2.steppingFrequency = 1.0 / 0.6;
  currentPhase = dynamicParams2.swingRatio;

  dynamicParams2.phaseOffsets = {-currentPhase + 0.03 / 0.6, -currentPhase + 0.03 / 0.6, 0};
  
  gaitController.update(2.0, dynamicParams2);

  scalar_t finalTime = 5.0;

  auto dynamicParamsTest1 = gaitController.getDynamicParametersAtTime(1.0);
  auto dynamicParamsTest2 = gaitController.getDynamicParametersAtTime(3.0);
  auto dynamicParamsTest3 = gaitController.getDynamicParametersAtTime(2.0);
  auto dynamicParamsTest4 = gaitController.getDynamicParametersAtTime(100.0);

  ASSERT_TRUE(dynamicParamsTest1.phaseOffsets == dynamicParams1.phaseOffsets);
  ASSERT_TRUE(dynamicParamsTest2.phaseOffsets == dynamicParams2.phaseOffsets);
  ASSERT_TRUE(dynamicParamsTest3.phaseOffsets == dynamicParams1.phaseOffsets);
  ASSERT_TRUE(dynamicParamsTest4.phaseOffsets == dynamicParams2.phaseOffsets);

  ASSERT_TRUE(dynamicParamsTest1.steppingFrequency == dynamicParams1.steppingFrequency);
  ASSERT_TRUE(dynamicParamsTest2.steppingFrequency == dynamicParams2.steppingFrequency);
  ASSERT_TRUE(dynamicParamsTest3.steppingFrequency == dynamicParams1.steppingFrequency);
  ASSERT_TRUE(dynamicParamsTest4.steppingFrequency == dynamicParams2.steppingFrequency);

  ASSERT_TRUE(dynamicParamsTest1.swingRatio == dynamicParams1.swingRatio);
  ASSERT_TRUE(dynamicParamsTest2.swingRatio == dynamicParams2.swingRatio);
  ASSERT_TRUE(dynamicParamsTest3.swingRatio == dynamicParams1.swingRatio);
  ASSERT_TRUE(dynamicParamsTest4.swingRatio == dynamicParams2.swingRatio);

}