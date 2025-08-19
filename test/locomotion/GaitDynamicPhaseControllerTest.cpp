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
  
  for(int i = 0; i < phases.size(); ++i)
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

  std::cerr << "TRUE FIRST: " << trueNotNormalizedPhase << std::endl;
  
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

  std::cerr << "TRUE SECOND: " << trueNotNormalizedPhase << std::endl;

  std::vector<scalar_t> truePhases{normalizePhase(trueNotNormalizedPhase), 
    normalizePhase(trueNotNormalizedPhase + dynamicParams.phaseOffsets[0]),
    normalizePhase(trueNotNormalizedPhase + dynamicParams.phaseOffsets[1]),
    normalizePhase(trueNotNormalizedPhase + dynamicParams.phaseOffsets[2])};

  for(int i = 0; i < phases.size(); ++i)
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

  for(int i = 0; i < phases.size(); ++i)
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
  for(int i = 1; i < staticParams.endEffectorNumber; ++i)
  {
    if(normalizePhase(trueNotNormalizedPhase + dynamicParams.phaseOffsets[i - 1]) >= dynamicParams.swingRatio)
    {
      trueContacts[i] = true;
    }
  }

  ASSERT_TRUE(contacts == trueContacts);

}