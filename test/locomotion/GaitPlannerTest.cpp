#include <gtest/gtest.h>

#include <legged_locomotion_mpc/locomotion/GaitPlanner.hpp>

using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::locomotion;



TEST(GaitPlannerTest, Constructor)
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

  auto modeSequenceTemplate = getDynamicModeSequenceTemplate(currentPhase,
    staticParams.timeHorizion, staticParams, dynamicParams);

  GaitPlanner gaitPlanner(staticParams, dynamicParams, modeSequenceTemplate, currentPhase);

  ocs2::scalar_t timeHorizon = 3 * staticParams.timeHorizion;
  auto modeSchedule = gaitPlanner.getModeSchedule(0.0, timeHorizon);

  std::vector<ocs2::scalar_t> goodTimings = {0, 0.3, 0.35, 0.65, 0.7, 1, 1.05, 1.35, 1.4, 1.7, 1.75, 2.05, 2.1};
  std::vector<size_t> goodSequence = {15, 9, 15, 6, 15, 9, 15, 6, 15, 9, 15, 6, 15, 15};

  ASSERT_TRUE(modeSchedule.modeSequence == goodSequence);

  for(size_t i = 0; i < goodTimings.size(); ++i)
  {
    ASSERT_NEAR(modeSchedule.eventTimes[i], goodTimings[i], 1e-6);
  }
}


TEST(GaitPlannerTest, getModeSchedule)
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

  auto modeSequenceTemplate = getDynamicModeSequenceTemplate(currentPhase,
    staticParams.timeHorizion, staticParams, dynamicParams);

  GaitPlanner gaitPlanner(staticParams, dynamicParams, modeSequenceTemplate, currentPhase);

  ocs2::scalar_t startTime = 0.5;
  ocs2::scalar_t finalTime = 3 * staticParams.timeHorizion;
  auto modeSchedule = gaitPlanner.getModeSchedule(startTime, finalTime);

  std::vector<ocs2::scalar_t> goodTimings = {0.35, 0.65, 0.7, 1, 1.05, 1.35, 1.4, 1.7, 1.75, 2.05, 2.1};
  std::vector<size_t> goodSequence = {15, 6, 15, 9, 15, 6, 15, 9, 15, 6, 15, 15};

  ASSERT_TRUE(modeSchedule.modeSequence == goodSequence);

  for(size_t i = 0; i < goodTimings.size(); ++i)
  {
    ASSERT_NEAR(modeSchedule.eventTimes[i], goodTimings[i], 1e-6);
  }

  startTime = 0.65;
  finalTime = 3 * staticParams.timeHorizion;
  modeSchedule = gaitPlanner.getModeSchedule(startTime, finalTime);
  std::cerr << modeSchedule << std::endl;

  goodTimings = {0.65, 0.7, 1, 1.05, 1.35, 1.4, 1.7, 1.75, 2.05, 2.1};
  goodSequence = {6, 15, 9, 15, 6, 15, 9, 15, 6, 15, 15};

  ASSERT_TRUE(modeSchedule.modeSequence == goodSequence);

  for(size_t i = 0; i < goodTimings.size(); ++i)
  {
    ASSERT_NEAR(modeSchedule.eventTimes[i], goodTimings[i], 1e-6);
  }
}
