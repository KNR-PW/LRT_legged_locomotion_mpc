#include <gtest/gtest.h>

#include <legged_locomotion_mpc/locomotion/GaitPlanner.hpp>
#include <legged_locomotion_mpc/locomotion/GaitCommon.hpp>

using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::locomotion;
using namespace ocs2;


TEST(GaitPlannerTest, Constructor)
{
  /* STANDING TROT */
  scalar_t currentPhase = 3.5 / 7.0;

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

  GaitPlanner gaitPlanner(staticParams, dynamicParams, modeSequenceTemplate, currentPhase, 0.0);

  scalar_t timeHorizon = 3 * staticParams.timeHorizion;
  auto modeSchedule = gaitPlanner.getModeSchedule(0.0, timeHorizon);

  std::vector<scalar_t> goodTimings = {0, 0.3, 0.35, 0.65, 0.7, 1, 1.05, 1.35, 1.4, 1.7, 1.75, 2.05, 2.1};
  std::vector<size_t> goodSequence = {15, 9, 15, 6, 15, 9, 15, 6, 15, 9, 15, 6, 15, 15};

  ASSERT_TRUE(modeSchedule.modeSequence == goodSequence);

  for(size_t i = 0; i < goodTimings.size(); ++i)
  {
    ASSERT_NEAR(modeSchedule.eventTimes[i], goodTimings[i], 1e-6);
  }

  const scalar_t MIN_TIME_BETWEEN_CHANGES = 1e-4;

  for(size_t i = 1; i < modeSchedule.eventTimes.size(); ++i)
  {
    scalar_t deltaTime = modeSchedule.eventTimes[i] - modeSchedule.eventTimes[i - 1];
    ASSERT_GT(deltaTime, MIN_TIME_BETWEEN_CHANGES);
  }

  for(int i = 0; i < 10; ++i)
  {
    scalar_t time = i * timeHorizon / 10.0 + 1e-6;
    auto contactFlags = gaitPlanner.getContactFlagsAtTime(time);
    auto trueContactFlags = modeNumber2ContactFlags(modeSchedule.modeAtTime(time));
    ASSERT_TRUE(contactFlags == trueContactFlags);
  }

}


TEST(GaitPlannerTest, getModeSchedule)
{
  /* STANDING TROT */
  scalar_t currentPhase = 3.5 / 7.0;

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

  GaitPlanner gaitPlanner(staticParams, dynamicParams, modeSequenceTemplate, currentPhase, 0.0);

  scalar_t startTime = 0.5;
  scalar_t finalTime = 3 * staticParams.timeHorizion;
  auto modeSchedule = gaitPlanner.getModeSchedule(startTime, finalTime);

  std::vector<scalar_t> goodTimings = {0.35, 0.65, 0.7, 1, 1.05, 1.35, 1.4, 1.7, 1.75, 2.05, 2.1};
  std::vector<size_t> goodSequence = {15, 6, 15, 9, 15, 6, 15, 9, 15, 6, 15, 15};

  ASSERT_TRUE(modeSchedule.modeSequence == goodSequence);
  
  std::vector<scalar_t> times;
  for(int i = 0; i < 10; ++i)
  {
    scalar_t time = startTime + i * (finalTime - startTime)  / 10.0 + 1e-6;
    times.push_back(time);
    auto contactFlags = gaitPlanner.getContactFlagsAtTime(time);
    auto trueContactFlags = modeNumber2ContactFlags(modeSchedule.modeAtTime(time));
    ASSERT_TRUE(contactFlags == trueContactFlags);
  }

  std::vector<contact_flags_t> flagsTrajectory = gaitPlanner.getContactFlagsAtTimes(times);

  for(size_t i = 0; i < 10; ++i)
  {
     auto trueContactFlags = modeNumber2ContactFlags(modeSchedule.modeAtTime(times[i]));
    ASSERT_TRUE(flagsTrajectory[i] == trueContactFlags);
  }

  for(size_t i = 0; i < goodTimings.size(); ++i)
  {
    ASSERT_NEAR(modeSchedule.eventTimes[i], goodTimings[i], 1e-6);
  }

  startTime = 0.65;
  finalTime = 3 * staticParams.timeHorizion;
  modeSchedule = gaitPlanner.getModeSchedule(startTime, finalTime);

  goodTimings = {0.65, 0.7, 1, 1.05, 1.35, 1.4, 1.7, 1.75, 2.05, 2.1};
  goodSequence = {6, 15, 9, 15, 6, 15, 9, 15, 6, 15, 15};

  ASSERT_TRUE(modeSchedule.modeSequence == goodSequence);

  for(size_t i = 0; i < goodTimings.size(); ++i)
  {
    ASSERT_NEAR(modeSchedule.eventTimes[i], goodTimings[i], 1e-6);
  }

  const scalar_t MIN_TIME_BETWEEN_CHANGES = 1e-4;

  for(size_t i = 1; i < modeSchedule.eventTimes.size(); ++i)
  {
    scalar_t deltaTime = modeSchedule.eventTimes[i] - modeSchedule.eventTimes[i - 1];
    ASSERT_GT(deltaTime, MIN_TIME_BETWEEN_CHANGES);
  }

  times.clear();
  for(int i = 0; i < 10; ++i)
  {
    scalar_t time = startTime + i * (finalTime - startTime)  / 10.0 + 1e-6;
    times.push_back(time);
    auto contactFlags = gaitPlanner.getContactFlagsAtTime(time);
    auto trueContactFlags = modeNumber2ContactFlags(modeSchedule.modeAtTime(time));
    ASSERT_TRUE(contactFlags == trueContactFlags);
  }

  flagsTrajectory = gaitPlanner.getContactFlagsAtTimes(times);

  for(size_t i = 0; i < 10; ++i)
  {
     auto trueContactFlags = modeNumber2ContactFlags(modeSchedule.modeAtTime(times[i]));
    ASSERT_TRUE(flagsTrajectory[i] == trueContactFlags);
  }
}


TEST(GaitPlannerTest, updateDynamicParameters)
{
  /* STANDING TROT */
  scalar_t startingPhase = 3.5 / 7.0;

  GaitStaticParameters staticParams;
  staticParams.endEffectorNumber = 4;
  staticParams.plannerFrequency = 2.0;
  staticParams.timeHorizion = 0.7;

  GaitDynamicParameters dynamicParams1;
  dynamicParams1.steppingFrequency = 1.0 / 0.7;
  dynamicParams1.swingRatio = 3.0 / 7.0;
  
  dynamicParams1.phaseOffsets = {-startingPhase , -startingPhase , 0};

  /* FLYING TROT */
  GaitDynamicParameters dynamicParams2;
  dynamicParams2.swingRatio =  0.33 / 0.6;
  dynamicParams2.steppingFrequency = 1.0 / 0.6;
  scalar_t currentPhase = dynamicParams2.swingRatio;

  dynamicParams2.phaseOffsets = {-currentPhase + 0.03 / 0.6, -currentPhase + 0.03 / 0.6, 0};

  /* DYNAMIC WALK */

  GaitDynamicParameters dynamicParams3;
  dynamicParams3.swingRatio =  0.3;
  dynamicParams3.steppingFrequency = 1.0;
  currentPhase = 0.8;

  dynamicParams3.phaseOffsets = {0.5, 0.2, 0.7};

  scalar_t startTime = 0.0;
  scalar_t firstTime = 10.0;

  auto modeSequenceTemplate = getDynamicModeSequenceTemplate(startingPhase,
    firstTime, staticParams, dynamicParams1);

  GaitPlanner gaitPlanner(staticParams, dynamicParams1, modeSequenceTemplate, startingPhase, startTime);

  auto modeSchedule = gaitPlanner.getModeSchedule(startTime, firstTime);

  const scalar_t MIN_TIME_BETWEEN_CHANGES = 1e-4;

  for(size_t i = 1; i < modeSchedule.eventTimes.size(); ++i)
  {
    scalar_t deltaTime = modeSchedule.eventTimes[i] - modeSchedule.eventTimes[i - 1];
    ASSERT_GT(deltaTime, MIN_TIME_BETWEEN_CHANGES);
  }

  for(int i = 0; i < 10; ++i)
  {
    scalar_t time = startTime + i * (firstTime - startTime)  / 10.0 + 1e-6;
    auto contactFlags = gaitPlanner.getContactFlagsAtTime(time);
    auto trueContactFlags = modeNumber2ContactFlags(modeSchedule.modeAtTime(time));
    ASSERT_TRUE(contactFlags == trueContactFlags);
  }

  scalar_t secondTime = 2.075; // Change mode template in 3 second, remove 7 seconds of first template 
  scalar_t thirdTime = 10.0;
  gaitPlanner.updateDynamicParameters(secondTime, dynamicParams2);

  modeSchedule = gaitPlanner.getModeSchedule(startTime, thirdTime);

  for(int i = 0; i < 10; ++i)
  {
    scalar_t time = startTime + i * (thirdTime - startTime)  / 10.0 + 1e-6;
    auto contactFlags = gaitPlanner.getContactFlagsAtTime(time);
    auto trueContactFlags = modeNumber2ContactFlags(modeSchedule.modeAtTime(time));
    ASSERT_TRUE(contactFlags == trueContactFlags);
  }

  for(size_t i = 1; i < modeSchedule.eventTimes.size(); ++i)
  {
    scalar_t deltaTime = modeSchedule.eventTimes[i] - modeSchedule.eventTimes[i - 1];
    ASSERT_GT(deltaTime, MIN_TIME_BETWEEN_CHANGES);
  }

  scalar_t forthTime = 5.0; // Change mode template in 5 second, remove 5 seconds of second template 
  scalar_t fifthTime = 10.0;

  gaitPlanner.updateDynamicParameters(forthTime, dynamicParams3);

  modeSchedule = gaitPlanner.getModeSchedule(startTime, fifthTime);

  for(size_t i = 1; i < modeSchedule.eventTimes.size(); ++i)
  {
    scalar_t deltaTime = modeSchedule.eventTimes[i] - modeSchedule.eventTimes[i - 1];
    ASSERT_GT(deltaTime, MIN_TIME_BETWEEN_CHANGES);
  }

  for(int i = 0; i < 10; ++i)
  {
    scalar_t time = startTime + i * (fifthTime - startTime)  / 10.0 + 1e-6;
    auto contactFlags = gaitPlanner.getContactFlagsAtTime(time);
    auto trueContactFlags = modeNumber2ContactFlags(modeSchedule.modeAtTime(time));
    ASSERT_TRUE(contactFlags == trueContactFlags);
  }

  startTime = 7.4;

  modeSchedule = gaitPlanner.getModeSchedule(startTime, fifthTime);

  for(int i = 0; i < 10; ++i)
  {
    scalar_t time = startTime + i * (fifthTime - startTime)  / 10.0 + 1e-6;
    auto contactFlags = gaitPlanner.getContactFlagsAtTime(time);
    auto trueContactFlags = modeNumber2ContactFlags(modeSchedule.modeAtTime(time));
    ASSERT_TRUE(contactFlags == trueContactFlags);
  }

}