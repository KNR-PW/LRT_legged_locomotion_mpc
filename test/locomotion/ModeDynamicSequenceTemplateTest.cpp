#include <gtest/gtest.h>

#include <legged_locomotion_mpc/locomotion/ModeDynamicSequenceTemplate.hpp>

using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::locomotion;
using namespace ocs2;

TEST(ModeDynamicSequenceTemplateTest, getter)
{

  /* STANDING TROT */

  scalar_t currentPhase = 3.5 / 7.0;

  GaitStaticParameters staticParams;
  staticParams.endEffectorNumber = 4;

  GaitDynamicParameters dynamicParams;
  dynamicParams.steppingFrequency = 1.0 / 0.7;
  dynamicParams.swingRatio = 3.0 / 7.0;
  
  dynamicParams.phaseOffsets = {-currentPhase , -currentPhase , 0};

  auto modeSequenceTemplate = getDynamicModeSequenceTemplate(currentPhase,
    0.7, staticParams, dynamicParams);

  std::vector<size_t> goodSequence = {9, 15, 6, 15};
  std::vector<scalar_t> goodTiming = {0.0, 0.3, 0.35, 0.65, 0.7};

  ASSERT_TRUE(modeSequenceTemplate.modeSequence == goodSequence);

  for(size_t i = 0; i < goodTiming.size(); ++i)
  {
    ASSERT_NEAR(modeSequenceTemplate.switchingTimes[i], goodTiming[i], 1e-6);
  }


  /* FLYING TROT */
  dynamicParams.swingRatio =  0.33 / 0.6;
  dynamicParams.steppingFrequency = 1.0 / 0.6;
  currentPhase = dynamicParams.swingRatio;

  dynamicParams.phaseOffsets = {-currentPhase + 0.03 / 0.6, -currentPhase + 0.03 / 0.6, 0};

  modeSequenceTemplate = getDynamicModeSequenceTemplate(currentPhase,
    1.2, staticParams, dynamicParams);

  goodSequence = {9, 0, 6, 0, 9, 0, 6, 0};
  goodTiming = {0.0, 0.27, 0.3, 0.57, 0.6, 0.87, 0.9, 1.17, 1.2};

  ASSERT_TRUE(modeSequenceTemplate.modeSequence == goodSequence);

  for(size_t i = 0; i < goodTiming.size(); ++i)
  {
    ASSERT_NEAR(modeSequenceTemplate.switchingTimes[i], goodTiming[i], 1e-6);
  }

  /* DYNAMIC WALK */

  dynamicParams.swingRatio =  0.3;
  dynamicParams.steppingFrequency = 1.0;
  currentPhase = 0.8;

  dynamicParams.phaseOffsets = {0.5, 0.2, 0.7};

  modeSequenceTemplate = getDynamicModeSequenceTemplate(currentPhase,
    3.0, staticParams, dynamicParams);

  goodSequence = {11, 10, 14, 7, 5, 13, 11, 10, 14, 7, 5, 13, 11, 10, 14, 7, 5, 13};
  goodTiming = {0.0, 0.2, 0.3, 0.5, 0.7, 0.8, 1.0, 1.2, 1.3, 1.5, 1.7, 1.8, 2.0,
    2.2, 2.3, 2.5, 2.7, 2.8, 3.0};

  ASSERT_TRUE(modeSequenceTemplate.modeSequence == goodSequence);

  for(size_t i = 0; i < goodTiming.size(); ++i)
  {
    ASSERT_NEAR(modeSequenceTemplate.switchingTimes[i], goodTiming[i], 1e-6);
  }

  const scalar_t MIN_TIME_BETWEEN_CHANGES = 1e-4;

  for(int i = 1; i < modeSequenceTemplate.switchingTimes.size(); ++i)
  {
    scalar_t deltaTime = modeSequenceTemplate.switchingTimes[i] - modeSequenceTemplate.switchingTimes[i - 1];
    ASSERT_GT(deltaTime, MIN_TIME_BETWEEN_CHANGES);
  }
}