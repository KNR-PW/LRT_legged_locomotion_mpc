#include <gtest/gtest.h>

#include <legged_locomotion_mpc/locomotion/ModeDynamicSequenceTemplate.hpp>

using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::locomotion;

TEST(ModeDynamicSequenceTemplate, getter)
{
  const ocs2::scalar_t currentPhase = 3.5 / 7.0;

  GaitStaticParameters staticParams;
  staticParams.endEffectorNumber = 4;
  staticParams.plannerFrequency = 2.0;
  staticParams.timeHorizion = 0.7;

  GaitDynamicParameters dynamicParams;
  dynamicParams.steppingFrequency = 1.0 / 0.7;
  dynamicParams.swingRatio = 3.0 / 7.0;
  
  dynamicParams.phaseOffsets = {-currentPhase , -currentPhase , 0};

  const auto& modeSequenceTemplate = getDynamicModeSequenceTemplate(currentPhase,
    staticParams.timeHorizion, staticParams, dynamicParams);

  std::vector<size_t> goodSequence = {9, 15, 6, 15};
  std::vector<ocs2::scalar_t> goodTiming = {0.0, 0.3, 0.35, 0.65, 0.7};

  std::cerr << modeSequenceTemplate << std::endl;

  ASSERT_TRUE(modeSequenceTemplate.modeSequence == goodSequence);

  for(size_t i = 0; i < goodTiming.size(); ++i)
  {
    ASSERT_NEAR(modeSequenceTemplate.switchingTimes[i], goodTiming[i], 1e-6);
  }

}