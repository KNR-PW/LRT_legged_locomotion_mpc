#include <gtest/gtest.h>

#include <legged_locomotion_mpc/common/ModelSettings.hpp>

#include "../include/definitions.hpp"

using namespace ocs2;
using namespace legged_locomotion_mpc;

TEST(ModelSettingsTest, loader)
{
  const std::string filePath = meldogConfigFolder + "model_settings.info";

  const auto modelSettings = loadModelSettings(filePath);

  std::vector<std::pair<std::string, std::string>> trueSelfCollisionPairNames;
  trueSelfCollisionPairNames.push_back(std::make_pair("LFLL_link", "RFLL_link"));
  trueSelfCollisionPairNames.push_back(std::make_pair("RRLL_link", "LRLL_link"));

  std::vector<scalar_t> trueExcesses = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8};
  std::vector<scalar_t> trueRelaxations = {0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1};

  EXPECT_TRUE(modelSettings.verboseCppAd == false);
  EXPECT_TRUE(modelSettings.recompileLibrariesCppAd == false);
  EXPECT_TRUE(modelSettings.modelFolderCppAd == "/tmp/legged_locomotion_mpc");
  EXPECT_TRUE(modelSettings.baseLinkName == "trunk_link");
  EXPECT_TRUE(modelSettings.contactNames3DoF == meldog3DofContactNames);
  EXPECT_TRUE(modelSettings.contactNames6DoF.size() == 0);
  EXPECT_TRUE(modelSettings.collisionLinkNames == meldogCollisions);
  EXPECT_TRUE(modelSettings.selfCollisionPairNames == trueSelfCollisionPairNames);
  EXPECT_TRUE(modelSettings.maxExcesses == trueExcesses);
  EXPECT_TRUE(modelSettings.relaxations == trueRelaxations);
  EXPECT_TRUE(modelSettings.shrinkRatio == 0.1);
}