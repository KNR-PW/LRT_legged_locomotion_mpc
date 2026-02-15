#include <gtest/gtest.h>

#include <legged_locomotion_mpc/collision/CollisionSettings.hpp>

#include "../include/definitions.hpp"

using namespace ocs2;
using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::collision;

TEST(CollisionSettingsTest, loader)
{
  const std::string modelFilePath = meldogConfigFolder + "model_settings.info";
  const std::string collisionFilePath = meldogConfigFolder + "collision_settings.info";

  const auto modelSettings = loadModelSettings(modelFilePath);

  const auto collisionSettings = loadCollisionSettings(collisionFilePath, modelSettings);

  std::vector<std::pair<std::string, std::string>> trueSelfCollisionPairNames;
  trueSelfCollisionPairNames.push_back(std::make_pair("LFLL_link", "RFLL_link"));
  trueSelfCollisionPairNames.push_back(std::make_pair("RRLL_link", "LRLL_link"));

  std::vector<scalar_t> trueExcesses = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8};
  std::vector<scalar_t> trueRelaxations = {0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1};

  EXPECT_TRUE(collisionSettings.collisionLinkNames == meldogCollisions);
  EXPECT_TRUE(collisionSettings.selfCollisionPairNames == trueSelfCollisionPairNames);
  EXPECT_TRUE(collisionSettings.maxExcesses == trueExcesses);
  EXPECT_TRUE(collisionSettings.relaxations == trueRelaxations);
  EXPECT_TRUE(collisionSettings.shrinkRatio == 0.1);
}