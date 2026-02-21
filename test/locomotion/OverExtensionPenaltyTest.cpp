#include <gtest/gtest.h>

#include <legged_locomotion_mpc/locomotion/OverExtensionPenalty.hpp>

#include <legged_locomotion_mpc/locomotion/GaitPlanner.hpp>
#include <legged_locomotion_mpc/trajectory_planners/BaseTrajectoryPlanner.hpp>

#include <pinocchio/algorithm/frames.hpp>

#include <floating_base_model/FactoryFunctions.hpp>

#include <terrain_model/core/TerrainPlane.hpp>
#include <terrain_model/planar_model/PlanarTerrainModel.hpp>

#include "../test/include/definitions.hpp"

using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::locomotion;
using namespace legged_locomotion_mpc::planners;
using namespace floating_base_model;
using namespace ocs2;
using namespace terrain_model;

const double tolerance = 1e-9;
const size_t TEST_NUM = 100;

// Not round because std::lower_bound is sensitive for time points of mode change
const size_t ITERATIONS = 50;

TEST(OverExtensionPenaltyTest, getPenaltiesAndGetPenalty)
{
  std::string urdfPathName = meldogWithBaseLinkUrdfFile;

  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);

  FloatingBaseModelPinocchioMapping mapping(modelInfo);
  mapping.setPinocchioInterface(interface);
  
  ModelSettings modelSettings;
  modelSettings.baseLinkName =  baseLink;
  modelSettings.contactNames3DoF = meldog3DofContactNames;
  modelSettings.contactNames6DoF = meldog6DofContactNames;
  modelSettings.hipFrameNames = meldogHipNames;

  OverExtensionPenalty::Settings penaltySettings;
  penaltySettings.nominalLegExtension = 0.25;
  penaltySettings.legOverExtensionWeight = 1.0;

  const std::string modelName = "over_extension_penalty";

  OverExtensionPenalty penalty(interface, 
    modelSettings, penaltySettings, modelInfo, modelName);

  const auto model = interface.getModel();
  auto data = interface.getData();
  
  std::vector<size_t> hipFrames;
  for(const auto& name: meldogHipNames)
  {
    hipFrames.push_back(model.getFrameId(name));
  }

  // Maximal extension
  const vector_t state = vector_t::Zero(modelInfo.stateDim);
  const auto q = mapping.getPinocchioJointPosition(state);
  pinocchio::framesForwardKinematics(model, data, q);

  const auto hipFrame = hipFrames[0];
  const vector3_t maxHipPositon = data.oMf[hipFrame].translation();

  const auto endEffectorIndex = modelInfo.endEffectorFrameIndices[0];
  const vector3_t maxEndEffectorPosition = data.oMf[endEffectorIndex].translation();

  const scalar_t maxLentgh = (maxEndEffectorPosition - maxHipPositon).norm();
  const scalar_t maxExtension = (maxLentgh - penaltySettings.nominalLegExtension);
  const scalar_t maxPenaltyValue = penaltySettings.legOverExtensionWeight * maxExtension 
    * maxExtension;

  for(size_t i = 0; i < TEST_NUM; ++i)
  {
    const vector_t state = vector_t::Random(modelInfo.stateDim);
    const auto q = mapping.getPinocchioJointPosition(state);
    pinocchio::framesForwardKinematics(model, data, q);

    const auto penalties = penalty.getPenalties(state);

    for(size_t j = 0; j < hipFrames.size(); ++j)
    {
      const auto& penaltyFunction = penalties[j];
      const vector3_t queryPosition = vector3_t::Random();
      const auto hipFrame = hipFrames[j];
      const vector3_t trueHipPositon = data.oMf[hipFrame].translation();

      const auto endEffectorIndex = modelInfo.endEffectorFrameIndices[j];
      const vector3_t trueEndEffectorPosition = data.oMf[endEffectorIndex].translation();

      const scalar_t extension = std::max(0.0, (queryPosition - trueHipPositon).norm() - penaltySettings.nominalLegExtension);
      const scalar_t truePenalty = penaltySettings.legOverExtensionWeight * extension * extension;
    
      EXPECT_TRUE(penaltyFunction(trueEndEffectorPosition) < maxPenaltyValue);
      EXPECT_TRUE(std::abs(truePenalty - penaltyFunction(queryPosition)) < tolerance);
      if(((queryPosition - trueHipPositon).norm() - penaltySettings.nominalLegExtension) < 0.0)
      {
         EXPECT_TRUE(penaltyFunction(queryPosition) < tolerance); 
      }

      const auto penaltyFunctionSingle = penalty.getPenalty(j, state);
      EXPECT_TRUE(std::abs(penaltyFunctionSingle(queryPosition) 
        - penaltyFunction(queryPosition)) < tolerance);
    }
  }
}