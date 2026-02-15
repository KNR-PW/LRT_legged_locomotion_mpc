#include <gtest/gtest.h>

#include <legged_locomotion_mpc/cost/TrajectoryTrackingCost.hpp>

#include <floating_base_model/FactoryFunctions.hpp>

#include "../test/include/definitions.hpp"

using namespace ocs2;
using namespace floating_base_model;
using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::cost;

const static scalar_t eps = 1e-9;

TEST(TrajectoryTrackingCost, loaders) 
{

  const std::string modelFilePath = meldogConfigFolder + "model_settings.info";

  const auto modelSettings = loadModelSettings(modelFilePath);

  std::string urdfPathName = meldogWithBaseLinkUrdfFile;

  PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);

  const auto& model = interface.getModel();

  const std::string filePath = meldogConfigFolder + "trajectory_tracking_cost_settings.info";

  const auto baseWeights = loadBaseWeights(filePath);

  const vector3_t trueBasePosition        = vector3_t{0.1, 0.2, 0.3};
  const vector3_t trueBaseRotation        = vector3_t{0.4, 0.5, 0.6};
  const vector3_t trueBaseLinearVelocity  = vector3_t{0.7, 0.8, 0.9};
  const vector3_t trueBaseAngularVelocity = vector3_t{1.0, 1.1, 1.2};


  EXPECT_TRUE((baseWeights.position - trueBasePosition).norm() < eps);
  EXPECT_TRUE((baseWeights.rotation - trueBaseRotation).norm() < eps);
  EXPECT_TRUE((baseWeights.linearVelocity - trueBaseLinearVelocity).norm() < eps);
  EXPECT_TRUE((baseWeights.angularVelocity - trueBaseAngularVelocity).norm() < eps);

  const auto jointWeights = loadJointWeights(filePath, modelInfo, model);

  vector_t trueJointPositions = vector_t::Zero(modelInfo.actuatedDofNum);
  trueJointPositions << 0.1, 0.3, 0.5, 0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9, 2.1, 2.3;
  vector_t trueJointVelocities = vector_t::Zero(modelInfo.actuatedDofNum);
  trueJointVelocities << 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4;

  EXPECT_TRUE((jointWeights.positions - trueJointPositions).norm() < eps);
  EXPECT_TRUE((jointWeights.velocities - trueJointVelocities).norm() < eps);

  const auto endEffectorWeights = loadEndEffectorWeights(filePath, modelSettings);

  const std::vector<vector3_t> trueEndEffectorPosition{{0.1, 0.2, 0.3}, {1.0, 1.1, 1.2}, 
    {1.9, 2.0, 2.1}, {2.8, 2.9, 3.0}};

  const std::vector<vector3_t> trueEndEffectorVelocity{{0.4, 0.5, 0.6}, {1.3, 1.4, 1.5}, 
    {2.2, 2.3, 2.4}, {3.1, 3.2, 3.3}};

  const std::vector<vector3_t> trueEndEffectorForce{{0.7, 0.8, 0.9}, {1.6, 1.7, 1.8}, 
    {2.5, 2.6, 2.7}, {3.4, 3.5, 3.6}};

  const size_t endEffectorNum = modelSettings.contactNames3DoF.size() + modelSettings.contactNames6DoF.size();

  for(size_t i = 0; i < endEffectorNum; ++i)
  {
    EXPECT_TRUE((endEffectorWeights.positions[i] - trueEndEffectorPosition[i]).norm() < eps);
    EXPECT_TRUE((endEffectorWeights.velocities[i] - trueEndEffectorVelocity[i]).norm() < eps);
    EXPECT_TRUE((endEffectorWeights.forces[i] - trueEndEffectorForce[i]).norm() < eps);
  }
}