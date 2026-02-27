#include <gtest/gtest.h>

#include <legged_locomotion_mpc/common/ModelSettings.hpp>
#include <legged_locomotion_mpc/cost/JointTorqueCost.hpp>

#include <floating_base_model/FactoryFunctions.hpp>

#include "../test/include/definitions.hpp"

using namespace ocs2;
using namespace floating_base_model;
using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::cost;

const static scalar_t eps = 1e-9;

TEST(JointTorqueCostTest, loaders) 
{

  ModelSettings modelSettings;
  modelSettings.baseLinkName =  baseLink;
  modelSettings.endEffectorThreeDofNames = meldog3DofContactNames;
  modelSettings.endEffectorSixDofNames = meldog6DofContactNames;
  modelSettings.hipFrameNames = meldogHipNames;

  std::string urdfPathName = meldogWithBaseLinkUrdfFile;

  PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);

  const auto& model = interface.getModel();

  const std::string filePath = meldogConfigFolder + "joint_torque_cost_settings.info";

  const auto jointWeights = loadJointTorqueWeights(filePath, modelInfo, model);

  vector_t trueJointPositions = vector_t::Zero(modelInfo.actuatedDofNum);
  trueJointPositions << 0.1, 0.3, 0.5, 0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9, 2.1, 2.3;
  
  EXPECT_TRUE((jointWeights.weights - trueJointPositions).norm() < eps);

}