#include <gtest/gtest.h>

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/common/Utils.hpp>

#include <floating_base_model/FactoryFunctions.hpp>
#include <floating_base_model/AccessHelperFunctions.hpp>

#include "../include/definitions.hpp"

using namespace ocs2;
using namespace floating_base_model;
using namespace legged_locomotion_mpc;
using namespace floating_base_model::access_helper_functions;

const scalar_t tolerance = 1e-9;

TEST(UtilsTest, robotStateToOptimizationStateAndInput)
{
  std::string urdfPathName = meldogWithBaseLinkUrdfFile;

  std::vector<std::string> testThreeDofMeldogContacts(2);
  testThreeDofMeldogContacts[0] = meldog3DofContactNames[0];
  testThreeDofMeldogContacts[1] = meldog3DofContactNames[1];

  std::vector<std::string> testSixDofMeldogContacts(2);
  testSixDofMeldogContacts[0] = meldog3DofContactNames[2];
  testSixDofMeldogContacts[1] = meldog3DofContactNames[3];

  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, 
    baseLink);

  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, 
    testThreeDofMeldogContacts, testSixDofMeldogContacts);

  state_vector_t stateVector = getAccessTestRobotStateForOptimalState();

  const auto optimalStateVectorTrue = getAccessTestRobotOptimalState();
  const auto optimalInputVectorTrue = getAccessTestRobotOptimalInput();

  const auto [optimalStateVector, optimalInputVector] = 
    utils::robotStateToOptimizationStateAndInput(modelInfo, stateVector);

  EXPECT_TRUE(optimalStateVector.rows() ==  optimalStateVectorTrue.rows());
  EXPECT_TRUE((optimalStateVector - optimalStateVectorTrue).norm() < tolerance);

  EXPECT_TRUE(optimalInputVector.rows() ==  optimalInputVectorTrue.rows());
  EXPECT_TRUE((optimalInputVector.block(
    3 * NUM_THREE_DOF_CONTACTS + NUM_SIX_DOF_CONTACTS * 6, 0, ACTUATED_DOF_NUM, 1) - 
    optimalInputVectorTrue.block(3 * NUM_THREE_DOF_CONTACTS + NUM_SIX_DOF_CONTACTS * 6, 
    0, ACTUATED_DOF_NUM, 1)).norm() < tolerance);
  EXPECT_TRUE((optimalInputVector.block(0, 0, 
    3 * NUM_THREE_DOF_CONTACTS + NUM_SIX_DOF_CONTACTS * 6, 1) - vector_t::Zero( 
    3 * NUM_THREE_DOF_CONTACTS + NUM_SIX_DOF_CONTACTS * 6)).norm() < tolerance);
}

TEST(UtilsTest, subsampleReferenceTrajectory)
{
  TargetTrajectories initTrajectory;
  const scalar_t initTime = 0.0;
  const scalar_t finalTime = 10.0;
  const size_t initSize = 5;
  initTrajectory.timeTrajectory.resize(initSize);
  vector_t initVector = vector_t::Zero(2);
  initTrajectory.stateTrajectory.resize(initSize, initVector);
  initTrajectory.inputTrajectory.resize(initSize, initVector);

  // time
  initTrajectory.timeTrajectory[0] = initTime;
  initTrajectory.timeTrajectory[1] = initTime + 1; 
  initTrajectory.timeTrajectory[2] = initTime + 5; 
  initTrajectory.timeTrajectory[3] = initTime + 8; 
  initTrajectory.timeTrajectory[4] = finalTime; 

  // state
  initTrajectory.stateTrajectory[1] << 2.0, 5.0;
  initTrajectory.stateTrajectory[2] << 10.0, 15.0;
  initTrajectory.stateTrajectory[3] << 4.0, 3.0;
  initTrajectory.stateTrajectory[4] << 0.0, 0.0;

  // input
  initTrajectory.inputTrajectory[0] << 10.0, 7.0;
  initTrajectory.inputTrajectory[1] << 10.0, 15.0;
  initTrajectory.inputTrajectory[2] << 10.0, 15.0;
  initTrajectory.inputTrajectory[3] << 10.0, 0.0;
  initTrajectory.inputTrajectory[4] << 10.0, 0.0;
  
  const scalar_t newInitTime = 2.0;
  const scalar_t newFinalTime = 10.0;
  const scalar_t sampleTime = 2.0;
  TargetTrajectories targetTrajectory;
  targetTrajectory = 
    utils::subsampleReferenceTrajectory(initTrajectory, newInitTime, newFinalTime, 
      sampleTime);
  
  std::vector<scalar_t> trueTime{2.0, 4.0, 5.0, 7.0, 8.0, 10.0};

  std::vector<vector_t> trueState;
  trueState.resize(trueTime.size(), initVector);
  trueState[0] << 4.0, 7.5;
  trueState[1] << 8.0, 12.5;
  trueState[2] << 10.0, 15.0;
  trueState[3] << 6.0, 7.0;
  trueState[4] << 4.0, 3.0;
  trueState[5] << 0.0, 0.0;

  std::vector<vector_t> trueInput;
  trueInput.resize(trueTime.size(), initVector);
  trueInput[0] << 10.0, 15.0;
  trueInput[1] << 10.0, 15.0;
  trueInput[2] << 10.0, 15.0;
  trueInput[3] << 10.0, 5.0;
  trueInput[4] << 10.0, 0.0;
  trueInput[5] << 10.0, 0.0;
  
  for(size_t i = 0; i < trueTime.size(); ++i)
  {
    EXPECT_TRUE(std::abs(targetTrajectory.timeTrajectory[i] - trueTime[i]) < tolerance);
    EXPECT_TRUE((targetTrajectory.stateTrajectory[i] - trueState[i]).norm() < tolerance);
    EXPECT_TRUE((targetTrajectory.inputTrajectory[i] - trueInput[i]).norm() < tolerance);
  }
}

TEST(UtilsTest, numberOfClosedContacts)
{
  std::string urdfPathName = meldogWithBaseLinkUrdfFile;

  std::vector<std::string> testThreeDofMeldogContacts(2);
  testThreeDofMeldogContacts[0] = meldog3DofContactNames[0];
  testThreeDofMeldogContacts[1] = meldog3DofContactNames[1];

  std::vector<std::string> testSixDofMeldogContacts(2);
  testSixDofMeldogContacts[0] = meldog3DofContactNames[2];
  testSixDofMeldogContacts[1] = meldog3DofContactNames[3];

  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, 
    baseLink);

  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, 
    testThreeDofMeldogContacts, testSixDofMeldogContacts);

  contact_flags_t zero = 0;

  contact_flags_t one;
  one[0] = 1;

  contact_flags_t two;
  two[0] = 1;
  two[1] = 1;

  contact_flags_t three;
  three[0] = 1;
  three[1] = 1;
  three[2] = 1;

  contact_flags_t four;
  four[0] = 1;
  four[1] = 1;
  four[2] = 1;
  four[3] = 1;

  contact_flags_t twoAgain;
  twoAgain[0] = 1;
  twoAgain[1] = 0;
  twoAgain[2] = 1;
  twoAgain[3] = 0;

  contact_flags_t threeAgain;
  threeAgain[0] = 1;
  threeAgain[1] = 0;
  threeAgain[2] = 1;
  threeAgain[3] = 1;

  contact_flags_t five;
  five[0] = 1;
  five[1] = 1;
  five[2] = 1;
  five[3] = 1;
  five[4] = 1;
  
  EXPECT_TRUE(utils::numberOfClosedContacts(modelInfo, zero)       == 0);
  EXPECT_TRUE(utils::numberOfClosedContacts(modelInfo, one)        == 1);
  EXPECT_TRUE(utils::numberOfClosedContacts(modelInfo, two)        == 2);
  EXPECT_TRUE(utils::numberOfClosedContacts(modelInfo, three)      == 3);
  EXPECT_TRUE(utils::numberOfClosedContacts(modelInfo, four)       == 4);
  EXPECT_TRUE(utils::numberOfClosedContacts(modelInfo, twoAgain)   == 2);
  EXPECT_TRUE(utils::numberOfClosedContacts(modelInfo, threeAgain) == 3);
  EXPECT_TRUE(utils::numberOfClosedContacts(modelInfo, five)       == 4);
}

TEST(UtilsTest, weightCompensatingInput)
{
  std::string urdfPathName = meldogWithBaseLinkUrdfFile;

  std::vector<std::string> testThreeDofMeldogContacts(2);
  testThreeDofMeldogContacts[0] = meldog3DofContactNames[0];
  testThreeDofMeldogContacts[1] = meldog3DofContactNames[1];

  std::vector<std::string> testSixDofMeldogContacts(2);
  testSixDofMeldogContacts[0] = meldog3DofContactNames[2];
  testSixDofMeldogContacts[1] = meldog3DofContactNames[3];

  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, 
    baseLink);

  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, 
    testThreeDofMeldogContacts, testSixDofMeldogContacts);

  contact_flags_t zero = 0;

  contact_flags_t one;
  one[0] = 1;

  contact_flags_t two;
  two[0] = 1;
  two[2] = 1;

  contact_flags_t three;
  three[0] = 1;
  three[2] = 1;
  three[3] = 1;

  contact_flags_t four;
  four[0] = 1;
  four[1] = 1;
  four[2] = 1;
  four[3] = 1;

  const vector3_t forceZero(0.0, 0.0, 0.0);
  const vector3_t forceOne(0.0, 0.0, modelInfo.robotMass / 1.0 * PLUS_GRAVITY_VALUE);
  const vector3_t forceTwo(0.0, 0.0, modelInfo.robotMass / 2.0 * PLUS_GRAVITY_VALUE);
  const vector3_t forceThree(0.0, 0.0, modelInfo.robotMass / 3.0 * PLUS_GRAVITY_VALUE);
  const vector3_t forceFour(0.0, 0.0, modelInfo.robotMass / 4.0 * PLUS_GRAVITY_VALUE);

  const vector_t zeroInput = utils::weightCompensatingInput(modelInfo, zero);
  const vector_t oneInput = utils::weightCompensatingInput(modelInfo, one);
  const vector_t twoInput = utils::weightCompensatingInput(modelInfo, two);
  const vector_t threeInput = utils::weightCompensatingInput(modelInfo, three);
  const vector_t fourInput = utils::weightCompensatingInput(modelInfo, four);

  EXPECT_TRUE(zeroInput.norm() < tolerance);

  EXPECT_TRUE((getContactForces(oneInput, 0, modelInfo) - forceOne).norm() < tolerance);
  EXPECT_TRUE((getContactForces(oneInput, 1, modelInfo)).norm() < tolerance);
  EXPECT_TRUE((getContactForces(oneInput, 2, modelInfo)).norm() < tolerance);
  EXPECT_TRUE((getContactForces(oneInput, 3, modelInfo)).norm() < tolerance);

  EXPECT_TRUE((getContactForces(twoInput, 0, modelInfo) - forceTwo).norm() < tolerance);
  EXPECT_TRUE((getContactForces(twoInput, 1, modelInfo)).norm() < tolerance);
  EXPECT_TRUE((getContactForces(twoInput, 2, modelInfo) - forceTwo).norm() < tolerance);
  EXPECT_TRUE((getContactForces(twoInput, 3, modelInfo)).norm() < tolerance);

  EXPECT_TRUE((getContactForces(threeInput, 0, modelInfo) - forceThree).norm() < tolerance);
  EXPECT_TRUE((getContactForces(threeInput, 1, modelInfo)).norm() < tolerance);
  EXPECT_TRUE((getContactForces(threeInput, 2, modelInfo) - forceThree).norm() < tolerance);
  EXPECT_TRUE((getContactForces(threeInput, 3, modelInfo) - forceThree).norm() < tolerance);

  EXPECT_TRUE((getContactForces(fourInput, 0, modelInfo) - forceFour).norm() < tolerance);
  EXPECT_TRUE((getContactForces(fourInput, 1, modelInfo) - forceFour).norm() < tolerance);
  EXPECT_TRUE((getContactForces(fourInput, 2, modelInfo) - forceFour).norm() < tolerance);
  EXPECT_TRUE((getContactForces(fourInput, 3, modelInfo) - forceFour).norm() < tolerance);

}

TEST(UtilsTest, weightCompensatingAppendInput)
{
    std::string urdfPathName = meldogWithBaseLinkUrdfFile;

  std::vector<std::string> testThreeDofMeldogContacts(2);
  testThreeDofMeldogContacts[0] = meldog3DofContactNames[0];
  testThreeDofMeldogContacts[1] = meldog3DofContactNames[1];

  std::vector<std::string> testSixDofMeldogContacts(2);
  testSixDofMeldogContacts[0] = meldog3DofContactNames[2];
  testSixDofMeldogContacts[1] = meldog3DofContactNames[3];

  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, 
    baseLink);

  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, 
    testThreeDofMeldogContacts, testSixDofMeldogContacts);

  contact_flags_t zero = 0;

  contact_flags_t one;
  one[0] = 1;

  contact_flags_t two;
  two[0] = 1;
  two[2] = 1;

  contact_flags_t three;
  three[0] = 1;
  three[2] = 1;
  three[3] = 1;

  contact_flags_t four;
  four[0] = 1;
  four[1] = 1;
  four[2] = 1;
  four[3] = 1;

  const vector3_t forceZero(0.0, 0.0, 0.0);
  const vector3_t forceOne(0.0, 0.0, modelInfo.robotMass / 1.0 * PLUS_GRAVITY_VALUE);
  const vector3_t forceTwo(0.0, 0.0, modelInfo.robotMass / 2.0 * PLUS_GRAVITY_VALUE);
  const vector3_t forceThree(0.0, 0.0, modelInfo.robotMass / 3.0 * PLUS_GRAVITY_VALUE);
  const vector3_t forceFour(0.0, 0.0, modelInfo.robotMass / 4.0 * PLUS_GRAVITY_VALUE);

  vector_t zeroInput  = vector_t::Zero(modelInfo.inputDim);
  vector_t oneInput   = vector_t::Zero(modelInfo.inputDim);
  vector_t twoInput   = vector_t::Zero(modelInfo.inputDim);
  vector_t threeInput = vector_t::Zero(modelInfo.inputDim);
  vector_t fourInput  = vector_t::Zero(modelInfo.inputDim);

  utils::weightCompensatingAppendInput(zeroInput, modelInfo, zero);
  utils::weightCompensatingAppendInput(oneInput, modelInfo, one);
  utils::weightCompensatingAppendInput(twoInput, modelInfo, two);
  utils::weightCompensatingAppendInput(threeInput, modelInfo, three);
  utils::weightCompensatingAppendInput(fourInput, modelInfo, four);

  EXPECT_TRUE(zeroInput.norm() < tolerance);

  EXPECT_TRUE((getContactForces(oneInput, 0, modelInfo) - forceOne).norm() < tolerance);
  EXPECT_TRUE((getContactForces(oneInput, 1, modelInfo)).norm() < tolerance);
  EXPECT_TRUE((getContactForces(oneInput, 2, modelInfo)).norm() < tolerance);
  EXPECT_TRUE((getContactForces(oneInput, 3, modelInfo)).norm() < tolerance);

  EXPECT_TRUE((getContactForces(twoInput, 0, modelInfo) - forceTwo).norm() < tolerance);
  EXPECT_TRUE((getContactForces(twoInput, 1, modelInfo)).norm() < tolerance);
  EXPECT_TRUE((getContactForces(twoInput, 2, modelInfo) - forceTwo).norm() < tolerance);
  EXPECT_TRUE((getContactForces(twoInput, 3, modelInfo)).norm() < tolerance);

  EXPECT_TRUE((getContactForces(threeInput, 0, modelInfo) - forceThree).norm() < tolerance);
  EXPECT_TRUE((getContactForces(threeInput, 1, modelInfo)).norm() < tolerance);
  EXPECT_TRUE((getContactForces(threeInput, 2, modelInfo) - forceThree).norm() < tolerance);
  EXPECT_TRUE((getContactForces(threeInput, 3, modelInfo) - forceThree).norm() < tolerance);

  EXPECT_TRUE((getContactForces(fourInput, 0, modelInfo) - forceFour).norm() < tolerance);
  EXPECT_TRUE((getContactForces(fourInput, 1, modelInfo) - forceFour).norm() < tolerance);
  EXPECT_TRUE((getContactForces(fourInput, 2, modelInfo) - forceFour).norm() < tolerance);
  EXPECT_TRUE((getContactForces(fourInput, 3, modelInfo) - forceFour).norm() < tolerance);

}
