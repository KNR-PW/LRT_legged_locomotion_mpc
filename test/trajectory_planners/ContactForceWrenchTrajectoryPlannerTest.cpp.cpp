#include <gtest/gtest.h>

#include <legged_locomotion_mpc/trajectory_planners/ContactForceWrenchTrajectoryPlanner.hpp>

#include <floating_base_model/AccessHelperFunctions.hpp>
#include <floating_base_model/FloatingBaseModelPinocchioMapping.hpp>
#include <floating_base_model/FactoryFunctions.hpp>

#include <legged_locomotion_mpc/locomotion/GaitCommon.hpp>

#include "../include/definitions.hpp"

using namespace ocs2;
using namespace floating_base_model;
using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::planners;
using namespace legged_locomotion_mpc::locomotion;

const scalar_t tolerance = 1e-9;

TEST(ContactForceWrenchTrajectoryPlanner, updateTrajectory) 
{
  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(
    meldogWithBaseLinkUrdfFile, baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, 
    meldog3DofContactNames, meldog6DofContactNames);

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

  ModeSchedule modeSchedule;
  modeSchedule.clear();
  modeSchedule.eventTimes = {0.0, 0.2, 0.5, 0.7, 1.0, 1.5, 2.0};

  modeSchedule.modeSequence.push_back(contactFlags2ModeNumber(zero));
  modeSchedule.modeSequence.push_back(contactFlags2ModeNumber(one));
  modeSchedule.modeSequence.push_back(contactFlags2ModeNumber(two));
  modeSchedule.modeSequence.push_back(contactFlags2ModeNumber(three));
  modeSchedule.modeSequence.push_back(contactFlags2ModeNumber(four));
  modeSchedule.modeSequence.push_back(contactFlags2ModeNumber(five));
  modeSchedule.modeSequence.push_back(contactFlags2ModeNumber(twoAgain));
  modeSchedule.modeSequence.push_back(contactFlags2ModeNumber(threeAgain));
  
  TargetTrajectories targetTrajectories;

  targetTrajectories.timeTrajectory = modeSchedule.eventTimes;
  targetTrajectories.timeTrajectory.push_back(2.5);

  const size_t referenceSize = targetTrajectories.timeTrajectory.size();

  targetTrajectories.stateTrajectory.resize(referenceSize, 
    vector_t::Zero(modelInfo.stateDim));

  targetTrajectories.inputTrajectory.resize(referenceSize, 
    vector_t::Zero(modelInfo.inputDim));

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
  const vector_t fiveInput = utils::weightCompensatingInput(modelInfo, five);
  const vector_t twoAgainInput = utils::weightCompensatingInput(modelInfo, twoAgain);
  const vector_t threeAgainInput = utils::weightCompensatingInput(modelInfo, threeAgain);

  ContactForceWrenchTrajectoryPlanner planner(modelInfo);
  planner.updateTargetTrajectory(modeSchedule, targetTrajectories);

  EXPECT_TRUE((targetTrajectories.inputTrajectory[0] - zeroInput).norm() < tolerance);
  EXPECT_TRUE((targetTrajectories.inputTrajectory[1] - oneInput).norm() < tolerance);
  EXPECT_TRUE((targetTrajectories.inputTrajectory[2] - twoInput).norm() < tolerance);
  EXPECT_TRUE((targetTrajectories.inputTrajectory[3] - threeInput).norm() < tolerance);
  EXPECT_TRUE((targetTrajectories.inputTrajectory[4] - fourInput).norm() < tolerance);
  EXPECT_TRUE((targetTrajectories.inputTrajectory[5] - fiveInput).norm() < tolerance);
  EXPECT_TRUE((targetTrajectories.inputTrajectory[6] - twoAgainInput).norm() < tolerance);
  EXPECT_TRUE((targetTrajectories.inputTrajectory[7] - threeAgainInput).norm() < tolerance);
}