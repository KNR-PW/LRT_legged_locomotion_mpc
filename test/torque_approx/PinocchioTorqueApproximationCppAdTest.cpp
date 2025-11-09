#include <gtest/gtest.h>

#include <legged_locomotion_mpc/torque_approx/PinocchioTorqueApproximationCppAd.hpp>

#include <floating_base_model/FloatingBaseModelPinocchioMapping.hpp>
#include <floating_base_model/FactoryFunctions.hpp>

#include <pinocchio/algorithm/frames.hpp>

#include "../include/definitions.hpp"

using namespace ocs2;
using namespace floating_base_model;
using namespace legged_locomotion_mpc;
using namespace floating_base_model::access_helper_functions;

const scalar_t tolerance = 1e-9;
const size_t NUM_TEST = 100;

TEST(PinocchioTorqueApproximationCppAd, getValue)
{
  std::vector<std::string> testThreeDofMeldogContacts(2);
  testThreeDofMeldogContacts[0] = meldog3DofContactNames[0];
  testThreeDofMeldogContacts[1] = meldog3DofContactNames[1];

  std::vector<std::string> testSixDofMeldogContacts(2);
  testSixDofMeldogContacts[0] = meldog3DofContactNames[2];
  testSixDofMeldogContacts[1] = meldog3DofContactNames[3];

  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(meldogWithBaseLinkUrdfFile, baseLink);
  auto info = createFloatingBaseModelInfo(interface, testThreeDofMeldogContacts, 
    testSixDofMeldogContacts);

  // mapping
  FloatingBaseModelPinocchioMapping mapping(info);
  mapping.setPinocchioInterface(interface);

  // New modelTrue and data (copies)
  const pinocchio::Model modelTrue = interface.getModel();
  pinocchio::Data dataTrue = interface.getData();

  const std::string modelName = "torque_test";
  PinocchioTorqueApproximationCppAd torqueApproximator(interface, info,
    vector_t::Zero(info.actuatedDofNum), modelName);

  for(size_t i = 0; i < NUM_TEST; ++i)
  {
    const vector_t state = vector_t::Random(info.stateDim);
    const vector_t input = vector_t::Random(info.inputDim);
    
    const vector_t q = mapping.getPinocchioJointPosition(state);
   
    pinocchio::forwardKinematics(modelTrue, dataTrue, q);

    pinocchio::updateFramePlacements(modelTrue, dataTrue);
    pinocchio::computeJointJacobians(modelTrue, dataTrue);

    const vector_t tauStatic = torqueApproximator.getValue(state, input);

    Eigen::Vector<scalar_t, Eigen::Dynamic> tauStaticTrue = 
      Eigen::Vector<scalar_t, Eigen::Dynamic>::Zero(modelTrue.nv);

    for (size_t i = 0; i < info.numThreeDofContacts; i++) 
    {
      const auto forceWorldFrame = getContactForces(input, i, info);
      size_t contactFrameIndex = info.endEffectorFrameIndices[i];
      pinocchio::Data::Matrix6x J(6, modelTrue.nv);
      J.setZero();
      pinocchio::computeFrameJacobian(modelTrue, dataTrue, q, contactFrameIndex, pinocchio::LOCAL_WORLD_ALIGNED, J);
      tauStaticTrue += -J.transpose().block(0, 0, modelTrue.nv, 3) * forceWorldFrame;
    }

    for (size_t i = info.numThreeDofContacts; i < info.numThreeDofContacts + info.numSixDofContacts; i++) 
    {
      const auto wrenchWorldFrame = getContactWrenches(input, i, info);
      size_t contactFrameIndex = info.endEffectorFrameIndices[i];
      pinocchio::Data::Matrix6x J(6, modelTrue.nv);
      J.setZero();
      pinocchio::computeFrameJacobian(modelTrue, dataTrue, q, contactFrameIndex, pinocchio::LOCAL_WORLD_ALIGNED, J);
      tauStaticTrue += -J.transpose() * wrenchWorldFrame;
    }
    EXPECT_TRUE((tauStatic - tauStaticTrue.block(6, 0, info.actuatedDofNum, 1)).norm() < tolerance);
  }
}