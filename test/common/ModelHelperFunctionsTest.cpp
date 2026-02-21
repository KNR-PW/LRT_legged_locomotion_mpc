#include <gtest/gtest.h>

#include <legged_locomotion_mpc/common/Types.hpp>

#include <legged_locomotion_mpc/common/ModelHelperFunctions.hpp>

#include "../include/definitions.hpp"

#include <floating_base_model/FactoryFunctions.hpp>
#include <floating_base_model/ModelHelperFunctions.hpp>

using namespace ocs2;
using namespace floating_base_model;
using namespace legged_locomotion_mpc;
using namespace floating_base_model::access_helper_functions;

const scalar_t tolerance = 1e-9;
const size_t NUM_TEST = 100;

TEST(ModelHelperFunctions, GeneralizedTorques)
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
  
  // Interface model and data (references)
  const pinocchio::Model& model = interface.getModel();
  pinocchio::Data& data = interface.getData();

  // New model and data (copies)
  const pinocchio::Model modelTrue = interface.getModel();
  pinocchio::Data dataTrue = interface.getData();

  for(size_t i = 0; i < NUM_TEST; ++i)
  {
    ocs2::vector_t q = ocs2::vector_t::Random(model.nq);
    Eigen::Quaternion quaterion(q[6], q[3], q[4], q[5]);
    quaterion.normalize();
    q.block<4,1>(3, 0) = quaterion.coeffs();

    pinocchio::forwardKinematics(model, data, q);
    pinocchio::forwardKinematics(modelTrue, dataTrue, q);

    pinocchio::Force zero_force(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    pinocchio::container::aligned_vector<pinocchio::Force> fext(model.njoints, zero_force);

    const vector_t input = vector_t::Random(info.inputDim);

    floating_base_model::model_helper_functions::computeSpatialForces(interface, info, 
      input, fext);

    const auto tauStatic = 
      legged_locomotion_mpc::model_helper_functions::
      computeApproxActuatedJointGeneralizedTorques(interface, 
        info, q, fext);

    pinocchio::updateFramePlacements(modelTrue, dataTrue);
    pinocchio::computeJointJacobians(modelTrue, dataTrue);

    Eigen::Vector<scalar_t, Eigen::Dynamic> tauStaticTrue = Eigen::Vector<scalar_t, Eigen::Dynamic>::Zero(model.nv);

    for (size_t i = 0; i < info.numThreeDofContacts; i++) 
    {
      const auto forceWorldFrame = getContactForces(input, i, info);
      size_t contactFrameIndex = info.endEffectorFrameIndices[i];
      pinocchio::Data::Matrix6x J(6, model.nv);
      J.setZero();
      pinocchio::computeFrameJacobian(modelTrue, dataTrue, q, contactFrameIndex, pinocchio::LOCAL_WORLD_ALIGNED, J);
      tauStaticTrue += -J.transpose().block(0, 0, model.nv, 3) * forceWorldFrame;
    }

    for (size_t i = info.numThreeDofContacts; i < info.numThreeDofContacts + info.numSixDofContacts; i++) 
    {
      const auto wrenchWorldFrame = getContactWrenches(input, i, info);
      size_t contactFrameIndex = info.endEffectorFrameIndices[i];
      pinocchio::Data::Matrix6x J(6, model.nv);
      J.setZero();
      pinocchio::computeFrameJacobian(modelTrue, dataTrue, q, contactFrameIndex, pinocchio::LOCAL_WORLD_ALIGNED, J);
      tauStaticTrue += -J.transpose() * wrenchWorldFrame;
    }
    EXPECT_TRUE((tauStatic - tauStaticTrue.block(6, 0, info.actuatedDofNum, 1)).norm() < tolerance);
  }
}