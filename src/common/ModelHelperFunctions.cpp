#include <legged_locomotion_mpc/common/ModelHelperFunctions.hpp>

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>

namespace legged_locomotion_mpc
{
  namespace model_helper_functions
  {
    using namespace ocs2;
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename SCALAR_T>
    Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> computeApproxActuatedJointGeneralizedTorques(
      PinocchioInterfaceTpl<SCALAR_T>& interface,
      const floating_base_model::FloatingBaseModelInfoTpl<SCALAR_T>& info,
      const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q,
      const pinocchio::container::aligned_vector<pinocchio::ForceTpl<SCALAR_T, 0>>& fext)
    {
      const auto& model = interface.getModel();
      auto& data = interface.getData();
      return (pinocchio::computeStaticTorque(model, data, q, fext) - pinocchio::computeGeneralizedGravity(model, data, q)).block(6, 0, info.actuatedDofNum, 1);
    }
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename SCALAR_T>
    std::vector<Eigen::Matrix<SCALAR_T, 6, 1>> computeContactWrenches(
      ocs2::PinocchioInterfaceTpl<SCALAR_T>& interface,
      const floating_base_model::FloatingBaseModelInfoTpl<SCALAR_T>& info,
      const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q, 
      const contact_flags_t &contactFlags)
    {
      const auto& model = interface.getModel();
      auto& data = interface.getData();
      pinocchio::framesForwardKinematics(model, data, q);
      pinocchio::centerOfMass(model, data, q, false);

      const Eigen::Matrix<SCALAR_T, 3, 1> comPosition = data.com[0];

      const Eigen::Matrix<SCALAR_T, 3, 1> gravityForce(SCALAR_T(0), SCALAR_T(0), 
        info.robotMass * PLUS_GRAVITY_VALUE);

      const Eigen::Matrix<SCALAR_T, 3, 1> gravityTorque = comPosition.cross(gravityForce);

      const size_t numStanceLegs = contactFlags.count();

      const auto sixDofContacts = contactFlags >> info.numThreeDofContacts;

      const size_t numSixDoFStanceLegs = contactFlags.count();

      size_t numEndEffectors = info.numThreeDofContacts + info.numSixDofContacts;

      const Eigen::Matrix<SCALAR_T, 3, 1> forceInInertialFrame = gravityForce / SCALAR_T(numStanceLegs);

      Eigen::Matrix<SCALAR_T, 3, 1> completeTorque = gravityTorque;

      for(size_t i = 0; i < numEndEffectors; ++i)
      {
        if(!contactFlags[i]) continue;
        const size_t endEffectorIndex = info.endEffectorFrameIndices[i];
        const Eigen::Matrix<SCALAR_T, 3, 1>& endEffectorPosition = data.oMf[endEffectorIndex].translation();
        const Eigen::Matrix<SCALAR_T, 3, 1> endEffectorForceTorque = endEffectorPosition.cross(
          forceInInertialFrame);

        completeTorque -= endEffectorForceTorque;
      }

      const Eigen::Matrix<SCALAR_T, 3, 1> torqueInertialFrame = completeTorque / SCALAR_T(numSixDoFStanceLegs);

      std::vector<Eigen::Matrix<SCALAR_T, 6, 1>> wrenches;
      wrenches.reserve(numEndEffectors);

      for(size_t i = 0; i < numEndEffectors; ++i)
      {
        if(contactFlags[i])
        {
          Eigen::Matrix<SCALAR_T, 6, 1> wrench;
          const Eigen::Matrix<SCALAR_T, 3, 1> torque = i >= info.numThreeDofContacts ? torqueInertialFrame : Eigen::Matrix<SCALAR_T, 3, 1>::Zero();
          wrench << forceInInertialFrame, torque;
          wrenches.push_back(std::move(wrench));
        }
        else
        {
          wrenches.emplace_back(Eigen::Matrix<SCALAR_T, 6, 1>::Zero());
        }
      }
      return wrenches;
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    // Explicit template instantiation

    template Eigen::Matrix<scalar_t, Eigen::Dynamic, 1> computeApproxActuatedJointGeneralizedTorques(
      PinocchioInterfaceTpl<scalar_t>& interface,
      const floating_base_model::FloatingBaseModelInfoTpl<scalar_t>& info,
      const Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>& q,
      const pinocchio::container::aligned_vector<pinocchio::ForceTpl<scalar_t, 0>>& fext);

    template Eigen::Matrix<ad_scalar_t, Eigen::Dynamic, 1> computeApproxActuatedJointGeneralizedTorques(
      PinocchioInterfaceTpl<ad_scalar_t>& interface,
      const floating_base_model::FloatingBaseModelInfoTpl<ad_scalar_t>& info,
      const Eigen::Matrix<ad_scalar_t, Eigen::Dynamic, 1>& q,
      const pinocchio::container::aligned_vector<pinocchio::ForceTpl<ad_scalar_t, 0>>& fext);

    template std::vector<Eigen::Matrix<ocs2::scalar_t, 6, 1>> computeContactWrenches(
      ocs2::PinocchioInterfaceTpl<ocs2::scalar_t>& interface,
      const floating_base_model::FloatingBaseModelInfoTpl<ocs2::scalar_t>& info,
      const Eigen::Matrix<ocs2::scalar_t, Eigen::Dynamic, 1>& q, 
      const contact_flags_t &contactFlags);

    template std::vector<Eigen::Matrix<ocs2::ad_scalar_t, 6, 1>> computeContactWrenches(
      ocs2::PinocchioInterfaceTpl<ocs2::ad_scalar_t>& interface,
      const floating_base_model::FloatingBaseModelInfoTpl<ocs2::ad_scalar_t>& info,
      const Eigen::Matrix<ocs2::ad_scalar_t, Eigen::Dynamic, 1>& q, 
      const contact_flags_t &contactFlags);
  } // model_helper_functions
} // legged_locomotion_mpc