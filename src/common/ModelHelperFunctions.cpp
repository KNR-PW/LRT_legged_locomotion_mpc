#include <legged_locomotion_mpc/common/ModelHelperFunctions.hpp>

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
  } // model_helper_functions
} // legged_locomotion_mpc