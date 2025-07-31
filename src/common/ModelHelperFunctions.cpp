#include <legged_locomotion_mpc/common/ModelHelperFunctions.hpp>

namespace floating_base_model
{
  namespace model_helper_function
  {

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename SCALAR_T>
    Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> computeApproxActuatedJointGeneralizedTorques(
      ocs2::PinocchioInterfaceTpl<SCALAR_T>& interface,
      const floating_base_model::FloatingBaseModelInfoTpl<SCALAR_T>& info,
      const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q,
      const pinocchio::container::aligned_vector<pinocchio::ForceTpl<SCALAR_T, 0>>& fext)
    {
      const auto& model = interface.getModel();
      auto& data = interface.getData();
      Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> v = Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>::Zero(model.nv);
      Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> a = Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>::Zero(model.nv);
      return (pinocchio::rnea(model, data, q, v, a, fext) - pinocchio::rnea(model, data, q, v, a, fext)).block(6, 0, info.actuatedDofNum, 1);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    // Explicit template instantiation

    template Eigen::Matrix<ocs2::scalar_t, Eigen::Dynamic, 1> computeApproxActuatedJointGeneralizedTorques(
      ocs2::PinocchioInterfaceTpl<ocs2::scalar_t>& interface,
      const floating_base_model::FloatingBaseModelInfoTpl<ocs2::scalar_t>& info,
      const Eigen::Matrix<ocs2::scalar_t, Eigen::Dynamic, 1>& q,
      const pinocchio::container::aligned_vector<pinocchio::ForceTpl<ocs2::scalar_t, 0>>& fext);

    template Eigen::Matrix<ocs2::ad_scalar_t, Eigen::Dynamic, 1> computeApproxActuatedJointGeneralizedTorques(
      ocs2::PinocchioInterfaceTpl<ocs2::ad_scalar_t>& interface,
      const floating_base_model::FloatingBaseModelInfoTpl<ocs2::ad_scalar_t>& info,
      const Eigen::Matrix<ocs2::ad_scalar_t, Eigen::Dynamic, 1>& q,
      const pinocchio::container::aligned_vector<pinocchio::ForceTpl<ocs2::ad_scalar_t, 0>>& fext);
  }
}