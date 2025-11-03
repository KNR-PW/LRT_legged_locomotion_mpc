// Copyright (c) 2025, Koło Naukowe Robotyków
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

/*
 * Authors: Bartłomiej Krajewski (https://github.com/BartlomiejK2)
 */

#ifndef __MODEL_HELPER_FUNCTIONS_LEGGED_LOCOMOTION_MPC__
#define __MODEL_HELPER_FUNCTIONS_LEGGED_LOCOMOTION_MPC__

#include <Eigen/Dense>

#include <pinocchio/container/aligned-vector.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>


namespace legged_locomotion_mpc
{
  namespace model_helper_functions
  {
    /**
     * Compute approximated actuated joint torques due to:
     * - External Forces: -J^t * f_ext
     *
     * @param [in] interface: pinocchio interface
     * @param [in] info: floating base model info
     * @param [in] q: pinocchio joint configuration
     * @param [in] fext: external forces expressed in the local frame of the joints (dim model.njoints)
     * @return tau: generalized torques of actuated joints
     * 
     * @remark: This function also internally calls:
     * pinocchio::computeStaticTorque() and pinocchio::computeGeneralizedGravity()
     * 
     * @remark: to get fext vector call:
     * pinocchio::forwardKinematics(model, data, q)
     * floating_base_model::model_helper_functions::computeSpatialForces(interface, 
     * info, input, fext)
     */
    template <typename SCALAR_T>
    Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> computeApproxActuatedJointGeneralizedTorques(
      ocs2::PinocchioInterfaceTpl<SCALAR_T>& interface,
      const floating_base_model::FloatingBaseModelInfoTpl<SCALAR_T>& info,
      const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q,
      const pinocchio::container::aligned_vector<pinocchio::ForceTpl<SCALAR_T, 0>>& fext);


    /* Explicit template instantiation for scalar_t and ad_scalar_t */
    extern template Eigen::Matrix<ocs2::scalar_t, Eigen::Dynamic, 1> computeApproxActuatedJointGeneralizedTorques(
      ocs2::PinocchioInterfaceTpl<ocs2::scalar_t>& interface,
      const floating_base_model::FloatingBaseModelInfoTpl<ocs2::scalar_t>& info,
      const Eigen::Matrix<ocs2::scalar_t, Eigen::Dynamic, 1>& q,
      const pinocchio::container::aligned_vector<pinocchio::ForceTpl<ocs2::scalar_t, 0>>& fext);

    extern template Eigen::Matrix<ocs2::ad_scalar_t, Eigen::Dynamic, 1> computeApproxActuatedJointGeneralizedTorques(
      ocs2::PinocchioInterfaceTpl<ocs2::ad_scalar_t>& interface,
      const floating_base_model::FloatingBaseModelInfoTpl<ocs2::ad_scalar_t>& info,
      const Eigen::Matrix<ocs2::ad_scalar_t, Eigen::Dynamic, 1>& q,
      const pinocchio::container::aligned_vector<pinocchio::ForceTpl<ocs2::ad_scalar_t, 0>>& fext);
  } // model_helper_functions
} // legged_locomotion_mpc

#endif
