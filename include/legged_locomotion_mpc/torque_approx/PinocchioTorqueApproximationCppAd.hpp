// Copyright (c) 2025, Bartłomiej Krajewski
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

#ifndef __PINOCCHIO_TORQUE_APPROXIMATION_CPP_AD_LEGGED_LOCOMOTION_MPC__
#define __PINOCCHIO_TORQUE_APPROXIMATION_CPP_AD_LEGGED_LOCOMOTION_MPC__

#include <legged_locomotion_mpc/common/Types.hpp>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

#include <string>
#include <vector>

namespace legged_locomotion_mpc 
{

  /**
   * This class provides the CppAD implementation of the joint torque approximation
   * based on pinocchio. No pre-computation is required. 
   * Torque is calculated using transpose jacobian matrix: τ = -J^T * Fext
   */
  class PinocchioTorqueApproximationCppAd
  {
    public:

    /** Constructor
     * @param [in] pinocchioInterface: pinocchio interface.
     * @param [in] info: info of kinematics model
     * @param [in] torqueDynamicsError: constant torque error between real 
     * and aproximated value
     * @param [in] modelName: name of the generate model library
     * @param [in] modelFolder: folder to save the model library files to
     * @param [in] recompileLibraries: If true, the model library will be newly
     * compiled. If false, an existing library will be loaded if available.
     * @param [in] verbose: print information.
     */
    PinocchioTorqueApproximationCppAd(
      const ocs2::PinocchioInterface& pinocchioInterface,
      const floating_base_model::FloatingBaseModelInfo info,
      const ocs2::vector_t torqueDynamicsError,
      const std::string& modelName,
      const std::string& modelFolder = "/tmp/ocs2",
      bool recompileLibraries = true, bool verbose = false);

    ~PinocchioTorqueApproximationCppAd() = default;

    PinocchioTorqueApproximationCppAd* clone() const;

    PinocchioTorqueApproximationCppAd& operator =(
      const PinocchioTorqueApproximationCppAd&) = delete;

    ocs2::vector_t getValue(const ocs2::vector_t& state, const ocs2::vector_t& input) const;

    ocs2::VectorFunctionLinearApproximation getLinearApproximation(const ocs2::vector_t& state,
      const ocs2::vector_t& input) const;

   private:
    PinocchioTorqueApproximationCppAd(
        const PinocchioTorqueApproximationCppAd& rhs);

    ocs2::ad_vector_t getValueCppAd(ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
      const ocs2::PinocchioStateInputMapping<ocs2::ad_scalar_t>& mapping,
      const ocs2::ad_vector_t& state,
      const ocs2::ad_vector_t& input);

    std::unique_ptr<ocs2::CppAdInterface> torqueApproxCppAdInterfacePtr_;

    const ocs2::vector_t torqueDynamicsError_;

    const floating_base_model::FloatingBaseModelInfo info_;
  };
} // namespace legged_locomotion_mpc

#endif