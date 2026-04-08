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

#ifndef __PINOCCHIO_WEIGHT_COMPENSATOR_LEGGED_LOCOMOTION_MPC__
#define __PINOCCHIO_WEIGHT_COMPENSATOR_LEGGED_LOCOMOTION_MPC__

#include <string>
#include <vector>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>
#include <floating_base_model/FloatingBaseModelPinocchioMapping.hpp>

#include <legged_locomotion_mpc/common/Types.hpp>

namespace legged_locomotion_mpc 
{

  /**
   * This class provides the CppAD implementation of the weight compensation wrenches
   * based on pinocchio. No pre-computation is required. 
   */
  class PinocchioWeightCompensator
  {
    public:

    /** Constructor
     * @param [in] pinocchioInterface: pinocchio interface.
     * @param [in] info: info of kinematics model
     */
    PinocchioWeightCompensator(
      const ocs2::PinocchioInterface& pinocchioInterface,
      const floating_base_model::FloatingBaseModelInfo& info);

    ~PinocchioWeightCompensator() = default;

    PinocchioWeightCompensator* clone() const;

    PinocchioWeightCompensator(
        const PinocchioWeightCompensator& rhs);

    PinocchioWeightCompensator& operator =(
      const PinocchioWeightCompensator&) = delete;

    ocs2::vector_t getInput(const ocs2::vector_t& state, 
      const contact_flags_t& contactFlags);

    void appendInput(const ocs2::vector_t& state, ocs2::vector_t& input, 
      const contact_flags_t& contactFlags);

   private:
    const floating_base_model::FloatingBaseModelInfo info_;

    ocs2::PinocchioInterface pinocchioInterface_;

    floating_base_model::FloatingBaseModelPinocchioMapping mapping_;
  };
} // namespace legged_locomotion_mpc

#endif