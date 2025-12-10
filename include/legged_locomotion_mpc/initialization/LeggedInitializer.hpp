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

#ifndef __LEGGED_INITIALIZER_LEGGED_LOCOMOTION_MPC__
#define __LEGGED_INITIALIZER_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/initialization/Initializer.h>

#include "floating_base_model/FloatingBaseModelInfo.hpp"

#include <legged_locomotion_mpc/reference_manager/LeggedReferenceManager.hpp>

namespace legged_locomotion_mpc
{
  /**
   * This class provides intializer for legged robot. It computes same state 
   * and input when optimal problem is initialized. Next state and input are computed as:
   * - next state = previous state
   * - input = weight compensation
   */
  class LeggedInitializer : public ocs2::Initializer 
  {
    public:

      /** Constructor
       * @param [in] info: info of kinematics model
       * @param [in] referenceManager: Legged model ReferenceManager.
       */
      LeggedInitializer(floating_base_model::FloatingBaseModelInfo info,
        const LeggedReferenceManager& referenceManager);

      ~LeggedInitializer() override = default;

      LeggedInitializer* clone() const override;

      void compute(ocs2::scalar_t time, const ocs2::vector_t& state, 
        ocs2::scalar_t nextTime, ocs2::vector_t& input, ocs2::vector_t& nextState) override;

    private:
      LeggedInitializer(const LeggedInitializer &rhs) = default;
      
      const floating_base_model::FloatingBaseModelInfo info_;
      const LeggedReferenceManager& referenceManager_;
  };
} // namespace legged_locomotion_mpc

#endif