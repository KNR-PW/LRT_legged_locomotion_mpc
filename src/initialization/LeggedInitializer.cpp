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

#include <legged_locomotion_mpc/initialization/LeggedInitializer.hpp>

#include <legged_locomotion_mpc/common/Utils.hpp>

namespace legged_locomotion_mpc
{
  using namespace ocs2;
  using namespace floating_base_model;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  LeggedInitializer::LeggedInitializer(FloatingBaseModelInfo info,
    const LeggedReferenceManager& referenceManager):
      info_(info), referenceManager_(referenceManager) {}
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  LeggedInitializer* LeggedInitializer::clone() const
  {
    return new LeggedInitializer(*this);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedInitializer::compute(scalar_t time, const vector_t& state, 
    scalar_t nextTime, vector_t& input, vector_t& nextState)
  {
    nextState = state;
    const contact_flags_t currentFlags = referenceManager_.getContactFlags(time);
    utils::weightCompensatingAppendInput(input, info_, currentFlags);
  }
} // namespace legged_locomotion_mpc
