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

#ifndef __MODE_DYNAMIC_SEQUENCE_TEMPLATE_LEGGED_LOCOMOTION_MPC__
#define __MODE_DYNAMIC_SEQUENCE_TEMPLATE_LEGGED_LOCOMOTION_MPC__

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/locomotion/ModeSequenceTemplate.hpp>
#include <legged_locomotion_mpc/locomotion/GaitParameters.hpp>

namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    ModeSequenceTemplate getDynamicModeSequenceTemplate(
      ocs2::scalar_t currentPhase,
      ocs2::scalar_t timeHorizon,
      const GaitStaticParameters& staticParams,
      const GaitDynamicParameters& dynamicParams);
  } // namespace locomotion
} // namespace legged_locomotion_mpc


#endif
