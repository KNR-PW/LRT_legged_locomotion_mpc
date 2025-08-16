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


#ifndef __GAIT_PARAMETERS_LEGGED_LOCOMOTION_MPC__
#define __GAIT_PARAMETERS_LEGGED_LOCOMOTION_MPC__

#include <legged_locomotion_mpc/common/Types.hpp>

namespace legged_locomotion_mpc
{
  namespace locomotion
  {

    namespace Definitions
    {
      static const ocs2::scalar_t MAX_STEPPING_FREQUENCY = 5.0;  // [Hz]
      static const ocs2::scalar_t MIN_STEPPING_FREQUENCY = 0.1;  // [Hz]
    }

    struct GaitStaticParameters
    {
      size_t endEffectorNumber;
      ocs2::scalar_t plannerFrequency;
      ocs2::scalar_t timeHorizion;
    };

    struct GaitDynamicParameters
    {
      ocs2::scalar_t steppingFrequency;
      ocs2::scalar_t swingRatio; // val < swingRatio -> SWING, else STANCE
      std::vector<ocs2::scalar_t> phaseOffsets; // len: endEffectorNumber - 1
    };
  }; // namespace locomotion
}; // namespace legged_locomotion_mpc

#endif