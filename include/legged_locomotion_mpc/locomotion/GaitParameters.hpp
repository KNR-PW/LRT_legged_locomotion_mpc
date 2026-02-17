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


#ifndef __GAIT_PARAMETERS_LEGGED_LOCOMOTION_MPC__
#define __GAIT_PARAMETERS_LEGGED_LOCOMOTION_MPC__

#include <cmath>

#include <legged_locomotion_mpc/common/Types.hpp>

namespace legged_locomotion_mpc
{
  namespace locomotion
  {

    namespace Definitions
    {
      const size_t DEFAULT_BUFFER_SIZE = 10; 
      const ocs2::scalar_t GAIT_PARAMETERS_EPS = 1e-3;
      
      // If mode changes between modes are smaller than this, they are in same mode change!
      const ocs2::scalar_t MIN_TIME_BETWEEN_CHANGES = 1e-4;
    }

    enum class GaitFlags
    {
      OK = 0,
      IN_THE_AIR = 1,
      EARLY_TOUCHDOWN = 2,
      EARLY_TOUCHDOWNS = 3,
      ERROR = 4,
    };

    struct GaitStaticParameters
    {
      size_t endEffectorNumber;
      ocs2::scalar_t timeHorizion;
      ocs2::scalar_t maximumSteppingFrequency = 5.0; // [Hz]
      ocs2::scalar_t minimumSteppingFrequency = 0.0; // [Hz]
      ocs2::scalar_t touchDownWindow = 0.1; // [s]
    };

    struct GaitDynamicParameters
    {
      ocs2::scalar_t steppingFrequency; // [Hz]
      ocs2::scalar_t swingRatio; // val < swingRatio -> SWING, else STANCE
      std::vector<ocs2::scalar_t> phaseOffsets; // len: endEffectorNumber - 1
    };

    inline bool operator==(const GaitDynamicParameters& lhs, const GaitDynamicParameters& rhs) 
    { 
      if(lhs.phaseOffsets.size() != rhs.phaseOffsets.size()) return false;

      ocs2::scalar_t phaseOffsetsDifference = 0.0;

      for(size_t i = 0; i < lhs.phaseOffsets.size(); i++)
      {
        phaseOffsetsDifference += std::abs(lhs.phaseOffsets[i] - rhs.phaseOffsets[i]);
      }
      return std::abs(lhs.steppingFrequency - rhs.steppingFrequency) < Definitions::GAIT_PARAMETERS_EPS
        &&  std::abs(lhs.swingRatio - rhs.swingRatio ) < Definitions::GAIT_PARAMETERS_EPS
        &&  phaseOffsetsDifference < Definitions::GAIT_PARAMETERS_EPS
        && lhs.phaseOffsets.size() == rhs.phaseOffsets.size();
    };
      
    inline bool operator!=(const GaitDynamicParameters& lhs, const GaitDynamicParameters& rhs) 
    { 
      return !(lhs == rhs); 
    };
    
  } // namespace locomotion
} // namespace legged_locomotion_mpc

#endif