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


#ifndef __MOTION_PHASE_DEFINITIONS_LEGGED_LOCOMOTION_MPC__
#define __MOTION_PHASE_DEFINITIONS_LEGGED_LOCOMOTION_MPC__

#include <legged_locomotion_mpc/common/Types.hpp>

namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    /**
     * Contact modes used for passing information to OCS2 framework
     * Defined as `size_t` value, where each contact flag is represented as 1 bit in position 
     * corresponding to position in contact_flags_t (0 index -> LSB)
     *
     */

    /**
     * Get contact flags from mode number
     * @param [in] modeNumber : Mode number
     * @param [in] endEffectorNumber: Number of end effectors (legs)
     * @return Vector with contact flags
     */
    inline contact_flags_t modeNumber2ContactFlags(const size_t modeNumber, const size_t endEffectorNumber)
    {
      contact_flags_t contactFlags(endEffectorNumber);

      for(int i = 0; i < endEffectorNumber; ++i)
      {
        contactFlags[i] = (modeNumber >> i) & 0x01; 
      }

      return contactFlags;
    };

    /**
     * Get mode number from contact flags
     * @param [in] contactFlags : Vector with contact flags
     * @return Mode number
     */
    inline size_t contactFlags2ModeNumber(const contact_flag_t& contactFlags)
    {
      size_t modeNumber = 0;
      
      for(const auto& flag: contactFlags)
      {
        modeNumber |= 0x01 << static_cast<size_t>(flag);
      }

      return modeNumber;
    };
  };
};

#endif