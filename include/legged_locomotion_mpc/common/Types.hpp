/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2), 2025

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef __TYPES_LEGGED_LOCOMOTION_MPC__
#define __TYPES_LEGGED_LOCOMOTION_MPC__

#include <vector>
#include <cstddef>
#include <bitset>

#include <ocs2_core/Types.h>
namespace legged_locomotion_mpc
{ 
  static const size_t MAX_LEG_NUMBER = 8;
  using contact_flags_t = std::bitset<MAX_LEG_NUMBER>; // Dont expect for robot to have more than 8 legs

  using vector3_t = Eigen::Matrix<ocs2::scalar_t, 3, 1>;
  using matrix3_t = Eigen::Matrix<ocs2::scalar_t, 3, 3>;
  using vector6_t = Eigen::Matrix<ocs2::scalar_t, 6, 1>;
  using matrix6_t = Eigen::Matrix<ocs2::scalar_t, 6, 6>;
  using quaternion_t = Eigen::Quaternion<ocs2::scalar_t>;


  /** 
   * Vector representing state of the robot defined as:
   * 1. Base linear velocity (in base frame of reference): 0 - 2
   * 2. Base angular velocity (in base frame of reference): 3 - 5
   * 3. Base position (in world frame of reference): 6 - 8
   * 4. Base orientation (euler angles, in world frame of reference): 9 - 11
   * 5. Actuated joint positions: 11 - (11 + number of actuated joints - 1)
   * 6. Actuated joint velocities: (11 + number of actuated joints) - (11 + 2 * number of actuated joints - 1)
   */
  using state_vector_t = Eigen::Matrix<ocs2::scalar_t, Eigen::Dynamic, 1>;
}

#endif