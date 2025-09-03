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

#ifndef __UTILS_LEGGED_LOCOMOTION_MPC__
#define __UTILS_LEGGED_LOCOMOTION_MPC__

#include <array>
#include <floating_base_model/AccessHelperFunctions.hpp>

#include "legged_locomotion_mpc/common/Types.hpp"

namespace legged_locomotion_mpc
{ 
  namespace utils
  {

    /**
     * Transform vector of robot state defined in file Types.hpp
     * to ocs2 state and input vectors.
     *
     * @param [in] info: floating base model info
     * @param [in] robotState: vector of real state of the robot
     * @return ocs2 state and input vectors used in optimization
     * 
     * @remark: forces acting on end effectors are missing (value 0)
     */
    std::pair<ocs2::vector_t, ocs2::vector_t> robotStateToOptimizationStateAndInput(
      const floating_base_model::FloatingBaseModelInfo& info,
      const ocs2::vector_t& robotState);

    /**
      * Provides number of feet in contact.
      * @param [in] contactFlags: std::vector with contact flags
      * @return number of feet in contact
      */
    size_t numberOfClosedContacts(const contact_flags_t &contactFlags);


    /**
      * Computes an input with zero joint velocity and forces which 
      *  equally distribute the robot weight between contact feet.
      * @param [in] info: info of FloatingBase model
      * @param [in] contactFlags: std::vector with contact flags
      * @return input vector with calculated forces
      */
    ocs2::vector_t weightCompensatingInput(const floating_base_model::FloatingBaseModelInfo &info, 
      const contact_flags_t &contactFlags);

  }; // namespace utils
}; // namespace legged_locomotion_mpc

#endif