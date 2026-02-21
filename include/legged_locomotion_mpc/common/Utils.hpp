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

#include <legged_locomotion_mpc/common/Types.hpp>

#include <ocs2_core/reference/TargetTrajectories.h>

namespace legged_locomotion_mpc
{ 
  namespace utils
  {
    /**
     * Subsample reference trajectory, so that maximum interval between 
     * trajectories are less or equal maximumReferenceSampleInterval
     *
     * @param [in] targetTrajectories: target trajectories
     * @param [in] initTime: initial time
     * @param [in] finalTime: final time
     * @param [in] maximumReferenceSampleInterval: maximum time between trajectory points
     * @return subsampled target trajectories
     * 
     */
    ocs2::TargetTrajectories subsampleReferenceTrajectory(
      const ocs2::TargetTrajectories& targetTrajectories,
      ocs2::scalar_t initTime,
      ocs2::scalar_t finalTime,
      ocs2::scalar_t maximumReferenceSampleInterval);

    /**
     * Provides number of feet in contact.
     * @param [in] info: info of FloatingBase model
     * @param [in] contactFlags: std::vector with contact flags
     * @return number of feet in contact
     */
    size_t numberOfClosedContacts(
      const floating_base_model::FloatingBaseModelInfo &info,
      const contact_flags_t &contactFlags);


    /**
     * Computes an input with zero joint velocity and forces which 
     *  equally distribute the robot weight between contact feet.
     * @param [in] info: info of FloatingBase model
     * @param [in] contactFlags: std::vector with contact flags
     * @return input vector with calculated forces
     */
    ocs2::vector_t weightCompensatingInput(
      const floating_base_model::FloatingBaseModelInfo &info, 
      const contact_flags_t &contactFlags);

    /**
     * Computes an input with zero joint velocity and forces which 
     *  equally distribute the robot weight between contact feet.
     * @param [in] info: info of FloatingBase model
     * @param [in] contactFlags: std::vector with contact flags
     * @param [out] input: input vector with new calculated forces
     */
    void weightCompensatingAppendInput(ocs2::vector_t& input,
      const floating_base_model::FloatingBaseModelInfo &info, 
      const contact_flags_t &contactFlags);

    /**
     *  Find index into a sorted time Array
     *
     *  Indices are counted as follows:
     *          ------ | ----- | ---  ... ---    | -----
     *                t0     t1              t(n-1)
     *  Index     0        1      2   ...  (n-1)    n
     *
     *  Corner cases:
     *     - If time equal to a time in the timeArray is requested, the lower index is taken (e.g. t = t1 -> index = 1)
     *     - If multiple times in the timeArray are equal, the index before the first occurrence is taken.
     *       for example: if t1 = t2 = t3  and the requested time t <= t3 -> index = 1
     *
     *  It is a fixed version, ocs2::findIndexInTimeArray does not work properly!
     * @tparam SCALAR : numerical type of time
     * @param timeArray : sorted time array to perform the lookup in
     * @param time : enquiry time
     * @return index between [0, size(timeArray)]
     */
    size_t findIndexInTimeArray(const std::vector<ocs2::scalar_t> &timeArray, 
      ocs2::scalar_t time);
      
  } // namespace utils
} // namespace legged_locomotion_mpc

#endif