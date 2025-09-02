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

#ifndef __ACCESS_HELPER_FUNCTIONS_LEGGED_LOCOMOTION_MPC__
#define __ACCESS_HELPER_FUNCTIONS_LEGGED_LOCOMOTION_MPC__

#include <Eigen/Dense>

#include <pinocchio/container/aligned-vector.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>


namespace legged_locomotion_mpc
{
  namespace access_helper_functions
  {
    /**
    * Provides read/write access to the base position.
    * @param [in] input: system input vector
    * @param [in] contactIndex: index of contact frame
    * @param [in] info: info of FloatingBase model
    * @return block with contact force
    */
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 3, 1> getBasePosition(Eigen::MatrixBase<Derived>& input,
      size_t contactIndex, 
      const FloatingBaseModelInfoTpl<SCALAR_T>& info);
  };
};

#endif