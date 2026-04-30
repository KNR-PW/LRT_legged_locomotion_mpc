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

#ifndef __EULER_TRANSFORMS_LEGGED_LOCOMOTION_MPC__
#define __EULER_TRANSFORMS_LEGGED_LOCOMOTION_MPC__

#include <array>

#include <Eigen/Dense>

#include <legged_locomotion_mpc/common/Types.hpp>

namespace legged_locomotion_mpc
{
  namespace euler_transforms
  {
    /**
     * Get rotation matrix gradient with respect to euler ZYX angles
     * @param [in] eulerAngles: Euler ZYX angles
     * @return Array of arrays where a[i][j] is 3x1 gradient of element i,j 
     * in rotation matrix
     */
    template <typename SCALAR_T>
    std::array<std::array<Eigen::Matrix<SCALAR_T, 3, 1>, 3>, 3> getRotationMatrixEulerZyxGradient(
      const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesZyx);
  } // euler_transforms
} // legged_locomotion_mpc

#include <legged_locomotion_mpc/common/EulerTransforms.hxx>

#endif