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
      const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesZyx)
    {
      std::array<std::array<Eigen::Matrix<SCALAR_T, 3, 1>, 3>, 3> eulerGradient;

      const SCALAR_T z = eulerAnglesZyx[0];  // alfa  (Z)
      const SCALAR_T y = eulerAnglesZyx[1];  // beta  (Y)
      const SCALAR_T x = eulerAnglesZyx[2];  // gamma (X)
      
      const SCALAR_T cz = cos(z);
      const SCALAR_T cy = cos(y);
      const SCALAR_T cx = cos(x);
      const SCALAR_T sz = sin(z);
      const SCALAR_T sy = sin(y);
      const SCALAR_T sx = sin(x);

      const SCALAR_T szcy = sz * cy;
      const SCALAR_T czsy = cz * sy;
      const SCALAR_T szsy = sz * sy;
      const SCALAR_T czcy = cz * cy;

      const SCALAR_T sycx = sy * cx;
      const SCALAR_T cysx = cy * sx;
      const SCALAR_T sysx = sy * sx;
      const SCALAR_T cycx = cy * cx;

      const SCALAR_T szcx = sz * cx;
      const SCALAR_T czsx = cz * sx;
      const SCALAR_T szsx = sz * sx;
      const SCALAR_T czcx = cz * cx;

      const SCALAR_T szsysx = szsy * sx;
      const SCALAR_T czsycx = czsy * cx;
      const SCALAR_T szsycx = szsy * cx;
      const SCALAR_T czsysx = czsy * sx;

      eulerGradient[0][0] << -szcy, -czsy, SCALAR_T(0.0);
      eulerGradient[0][1] << -szsysx - czcx, czcy * sx, czsycx + szsx;
      eulerGradient[0][2] << -szsycx + czsx, czcy * cx, -czsysx + szcx;

      eulerGradient[1][0] << czcy, -szsy, SCALAR_T(0.0);
      eulerGradient[1][1] << czsysx - szcx, szcy * sx, szsycx - czsx; 
      eulerGradient[1][2] << czsycx + szsx, szcy * cx, -szsysx - czcx;

      eulerGradient[2][0] << SCALAR_T(0.0), -cy, SCALAR_T(0);
      eulerGradient[2][1] << SCALAR_T(0), -sysx, cycx;
      eulerGradient[2][2] << SCALAR_T(0), -sycx, -cysx;

      return eulerGradient;
    }
  } // euler_transforms
} // legged_locomotion_mpc