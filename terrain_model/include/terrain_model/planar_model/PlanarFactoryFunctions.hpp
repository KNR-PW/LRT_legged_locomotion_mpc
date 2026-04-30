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


#ifndef __PLANAR_FACTORY_FUNCTIONS_TERRAIN_MODEL__
#define __PLANAR_FACTORY_FUNCTIONS_TERRAIN_MODEL__

#include <terrain_model/core/Types.hpp>
#include <terrain_model/core/SignedDistanceField.hpp>
#include <terrain_model/core/TerrainPlane.hpp>

namespace terrain_model
{
  /**
   * Calculates terrain plane based on queried position, rotation in world frame
   * and its height (distance) from terrain plane
   * @param [in] position: 3D position in world frame
   * @param [in] eulerAngles: 3D rotation defined by euler angles terrain -> world
   * @param [in] height: distance from terrain plane
   * @return terrain plane
   */
  TerrainPlane computeTerrainPlane(const vector3_t& position, 
    const vector3_t& eulerAngles, ocs2::scalar_t height);
  
  /**
   * Calculates terrain plane based on many queried points using best fit method
   * @param [in] positions: 3D positions in world frame
   * @return terrain plane
   */
  TerrainPlane computeTerrainPlane(const std::vector<vector3_t>& positions); 
}; // namespace terrain_model

#endif