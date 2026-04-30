//
// Created by rgrandia on 08.09.22.
// Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2) on 29.08.2025 
//

#include <terrain_model/planar_model/PlanarSignedDistanceField.hpp>

namespace terrain_model 
{

  using namespace ocs2;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PlanarSignedDistanceField::PlanarSignedDistanceField(TerrainPlane terrainPlane) 
   : terrainPlane_(std::move(terrainPlane)) {}
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PlanarSignedDistanceField* PlanarSignedDistanceField::clone() const
  {
    return new PlanarSignedDistanceField(*this);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  scalar_t PlanarSignedDistanceField::value(const vector3_t &position) const 
  {
    return terrainPlane_.getTerrainSignedDistanceFromPositionInWorld(position);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector3_t PlanarSignedDistanceField::derivative(const vector3_t &position) const 
  {
    return terrainPlane_.getSurfaceNormalInWorld();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::pair<scalar_t, vector3_t> PlanarSignedDistanceField::valueAndDerivative(
    const vector3_t &position) const 
  {
    return {value(position), derivative(position)};
  }
} // namespace terrain_model
