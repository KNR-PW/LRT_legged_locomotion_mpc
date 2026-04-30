
#include <terrain_model/core/TerrainModel.hpp>

namespace terrain_model
{

  using namespace ocs2;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  TerrainPlane TerrainModel::getLocalTerrainAtPositionInWorldAlongGravity(
    const vector3_t& positionInWorld) const
  {
    return getLocalTerrainAtPositionInWorldAlongGravity(positionInWorld,
      [](const vector3_t&) { return 0.0; });
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ConvexTerrain TerrainModel::getConvexTerrainAtPositionInWorld(const vector3_t& positionInWorld) const
  {
    return getConvexTerrainAtPositionInWorld(positionInWorld, 
      [](const vector3_t&) { return 0.0; });
  }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ConvexTerrain TerrainModel::getConvexTerrainAtPositionInWorld(const vector3_t& positionInWorld,
    std::function<scalar_t(const vector3_t&)> penaltyFunction) const
  {
    return ConvexTerrain(getLocalTerrainAtPositionInWorldAlongGravity(positionInWorld,
      std::move(penaltyFunction)), {});
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const SignedDistanceField* TerrainModel::getSignedDistanceField() const 
  { 
    return nullptr; 
  }
} // namespace terrain_model