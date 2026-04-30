//
// Created by rgrandia on 29.04.20.
// Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2) on 29.08.2025 
//

#include <terrain_model/planar_model/PlanarTerrainModel.hpp>

namespace terrain_model
{
  
  using namespace ocs2;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PlanarTerrainModel::PlanarTerrainModel(TerrainPlane terrainPlane)
    : terrainPlane_(terrainPlane), sdf_(std::move(terrainPlane)) {}

   /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PlanarTerrainModel* PlanarTerrainModel::clone() const
  {
    return new PlanarTerrainModel(*this);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  TerrainPlane PlanarTerrainModel::getLocalTerrainAtPositionInWorldAlongGravity(
    const vector3_t &positionInWorld,
    std::function<scalar_t(const vector3_t &)> penaltyFunction) const 
  {
    // Project point to plane to find new center, orientation stays the same
    return TerrainPlane(
      terrainPlane_.projectPositionInWorldOntoPlaneAlongGravity(positionInWorld),
      terrainPlane_.getOrientationToTerrain());
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const SignedDistanceField*  PlanarTerrainModel::getSignedDistanceField() const
  { 
    return &sdf_; 
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector3_t PlanarTerrainModel::getHighestObstacleAlongLine(
    const vector3_t &position1InWorld,
    const vector3_t &position2InWorld) const 
  {
    // The highest point on a plane is at the end of the line
    const auto projection1 = terrainPlane_.projectPositionInWorldOntoPlaneAlongGravity(position1InWorld);
    const auto projection2 = terrainPlane_.projectPositionInWorldOntoPlaneAlongGravity(position2InWorld);

    if (projection1.z() > projection2.z()) 
    {
      return projection1;
    } else 
    {
      return projection2;
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ConvexTerrain PlanarTerrainModel::getConvexTerrainAtPositionInWorld(
    const vector3_t& positionInWorld,
    std::function<ocs2::scalar_t(const vector3_t&)>
    penaltyFunction) const
  {
    // As planar terrain is safe everywhere, convex terrain will be big triangle lmao 
    std::vector<vector2_t> boundryPoints = {{-100, -100}, {100, -100}, {0, 100}};
    const vector3_t positionOnTerrain = terrainPlane_.projectPositionInWorldOntoPlaneAlongGravity(positionInWorld);
    TerrainPlane newPlane(positionOnTerrain, terrainPlane_.getOrientationToTerrain());
    return ConvexTerrain(newPlane, boundryPoints);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::vector<vector2_t> PlanarTerrainModel::getHeightProfileAlongLine(
    const vector3_t &position1InWorld,
    const vector3_t &position2InWorld) const 
  {
    // Provide end points and one middle point as the height profile.
    const auto projection1 = terrainPlane_.projectPositionInWorldOntoPlaneAlongGravity(position1InWorld);
    const auto projection2 = terrainPlane_.projectPositionInWorldOntoPlaneAlongGravity(position2InWorld);
    return {{0.0, projection1.z()}, {0.5, 0.5 * (projection1.z() + projection2.z())}, {1.0, projection2.z()}};
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector3_t PlanarTerrainModel::getSmoothedPositon(const vector2_t& positionXYInWorld) const
  {
    return terrainPlane_.projectPositionInWorldOntoPlaneAlongGravity(positionXYInWorld);
  }
} // namespace terrain_model
