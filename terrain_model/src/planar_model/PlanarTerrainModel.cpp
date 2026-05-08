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
  PlanarTerrainModel::PlanarTerrainModel(TerrainPlane terrainPlane, 
    std::string referenceFrameName): 
      TerrainModel(std::move(referenceFrameName)), 
      terrainPlane_(terrainPlane), sdf_(std::move(terrainPlane)) 
  {
    // Prepare gridMap
    const auto& terrainPosition3D = terrainPlane_.getPosition();
    const grid_map::Position gridMapPosition{terrainPosition3D.x(), terrainPosition3D.y()};
    const grid_map::Length gridMapLength{GRIDMAP_LENGTH, GRIDMAP_LENGTH};
    gridMap_.setFrameId(referenceFrameName_);
    gridMap_.setGeometry(gridMapLength, GRIDMAP_RESOLUTION, gridMapPosition);

    const size_t matrixSize = static_cast<size_t>(GRIDMAP_LENGTH / GRIDMAP_RESOLUTION);

    Eigen::MatrixXf data(matrixSize, matrixSize);

    vector2_t currentPosition = gridMapPosition;
    currentPosition.array() += 
      (matrixSize - 1) / (2 * matrixSize) * GRIDMAP_LENGTH;

    for(size_t i = 0; i < matrixSize; ++i)
    {
      for(size_t j = 0; j < matrixSize; ++j)
      {
        data(i, j) = terrainPlane_.projectPositionInWorldOntoPlaneAlongGravity(
          currentPosition).z();
        currentPosition(1) -= GRIDMAP_LENGTH / matrixSize;
      }
      currentPosition(1) += GRIDMAP_LENGTH;
      currentPosition(0) -= GRIDMAP_LENGTH / matrixSize;
    }
    gridMap_.add(elevationLayerName, data);
  }

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

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const grid_map::GridMap& PlanarTerrainModel::getGridMapTerrain() const
  {
    return gridMap_;
  }
} // namespace terrain_model
