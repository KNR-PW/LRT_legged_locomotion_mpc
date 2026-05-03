//
// Created by rgrandia on 23.06.20.
// Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2) on 30.04.2026 
//

#ifndef __SEGMENTED_PLANES_TERRAIN_MODEL_TERRAIN_MODEL__
#define __SEGMENTED_PLANES_TERRAIN_MODEL_TERRAIN_MODEL__

#include <terrain_model/core/Types.hpp>
#include <terrain_model/core/TerrainModel.hpp>
#include <terrain_model/segmented_planes_model/SegmentedPlanesSignedDistanceField.h>

#include <convex_plane_decomposition/PlanarRegion.h>

namespace terrain_model
{
  class SegmentedPlanesTerrainModel : public TerrainModel 
  {
    public:

      SegmentedPlanesTerrainModel(
        convex_plane_decomposition::PlanarTerrain planarTerrain);

      SegmentedPlanesTerrainModel(
        convex_plane_decomposition::PlanarTerrain&& planarTerrain);

      TerrainPlane getLocalTerrainAtPositionInWorldAlongGravity(
        const vector3_t& positionInWorld,
        std::function<ocs2::scalar_t(const vector3_t&)> penaltyFunction) const override;

      vector3_t getHighestObstacleAlongLine(
        const vector3_t& position1InWorld,
        const vector3_t& position2InWorld) const override;

      std::vector<vector2_t> getHeightProfileAlongLine(
        const vector3_t& position1InWorld,
        const vector3_t& position2InWorld) const override;

      ConvexTerrain getConvexTerrainAtPositionInWorld(
        const vector3_t& positionInWorld,
        std::function<ocs2::scalar_t(const vector3_t&)> penaltyFunction) const override;

      const SignedDistanceField* getSignedDistanceField() const override;

      vector3_t getSmoothedPositon(const vector2_t& positionXYInWorld) const override;

      const grid_map::GridMap& getGridMapTerrain() const override;

    private:
      void createSignedDistanceBetween(const Eigen::Vector3d& minCoordinates,
        const Eigen::Vector3d& maxCoordinates);

      std::pair<Eigen::Vector3d, Eigen::Vector3d> getSignedDistanceRange(
        const grid_map::GridMap& gridMap, const std::string& elevationLayer);

      const convex_plane_decomposition::PlanarTerrain planarTerrain_;
      std::unique_ptr<SegmentedPlanesSignedDistanceField> signedDistanceField_;
      const grid_map::Matrix* const elevationData_;
  };
} // namespace terrain_model

#endif