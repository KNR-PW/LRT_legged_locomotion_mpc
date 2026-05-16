//
// Created by rgrandia on 23.06.20.
// Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2) on 30.04.2026
//

#include <terrain_model/segmented_planes_model/SegmentedPlanesTerrainModel.hpp>

#include <algorithm>

#include <grid_map_filters_rsl/lookup.hpp>

#include <convex_plane_decomposition/ConvexRegionGrowing.h>
#include <convex_plane_decomposition/SegmentedPlaneProjection.h>

namespace terrain_model
{
  using namespace ocs2;
  using namespace grid_map;
  using namespace convex_plane_decomposition;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  SegmentedPlanesTerrainModel::SegmentedPlanesTerrainModel(
    const PlanarTerrain& planarTerrain, std::string referenceFrameName): 
      TerrainModel(std::move(referenceFrameName)),
      planarTerrain_(planarTerrain), 
      signedDistanceField_(nullptr),
      elevationData_(&planarTerrain_.gridMap.get(elevationLayerName))
  {
    planarTerrain_.gridMap.setFrameId(referenceFrameName_);
    const auto ranges = getSignedDistanceRange(planarTerrain_.gridMap, elevationLayerName);
    createSignedDistanceBetween(ranges.first, ranges.second);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  SegmentedPlanesTerrainModel::SegmentedPlanesTerrainModel(
    PlanarTerrain&& planarTerrain, std::string referenceFrameName): 
    TerrainModel(std::move(referenceFrameName)),
    planarTerrain_(std::move(planarTerrain)),
    signedDistanceField_(nullptr),
    elevationData_(&planarTerrain_.gridMap.get(elevationLayerName))
  {
    planarTerrain_.gridMap.setFrameId(referenceFrameName_);
    const auto ranges = getSignedDistanceRange(planarTerrain_.gridMap, elevationLayerName);
    createSignedDistanceBetween(ranges.first, ranges.second);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  SegmentedPlanesTerrainModel* SegmentedPlanesTerrainModel::clone() const
  {
    return new SegmentedPlanesTerrainModel(*this);
  }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  TerrainPlane SegmentedPlanesTerrainModel::getLocalTerrainAtPositionInWorldAlongGravity(
    const vector3_t& positionInWorld, 
    std::function<scalar_t(const vector3_t&)> penaltyFunction) const
  {
    const auto projection = getBestPlanarRegionAtPositionInWorld(positionInWorld, 
      planarTerrain_.planarRegions, std::move(penaltyFunction));

    if(projection.regionPtr == nullptr)
    {
      throw std::runtime_error("[SegmentedPlanesTerrainModel]: No region found");
    }
      return TerrainPlane{projection.positionInWorld, 
        projection.regionPtr->transformPlaneToWorld.linear().transpose()
    };
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ConvexTerrain SegmentedPlanesTerrainModel::getConvexTerrainAtPositionInWorld(
    const vector3_t& positionInWorld, 
    std::function<scalar_t(const vector3_t&)> penaltyFunction) const
  {
    const auto projection = getBestPlanarRegionAtPositionInWorld(positionInWorld, 
      planarTerrain_.planarRegions, std::move(penaltyFunction));

    if (projection.regionPtr == nullptr)
    {
      std::stringstream stringStream;
      stringStream << positionInWorld.transpose();

      std::string message = "[SegmentedPlanesTerrainModel] No region found at: " + stringStream.str();
      throw std::runtime_error(message.c_str());
    }

    // Convert boundary and projection to terrain frame
    const int numberOfVertices = 16; // Multiple of 4 is nice for symmetry.
    const double growthFactor = 1.05;
    const auto convexRegion = growConvexPolygonInsideShape(
      projection.regionPtr->boundaryWithInset.boundary, 
      projection.positionInTerrainFrame, numberOfVertices, growthFactor);

    TerrainPlane convexPlane(projection.positionInWorld, 
      projection.regionPtr->transformPlaneToWorld.linear().transpose());
    // Origin is at the projection

    std::vector<vector2_t> convexBoundries;
    convexBoundries.reserve(convexRegion.size());
    for(const auto& point : convexRegion)
    {
      convexBoundries.emplace_back(point.x() - projection.positionInTerrainFrame.x(),
        point.y() - projection.positionInTerrainFrame.y());
      // Shift points to new origin
    }

    // Return convex region with origin at the projection
    return ConvexTerrain(std::move(convexPlane), std::move(convexBoundries));
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector3_t SegmentedPlanesTerrainModel::getHighestObstacleAlongLine(
    const vector3_t& position1InWorld,
    const vector3_t& position2InWorld) const
  {
    const auto result = grid_map::lookup::maxValueBetweenLocations(
      {position1InWorld.x(), position1InWorld.y()}, {position2InWorld.x(), position2InWorld.y()},
      planarTerrain_.gridMap, *elevationData_);

    if (result.isValid)
    {
      return {result.position.x(), result.position.y(), result.value};
    }
    else
    {
      // return highest query point if the map didn't work.
      if (position1InWorld.z() > position2InWorld.z())
      {
          return position1InWorld;
      }
      else
      {
          return position2InWorld;
      }
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::vector<vector2_t> SegmentedPlanesTerrainModel::getHeightProfileAlongLine(
    const vector3_t& position1InWorld,
    const vector3_t& position2InWorld) const
  {
    const vector2_t diff2d = position2InWorld.head<2>() - position1InWorld.head<2>();
    const scalar_t diffSquareNorm = diff2d.squaredNorm();
    const auto resolution = planarTerrain_.gridMap.getResolution();
    if (diffSquareNorm > (resolution * resolution))
    {
      // norm(p2-p1)_XY > resolution
      const auto pointsOnLine = grid_map::lookup::valuesBetweenLocations(
        {position1InWorld.x(), position1InWorld.y()},
        {position2InWorld.x(), position2InWorld.y()},
        planarTerrain_.gridMap, *elevationData_);
      std::vector<vector2_t> heightProfile;
      heightProfile.reserve(pointsOnLine.size());
      for (const auto& point : pointsOnLine)
      {
          const vector2_t pointDiff = point.head<2>() - position1InWorld.head<2>();
          const scalar_t lineProgress = pointDiff.dot(diff2d) / diffSquareNorm;
          if (lineProgress >= 0.0 && lineProgress <= 1.0)
          {
            heightProfile.push_back({lineProgress, point.z()});
          }
      }
      return heightProfile;
    }
    else
    {
      grid_map::Index index;
      planarTerrain_.gridMap.getIndex({position1InWorld.x(), position1InWorld.y()}, index);
      scalar_t heightData = (*elevationData_)(index(0), index(1));
      return {{0.0, heightData}, {1.0, heightData}};
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const SignedDistanceField* SegmentedPlanesTerrainModel::getSignedDistanceField() const
  {
    return signedDistanceField_.get();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector3_t SegmentedPlanesTerrainModel::getSmoothedPositon(
    const vector2_t& positionXYInWorld) const
  {
    auto projection = grid_map::lookup::projectToMapWithMargin(planarTerrain_.gridMap, 
      grid_map::Position(positionXYInWorld.x(), positionXYInWorld.y()));

    try 
    {
      auto z = planarTerrain_.gridMap.atPosition("smooth_planar", 
        projection, grid_map::InterpolationMethods::INTER_NEAREST);
      return vector3_t(positionXYInWorld.x(), positionXYInWorld.y(), z);
    } 
    catch (std::out_of_range& e) 
    {
      scalar_t interp = planarTerrain_.gridMap.getResolution() / (projection - planarTerrain_.gridMap.getPosition()).norm();
      projection = (1.0 - interp) * projection + interp * planarTerrain_.gridMap.getPosition();
      const auto z = planarTerrain_.gridMap.atPosition("smooth_planar", projection, 
        grid_map::InterpolationMethods::INTER_NEAREST);
      return vector3_t(positionXYInWorld.x(), positionXYInWorld.y(), z);
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const grid_map::GridMap& SegmentedPlanesTerrainModel::getGridMapTerrain() const
  {
    return planarTerrain_.gridMap;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void SegmentedPlanesTerrainModel::createSignedDistanceBetween(
    const Eigen::Vector3d& minCoordinates, const Eigen::Vector3d& maxCoordinates)
  {
    // Compute coordinates of submap
    const auto minXY = grid_map::lookup::projectToMapWithMargin(planarTerrain_.gridMap,
      grid_map::Position(minCoordinates.x(), minCoordinates.y()));
    const auto maxXY = grid_map::lookup::projectToMapWithMargin(planarTerrain_.gridMap,
      grid_map::Position(maxCoordinates.x(), maxCoordinates.y()));

    const auto centerXY = 0.5 * (minXY + maxXY);
    const auto lengths = maxXY - minXY;
    bool success = true;
    grid_map::GridMap subMap = planarTerrain_.gridMap.getSubmap(centerXY, lengths, success);
    if (success)
    {
      signedDistanceField_ = std::make_unique<SegmentedPlanesSignedDistanceField>(subMap, 
        elevationLayerName, minCoordinates.z(), maxCoordinates.z());
    }
    else
    {
      throw std::runtime_error("[SegmentedPlanesTerrainModel]: Failed to get subMap");
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  SegmentedPlanesTerrainModel::SegmentedPlanesTerrainModel(
    const SegmentedPlanesTerrainModel& rhs):
    planarTerrain_(rhs.planarTerrain_), 
    signedDistanceField_(new SegmentedPlanesSignedDistanceField(*rhs.signedDistanceField_)),
    elevationData_(&planarTerrain_.gridMap.get(elevationLayerName)) {}
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::pair<Eigen::Vector3d, Eigen::Vector3d> SegmentedPlanesTerrainModel::getSignedDistanceRange(
    const grid_map::GridMap& gridMap, const std::string& elevationLayer)
  {
    // Read min-max from elevation map
    const float heightMargin = 0.1;
    const auto& elevationData = planarTerrain_.gridMap.get(elevationLayer);
    const float minValue = elevationData.minCoeffOfFinites() - heightMargin;
    const float maxValue = elevationData.maxCoeffOfFinites() + heightMargin;
    auto minXY = grid_map::lookup::projectToMapWithMargin(
        gridMap, grid_map::Position(std::numeric_limits<double>::lowest(),
                                    std::numeric_limits<double>::lowest()));
    auto maxXY = grid_map::lookup::projectToMapWithMargin(
        gridMap, grid_map::Position(std::numeric_limits<double>::max(),
                                      std::numeric_limits<double>::max()));

    const vector3_t minCoordinates{minXY.x(), minXY.y(), minValue};
    const vector3_t maxCoordinates{maxXY.x(), maxXY.y(), maxValue};
      
    return {minCoordinates, maxCoordinates};
  }
} // namespace terrain_model
