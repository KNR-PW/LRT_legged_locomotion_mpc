#include <terrain_model/segmented_planes_model/SegmentedPlanesSignedDistanceField.hpp>

namespace terrain_model
{
  using namespace ocs2;
  using namespace grid_map;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  SegmentedPlanesSignedDistanceField::SegmentedPlanesSignedDistanceField(
    const grid_map::GridMap& gridMap, const std::string& elevationLayer,
    scalar_t minHeight, scalar_t maxHeight)
  {
    sdf_.calculateSignedDistanceField(gridMap, elevationLayer, maxHeight);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  SegmentedPlanesSignedDistanceField* SegmentedPlanesSignedDistanceField::clone() const 
  {
    return new SegmentedPlanesSignedDistanceField(*this);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  scalar_t SegmentedPlanesSignedDistanceField::value(const vector3_t& position) const 
  {
    return sdf_.getDistanceAt(position);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector3_t SegmentedPlanesSignedDistanceField::derivative(const vector3_t& position) const
  {
    return sdf_.getDistanceGradientAt(position);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::pair<scalar_t, vector3_t> SegmentedPlanesSignedDistanceField::valueAndDerivative(
    const vector3_t& position) const
  {
    return {value(position), derivative(position)};
  }
} // namespace terrain_model