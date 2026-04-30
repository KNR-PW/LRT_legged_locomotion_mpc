//
// Created by rgrandia on 17.03.22.
// Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2) on 30.04.2026 
//

#ifndef __SEGMENTED_PLANES_SIGNED_DISTANCE_FIELD_TERRAIN_MODEL__
#define __SEGMENTED_PLANES_SIGNED_DISTANCE_FIELD_TERRAIN_MODEL__

#include <terrain_model/core/Types.hpp>
#include <terrain_model/core/SignedDistanceField.hpp>

#include <grid_map_sdf/SignedDistanceField.hpp>

namespace terrain_model
{
  /**
   * Simple wrapper class to implement the switched_model::SignedDistanceField
   * interface. See the forwarded function for documentation.
   */
  class SegmentedPlanesSignedDistanceField : public SignedDistanceField 
  {
    public:

      SegmentedPlanesSignedDistanceField(const grid_map::GridMap& gridMap,
        const std::string& elevationLayer, ocs2::scalar_t minHeight, ocs2::scalar_t maxHeight);

      ~SegmentedPlanesSignedDistanceField() override = default;
      
      SegmentedPlanesSignedDistanceField* clone() const override;

      ocs2::scalar_t value(const vector3_t& position) const override;

      vector3_t derivative(const vector3_t& position) const override;

      std::pair<ocs2::scalar_t, vector3_t> valueAndDerivative(
        const vector3_t& position) const override;
      
    private:
      grid_map::SignedDistanceField sdf_;
  };
} // namespace terrain_model

#endif