//
// Created by rgrandia on 08.09.22.
// Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2) on 29.08.2025 
//

#ifndef __PLANAR_SIGNED_DISTANCE_FIELD_TERRAIN_MODEL__
#define __PLANAR_SIGNED_DISTANCE_FIELD_TERRAIN_MODEL__

#include <terrain_model/core/Types.hpp>
#include <terrain_model/core/SignedDistanceField.hpp>
#include <terrain_model/core/TerrainPlane.hpp>

namespace terrain_model
{
  /**
   * Implements a flat terrain signed distance field
   */
  class PlanarSignedDistanceField : public SignedDistanceField
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      PlanarSignedDistanceField(TerrainPlane terrainPlane);
      ~PlanarSignedDistanceField() override = default;
      
      PlanarSignedDistanceField* clone() const override;
      
      ocs2::scalar_t value(const vector3_t& position) const override;

      vector3_t derivative(const vector3_t& position) const override;

      std::pair<ocs2::scalar_t, vector3_t> valueAndDerivative(
        const vector3_t& position) const override;

    private:
      TerrainPlane terrainPlane_;
  };
}; // namespace terrain_model

#endif