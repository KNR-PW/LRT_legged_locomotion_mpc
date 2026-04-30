//
// Created by rgrandia on 14.08.20.
// Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2) on 29.08.2025 
//

#ifndef __SIGNED_DISTANCE_FIELD_TERRAIN_MODEL__
#define __SIGNED_DISTANCE_FIELD_TERRAIN_MODEL__

#include <terrain_model/core/Types.hpp>

namespace terrain_model
{
  /**
   * This abstract class defines the interface for a signed distance field.
   */
  class SignedDistanceField
  {
    public:
      SignedDistanceField() = default;
      virtual ~SignedDistanceField() = default;
      
      virtual SignedDistanceField* clone() const = 0;

      /** Get closest distance from queried position to terrain model 
       * @param [in] position: 3D query position in world frame
      */
      virtual ocs2::scalar_t value(const vector3_t& position) const = 0;

      /** Get gradient from queried position to terrain model 
       * @param [in] position: 3D query position in world frame
      */
      virtual Eigen::Vector3d derivative(const vector3_t& position) const = 0;

      /** Get closest distance and its gradient from queried position to terrain model 
       * @param [in] position: 3D query position in world frame
      */
      virtual std::pair<ocs2::scalar_t, vector3_t> valueAndDerivative(
        const vector3_t& position) const = 0;
  };
}; // namespace terrain_model

#endif