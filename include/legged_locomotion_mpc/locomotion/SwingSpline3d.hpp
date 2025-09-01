//
// Created by rgrandia on 03.08.21.
// Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2) on 01.09.2025 
//

#ifndef __SWING_SPLINE_3D_LEGGED_LOCOMOTION_MPC__
#define __SWING_SPLINE_3D_LEGGED_LOCOMOTION_MPC__

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/locomotion/QuinticSplineSwing.hpp>

namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    struct SwingNode3d 
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      ocs2::scalar_t time;
      vector3_t position;
      vector3_t velocity;
    };

    class SwingSpline3d 
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * Construct a swing trajectory with given boundary conditions
         *
         * @param start : starting time and boundary conditions
         * @param mid : waypoint time, position, and velocity. Time must be between start and end.
         * @param end : ending time and boundary conditions
         */
        SwingSpline3d(const SwingNode3d &start, const SwingNode3d &mid, const SwingNode3d &end);

        /**
         * Construct a swing trajectory through the provided nodes (at least 3)
         */
        explicit SwingSpline3d(const std::vector<SwingNode3d> &nodes);

        /** returns p(t) */
        vector3_t position(ocs2::scalar_t time) const;

        /** returns dp/dt(t) */
        vector3_t velocity(ocs2::scalar_t time) const;

        /** returns d2p/dt2(t) */
        vector3_t acceleration(ocs2::scalar_t time) const;

        /** returns d3p/dt3(t) */
        vector3_t jerk(ocs2::scalar_t time) const;

      private:
        QuinticSwing x_;
        QuinticSwing y_;
        QuinticSwing z_;
    };  
  }; // namespace locomotion
}; //  namespace legged_locomotion_mpc
    
#endif
