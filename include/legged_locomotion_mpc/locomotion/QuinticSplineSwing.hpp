//
// Created by rgrandia on 21.01.21.
// Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2) on 01.09.2025 
//

#ifndef __QUINTIC_SPLINE_SWING_LEGGED_LOCOMOTION_MPC__
#define __QUINTIC_SPLINE_SWING_LEGGED_LOCOMOTION_MPC__

#include <legged_locomotion_mpc/common/Types.hpp>

namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    struct SwingNode 
    {
      ocs2::scalar_t time;
      ocs2::scalar_t position;
      ocs2::scalar_t velocity;
    };

    /**
     * Implements a quintic polynomial:
     *  y = c_5 * tau^5 + ... + c_1 * tau + c_0,
     *  where tau = (t - t0) / dt
     */
    class QuinticSpline 
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /** Default constructor creates a flat spline, y(t) = 0.0 for t in [-1e30, 1e30]*/
        QuinticSpline();

        /**
         * Constructs a quintic spline. See class description for the definition.
         *
         * @param coefficients : spline coefficients [c_5, ..., c_0]
         * @param t0 : start time of the spline.
         * @param dt : time normalization factor. The normalized time is 1.0 at t = t0 + dt
         */
        QuinticSpline(const vector6_t &coefficients, ocs2::scalar_t t0, ocs2::scalar_t dt);

        /** returns y(t) */
        ocs2::scalar_t position(ocs2::scalar_t time) const;

        /** returns dy/dt(t) */
        ocs2::scalar_t velocity(ocs2::scalar_t time) const;

        /** returns d2y/dt2(t) */
        ocs2::scalar_t acceleration(ocs2::scalar_t time) const;

        /** returns d3y/dt3(t) */
        ocs2::scalar_t jerk(ocs2::scalar_t time) const;

      private:
        ocs2::scalar_t normalizedTime(ocs2::scalar_t t) const;

        /// Coefficients [c_5, ..., c_0] for normalized time
        vector6_t c_;

        /// Start time of the spline
        ocs2::scalar_t t0_;

        /// Spline time normalization
        ocs2::scalar_t dt_;
    };

    /**
     * Implements a swing trajectory based on (at least 2) quintic splines
     * The trajectory is made with the following conditions
     *  start:
     *      position = given
     *      velocity = given
     *      acceleration = 0.0
     *  Mid (can be multiple):
     *      position = given & continuous
     *      velocity = given & continuous
     *      acceleration = continuous
     *      jerk = continuous
     *  End:
     *      position = given
     *      velocity = given
     *      acceleration = 0.0
     */
    class QuinticSwing 
    {
      public:
        /**
         * trajectory of constant zero
         */
        QuinticSwing();

        /**
         * Construct a swing trajectory with given boundary conditions
         *
         * @param start : starting time and boundary conditions
         * @param midHeight : desired height at the middle of the trajectory. Assumes zero velocity.
         * @param end : ending time and boundary conditions
         */
        QuinticSwing(const SwingNode &start, ocs2::scalar_t midHeight, const SwingNode &end);

        /**
         * Construct a swing trajectory with given boundary conditions
         *
         * @param start : starting time and boundary conditions
         * @param mid : waypoint time, position, and velocity. Time must be between start and end.
         * @param end : ending time and boundary conditions
         */
        QuinticSwing(const SwingNode &start, const SwingNode &mid, const SwingNode &end);

        /**
         * Construct a swing trajectory with through the given nodes. At least 3 need to be passed (such that there are at least 2 splines)
         *
         * @param nodes : each node is a waypoint in time, position, and velocity. Vector needs to be sorted in time.
         */
        explicit QuinticSwing(const std::vector<SwingNode> &nodes);

        /** returns z(t) */
        ocs2::scalar_t position(ocs2::scalar_t time) const;

        /** returns dz/dt(t) */
        ocs2::scalar_t velocity(ocs2::scalar_t time) const;

        /** returns d2z/dt2(t) */
        ocs2::scalar_t acceleration(ocs2::scalar_t time) const;

        /** returns d3z/dt3(t) */
        ocs2::scalar_t jerk(ocs2::scalar_t time) const;

      private:
        std::vector<ocs2::scalar_t> nodeTimes;
        std::vector<QuinticSpline> splines;
    };
  }; // namespace locomotion
}; // namespace legged_locomotion_mpc

#endif