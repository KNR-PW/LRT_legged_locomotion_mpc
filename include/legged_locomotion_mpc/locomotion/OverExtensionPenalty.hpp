// Copyright (c) 2025, Bartłomiej Krajewski
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

/*
 * Authors: Bartłomiej Krajewski (https://github.com/BartlomiejK2)
 */

#ifndef __OVER_EXTENSION_PENALTY_LEGGED_LOCOMOTION_MPC__
#define __OVER_EXTENSION_PENALTY_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>
#include <floating_base_model/FloatingBaseModelPinocchioMapping.hpp>

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/common/ModelSettings.hpp>

namespace legged_locomotion_mpc 
{
  namespace locomotion
  {
    /**
     * This class provides over extenstion penalty function object, 
     * required for finding optimal foothold position 
     */
    class OverExtensionPenalty
    {
      public:

      using PenaltyFunction = std::function<ocs2::scalar_t(const vector3_t &)>;

      struct Settings
      {
        /** Leg extension beyond this length [m] will be penalized in terrain selection */
        ocs2::scalar_t nominalLegExtension = 0.35;

        /** Weight of the end effector overextension penalty */
        ocs2::scalar_t legOverExtensionWeight = 5.0; 
      };

      /** Constructor
       * @param [in] pinocchioInterface: pinocchio interface.
       * @param [in] modelSettings: model settings
       * @param [in] penaltySettings: penalty settings
       * @param [in] info: info of Floating base model
       * @param [in] modelName: name of the generate model library
       * @param [in] modelFolder: folder to save the model library files to
       * @param [in] recompileLibraries: If true, the model library will be newly
       * compiled. If false, an existing library will be loaded if available.
       * @param [in] verbose: print information.
       */
      OverExtensionPenalty(
        const ocs2::PinocchioInterface& pinocchioInterface,
        const ModelSettings& modelSettings,
        const Settings penaltySettings,
        const floating_base_model::FloatingBaseModelInfo& info,
        const std::string& modelName,
        const std::string& modelFolder = "/tmp/legged_locomotion_mpc",
        bool recompileLibraries = true, bool verbose = false);

      ~OverExtensionPenalty() = default;
      
      OverExtensionPenalty* clone() const;
      
      OverExtensionPenalty& operator =(
          const OverExtensionPenalty&) = delete;

      std::vector<PenaltyFunction> getPenalties(const ocs2::vector_t& state) const;

      PenaltyFunction getPenalty(size_t endEffectorIndex, const ocs2::vector_t& state) const;

     private:
      OverExtensionPenalty(
        const OverExtensionPenalty& rhs);

      ocs2::ad_vector_t getHipPositionsCppAd(
        ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
        const floating_base_model::FloatingBaseModelPinocchioMappingCppAd& mapping,
        const ocs2::ad_vector_t& state);
      
      std::vector<vector3_t> getHipPositions(const ocs2::vector_t& state) const;

      const Settings settings_;

      size_t hipNum_;
      std::vector<size_t> hipIndices_;
      
      std::unique_ptr<ocs2::CppAdInterface> positionCppAdInterfacePtr_;
      
    };
  } // namespace locomotion
} // namespace legged_locomotion_mpc

#endif