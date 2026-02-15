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

#include <legged_locomotion_mpc/collision/CollisionSettings.hpp>

#include <stdexcept>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>


namespace legged_locomotion_mpc
{
  namespace collision
  {
    using namespace ocs2;

    CollisionSettings loadCollisionSettings(const std::string& filename,
      const ModelSettings& modelSettings, const std::string& fieldName,bool verbose)
    {

      CollisionSettings settings;

      boost::property_tree::ptree pt;
      read_info(filename, pt);

      if(verbose) 
      {
        std::cerr << "\n #### Legged Locomotion MPC Collision Settings:";
        std::cerr << "\n #### =============================================================================\n";
      }

      loadData::loadStdVector(filename, fieldName + ".collisionLinkNames", settings.collisionLinkNames, verbose);

      std::vector<std::string> collisionNames = modelSettings.contactNames3DoF;
      collisionNames.insert(modelSettings.contactNames3DoF.end(), 
        modelSettings.contactNames6DoF.begin(), modelSettings.contactNames6DoF.end());


      // Get all collision link names without end effector names
      std::vector<std::string> trueCollisionLinksNames;

      for(const auto& collisionLink: settings.collisionLinkNames)
      {
        if(std::find(collisionNames.cbegin(), collisionNames.cend(), collisionLink) != collisionNames.cend())
        {
          continue;
        }
        else
        {
          collisionNames.push_back(collisionLink);
          trueCollisionLinksNames.push_back(collisionLink);
        }
      }

      settings.collisionLinkNames = trueCollisionLinksNames;

      loadData::loadStdVector(filename, fieldName + ".terrainCollisionLinkNames", settings.terrainCollisionLinkNames,verbose);
      
      for(const auto& terrainCollisionLink: settings.terrainCollisionLinkNames)
      {
        if(std::find(collisionNames.cbegin(), collisionNames.cend(), terrainCollisionLink) != collisionNames.cend())
        {
          std::string message = "[CollisionSettings]: No " + terrainCollisionLink + " link in all collison links!";
          throw std::invalid_argument(message);
        }
      }

      loadData::loadStdVectorOfPair(filename, fieldName + ".selfCollisionPairNames", settings.selfCollisionPairNames, verbose);

      for(size_t i = 0; i < settings.selfCollisionPairNames.size(); ++i)
      {
        const auto& [firstCollisionName, secondCollisionName] = settings.selfCollisionPairNames[i];

        if(firstCollisionName == secondCollisionName)
        {
          throw std::invalid_argument("[CollisionSettings]: Same name for first and second collision link!");
        }

        if(std::find(collisionNames.cbegin(), collisionNames.cend(), firstCollisionName) == collisionNames.cend())
        {
          throw std::invalid_argument("[CollisionSettings]: First collision link not found in collision link names!");
        }

        if(std::find(collisionNames.cbegin(), collisionNames.cend(), secondCollisionName) == collisionNames.cend())
        {
          throw std::invalid_argument("[CollisionSettings]: Second collision link not found in collision link names!");
        }
      }

      loadData::loadStdVector(filename, fieldName + ".maxExcesses", settings.maxExcesses, verbose);

      const size_t collisionSize = collisionNames.size();

      if(settings.maxExcesses.size() != collisionSize)
      {
        throw std::invalid_argument("[CollisionSettings]: Max excesses not equal all collision size!");
      }

      loadData::loadStdVector(filename, fieldName + ".relaxations", settings.relaxations, verbose);

      if(settings.relaxations.size() != collisionSize)
      {
        throw std::invalid_argument("[CollisionSettings]: Relaxations not equal all collision size!");
      }

      loadData::loadPtreeValue(pt, settings.shrinkRatio, fieldName + ".shrinkRatio", verbose);

      if(settings.shrinkRatio <= 0.0 || settings.shrinkRatio >= 1.0)
      {
        throw std::invalid_argument("[CollisionSettings]: Shrink ratio must be between (0.0, 1.0)!");
      }

      if(verbose) 
      {
        std::cerr << " #### =============================================================================" <<
        std::endl;
      }

      return settings;
    }
  } // namespace collision
  
} // namespace legged_locomotion_mpc
