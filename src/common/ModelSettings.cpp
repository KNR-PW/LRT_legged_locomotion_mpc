#include "legged_locomotion_mpc/common/ModelSettings.hpp"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace legged_locomotion_mpc
{
  using namespace ocs2;
  ModelSettings loadModelSettings(const std::string& filename,
    const std::string& fieldName,
    bool verbose)
  {
    ModelSettings modelSettings;

    boost::property_tree::ptree pt;
    read_info(filename, pt);

    if(verbose) 
    {
      std::cerr << "\n #### Legged Locomotion MPC Common Model Settings:";
      std::cerr << "\n #### =============================================================================\n";
    }

    loadData::loadPtreeValue(pt, modelSettings.verboseCppAd, fieldName + ".verboseCppAd", verbose);

    loadData::loadPtreeValue(pt, modelSettings.recompileLibrariesCppAd, fieldName + ".recompileLibrariesCppAd", verbose);

    loadData::loadPtreeValue(pt, modelSettings.modelFolderCppAd, fieldName + ".modelFolderCppAd", verbose);
    
    loadData::loadPtreeValue(pt, modelSettings.baseLinkName, fieldName + ".baseLinkName", verbose);

    loadData::loadStdVector(filename, fieldName + ".contactNames3DoF", modelSettings.contactNames3DoF, verbose);
    
    loadData::loadStdVector(filename, fieldName + ".contactNames6DoF", modelSettings.contactNames6DoF, verbose);

    loadData::loadStdVector(filename, fieldName + ".collisionLinkNames", modelSettings.collisionLinkNames, verbose);

    std::vector<std::string> firstCollisionNames;
    std::vector<std::string> secondCollisionNames;
    loadData::loadStdVector(filename, fieldName + ".firstCollisionLinkNames", firstCollisionNames, verbose);
    loadData::loadStdVector(filename, fieldName + ".secondCollisionLinkNames", secondCollisionNames, verbose);

    if(firstCollisionNames.size() != secondCollisionNames.size())
    {
      throw std::invalid_argument("[ModelSettings]: First and second collision link names have to be same length!");
    }

    for(size_t i = 0; i < firstCollisionNames.size(); ++i)
    {
      if(firstCollisionNames[i] == secondCollisionNames[i])
      {
        throw std::invalid_argument("[ModelSettings]: Same name for first and second collision link!");
      }

      if(std::find(modelSettings.collisionLinkNames.cbegin(), modelSettings.collisionLinkNames.cend(), firstCollisionNames[i]) == modelSettings.collisionLinkNames.cend())
      {
        throw std::invalid_argument("[ModelSettings]: First collision link not found in collision link names!");
      }

      if(std::find(modelSettings.collisionLinkNames.cbegin(), modelSettings.collisionLinkNames.cend(), secondCollisionNames[i]) == modelSettings.collisionLinkNames.cend())
      {
        throw std::invalid_argument("[ModelSettings]: Second collision link not found in collision link names!");
      }

      auto collisionPair = std::make_pair(
        firstCollisionNames[i], secondCollisionNames[i]);
      modelSettings.selfCollisionPairNames.push_back(std::move(collisionPair));
    }

    loadData::loadStdVector(filename, fieldName + ".maxExcesses", modelSettings.maxExcesses, verbose);

    const size_t collisionSize = modelSettings.contactNames3DoF.size() 
      + modelSettings.contactNames6DoF.size() + modelSettings.collisionLinkNames.size();

    if(modelSettings.maxExcesses.size() != collisionSize)
    {
      throw std::invalid_argument("[ModelSettings]: Max excesses not equal all collision size!");
    }

    loadData::loadStdVector(filename, fieldName + ".relaxations", modelSettings.relaxations, verbose);

    if(modelSettings.relaxations.size() != collisionSize)
    {
      throw std::invalid_argument("[ModelSettings]: Relaxations not equal all collision size!");
    }

    loadData::loadPtreeValue(pt, modelSettings.shrinkRatio, fieldName + ".shrinkRatio", verbose);
    
    if(modelSettings.shrinkRatio <= 0.0 || modelSettings.shrinkRatio >= 1.0)
    {
      throw std::invalid_argument("[ModelSettings]: Shrink ratio must be between (0.0, 1.0)!");
    }

    if(verbose) 
    {
      std::cerr << " #### =============================================================================" <<
      std::endl;
    }

    return modelSettings;
  }
}
