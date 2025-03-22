#include "legged_locomotion_mpc/common/ModelSettings.hpp"

namespace legged_locomotion_mpc
{
  ModelSettings loadModelSettings(const std::string &filename,
    const std::string &fieldName,
    bool verbose)
  {
    ModelSettings modelSettings;

    boost::property_tree::ptree pt;
    read_info(filename, pt);

    if (verbose) 
    {
      std::cerr << "\n #### Legged Locomotion MPC Model Settings:";
      std::cerr << "\n #### =============================================================================\n";
    }

    ocs2::loadData::loadPtreeValue(pt, modelSettings.verboseCppAd, fieldName + ".verboseCppAd", verbose);

    ocs2::loadData::loadPtreeValue(pt, modelSettings.recompileLibrariesCppAd, fieldName + ".recompileLibrariesCppAd",
                             verbose);

    ocs2::loadData::loadPtreeValue(pt, modelSettings.modelFolderCppAd, fieldName + ".modelFolderCppAd", verbose);
    
    ocs2::loadData::loadPtreeValue(pt, modelSettings.baseLinkName, fieldName + ".baseLinkName", verbose);

    ocs2::loadData::loadStdVector(filename, fieldName + ".contactNames3DoF", modelSettings.contactNames3DoF, verbose);
    
    ocs2::loadData::loadStdVector(filename, fieldName + ".contactNames6DoF", modelSettings.contactNames6DoF, verbose);

    if (verbose) {
      std::cerr << " #### =============================================================================" <<
      std::endl;
    }

    return modelSettings;
  }
}
