#include <legged_locomotion_mpc/common/ModelSettings.hpp>

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

    if(verbose) 
    {
      std::cerr << " #### =============================================================================" <<
      std::endl;
    }

    return modelSettings;
  }
}
