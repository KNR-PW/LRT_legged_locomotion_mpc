#include <legged_locomotion_mpc/robot_interface/LeggedLoopshapingInterface.hpp>

#include <ocs2_core/loopshaping/LoopshapingPropertyTree.h>

namespace legged_locomotion_mpc
{ 

  using namespace ocs2;
  using namespace terrain_model;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  LeggedLoopshapingInterface::LeggedLoopshapingInterface(
    std::unique_ptr<LeggedInterface> leggedInterfacePtr,
    std::shared_ptr<LoopshapingDefinition> loopshapingDefinitionPtr):
      LoopshapingRobotInterface(std::move(leggedInterfacePtr), 
        std::move(loopshapingDefinitionPtr)) {}
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  LeggedInterface& LeggedLoopshapingInterface::getLeggedInterface()
  {
    return const_cast<LeggedInterface&>(get<LeggedInterface>());
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  LeggedLoopshapingInterface makeLeggedLoopshapingInterface(
    scalar_t initTime, const vector_t& currentState, 
    std::unique_ptr<TerrainModel> currentTerrainModel, const std::string& taskFile, 
    const std::string& modelFile, const std::string& urdfFile, 
    const std::string& loopshapingDefinitionFile)
  {
    auto loopShapingSharedPtr = loopshaping_property_tree::load(loopshapingDefinitionFile);
    std::unique_ptr<LoopshapingDefinition> loopShapingDefinitionPtr(loopShapingSharedPtr.get());

    const vector_t currentSystemState = loopShapingDefinitionPtr->getSystemState(currentState);
    
    std::unique_ptr<LeggedInterface> leggedInterfacePtr = std::make_unique<LeggedInterface>(
      initTime, currentSystemState, std::move(currentTerrainModel), taskFile, modelFile, urdfFile);

    return LeggedLoopshapingInterface(std::move(leggedInterfacePtr), 
      std::move(loopShapingDefinitionPtr));
  }
} // namespace legged_locomotion_mpc