#include <legged_locomotion_mpc/dynamics/LeggedDynamicsAD.hpp>


namespace legged_locmomtion_mpc
{
  using namespace ocs2;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  LeggedDynamicsAD::LeggedDynamicsAD(const PinocchioInterface &pinocchioInterface,
    const floating_base_model::FloatingBaseModelInfo &info,
    const std::string &modelName,
    const ModelSettings &modelSettings,
    const LeggedSynchronizedModule& synchronizedModule)
      : SystemDynamicsBase(), 
        dynamicsAd_(pinocchioInterface, info, modelName, modelSettings.modelFolderCppAd,
                    modelSettings.recompileLibrariesCppAd, modelSettings.verboseCppAd),
        leggedSynchronizedModule_(&synchronizedModule)
  {

  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector_t LeggedDynamicsAD::computeFlowMap(scalar_t time, const vector_t &state,
    const vector_t &input, const PreComputation &preComp)
  {
    const auto& floatingBaseDisturbance = leggedSynchronizedModule_.getActiveFloatingBaseDisturbance();
    return dynamicsAd_.getValue(time, state, input, floatingBaseDisturbance);

  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  VectorFunctionLinearApproximation LeggedDynamicsAD::linearApproximation(scalar_t time,
    const vector_t &state, const vector_t &input, const PreComputation &preComp)
  {
    const auto& floatingBaseDisturbance = leggedSynchronizedModule_.getActiveFloatingBaseDisturbance();
    return dynamicsAd_.ggetLinearApproximation(time, state, input, floatingBaseDisturbance);
  }

} // namespace legged_locmomtion_mpc