
#include <legged_locomotion_mpc/dynamics/LeggedDynamicsAD.hpp>

namespace legged_locomotion_mpc
{

  using namespace ocs2;
  using namespace floating_base_model;
  using namespace synchronization;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  LeggedDynamicsAD::LeggedDynamicsAD(const PinocchioInterface &pinocchioInterface,
    const FloatingBaseModelInfo &info,
    const std::string &modelName,
    const ModelSettings &modelSettings,
    const DisturbanceSynchronizedModule& disturbanceSynchronizedModule)
      : SystemDynamicsBase(), 
        dynamicsAdPtr_(pinocchioInterface, info, modelName, modelSettings.modelFolderCppAd,
                    modelSettings.recompileLibrariesCppAd, modelSettings.verboseCppAd),
        disturbanceSynchronizedModule_(&disturbanceSynchronizedModule) {}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector_t LeggedDynamicsAD::computeFlowMap(scalar_t time, const vector_t &state,
    const vector_t &input, const PreComputation &preComp)
  {
    const vector6_t floatingBaseDisturbance = disturbanceSynchronizedModule_->
      getCurrentDisturbance().second;
    return dynamicsAdPtr_.getValue(time, state, input, floatingBaseDisturbance);

  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  VectorFunctionLinearApproximation LeggedDynamicsAD::linearApproximation(scalar_t time,
    const vector_t &state, const vector_t &input, const PreComputation &preComp)
  {
    const vector6_t floatingBaseDisturbance = disturbanceSynchronizedModule_->
      getCurrentDisturbance().second;
    return dynamicsAdPtr_.getLinearApproximation(time, state, input, floatingBaseDisturbance);
  }

} // namespace legged_locomotion_mpc