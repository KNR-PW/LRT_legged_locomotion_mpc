#include <legged_locomotion_mpc/torque_approx/PinocchioTorqueApproximationCppAd.hpp>


namespace legged_locomotion_mpc 
{

  using namespace ocs2;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PinocchioTorqueApproximationCppAd::PinocchioTorqueApproximationCppAd(
    const vector& torqueDynamicsError,
    const PinocchioInterface& pinocchioInterface,
    const floating_base_model::FloatingBaseModelInfo info,
    const std::string& modelName,
    const std::string& modelFolder = "/tmp/ocs2",
    bool recompileLibraries = true, bool verbose = false): info_(info), torqueDynamicsError_(torqueDynamicsError)
  {
    const auto& model = pinocchioInterface.getModel();
    assert(torqueDynamicsError_.rows() == (info.actuatedDofNum));
    
    // torque approximation function
    auto torqueApproxFunc = [&, this](const ad_vector_t& x, ad_vector_t& y) {

      // initialize CppAD interface
      auto pinocchioInterfaceCppAd = pinocchioInterface.toCppAd();
  
      // mapping
      FloatingBaseModelPinocchioMappingCppAd mappingCppAd(info.toCppAd());
      mappingCppAd.setPinocchioInterface(pinocchioInterfaceCppAd);
  
      ad_vector_t state = x.head(info.stateDim);
      ad_vector_t input = x.tail(info.inputDim);
      y = getValueCppAd(pinocchioInterfaceCppAd, mappingCppAd, state, input);
    };
    torqueApproxCppAdInterfacePtr_.reset(new CppAdInterface(torqueApproxFunc,
      info_.stateDim, modelName + "_torque_approx", modelFolder));

    if (recompileLibraries) {
      torqueApproxCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
    } else {
      torqueApproxCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PinocchioTorqueApproximationCppAd::PinocchioTorqueApproximationCppAd(
    const PinocchioTorqueApproximationCppAd& rhs)
      :torqueApproxCppAdInterfacePtr_(new CppAdInterface(*rhs.torqueApproxCppAdInterfacePtr_)),
      info_(rhs.info_) {}
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PinocchioTorqueApproximationCppAd* PinocchioTorqueApproximationCppAd::clone() const 
  {
    return new PinocchioTorqueApproximationCppAd(*this);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector_t PinocchioTorqueApproximationCppAd::getValue(
    const vector_t& state, const vector_t& input) const
  {
    const vector_t stateInput = (vector_t(state.rows() + input.rows()) << state, input).finished();
    return torqueApproxCppAdInterfacePtr_->getFunctionValue(stateInput) + torqueDynamicsError_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  VectorFunctionLinearApproximation PinocchioTorqueApproximationCppAd::getLinearApproximation(
    const vector_t& state, const vector_t& input) const
  {
    const vector_t stateInput = (vector_t(state.rows() + input.rows()) << state, input).finished();
    VectorFunctionLinearApproximation approx;
    approx.f = torqueApproxCppAdInterfacePtr_->getFunctionValue(stateInput, disturbance);
    const matrix_t torqueApproxJacobian = torqueApproxCppAdInterfacePtr_->getJacobian(stateInput, disturbance);
    approx.dfdx = torqueApproxJacobian.leftCols(state.rows());
    approx.dfdu = torqueApproxJacobian.middleCols(state.rows(), input.rows());
    return approx;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::tuple<matrix_t, matrix_t, matrix_t> PinocchioTorqueApproximationCppAd::getWeightedHessians(
    const vector_t& weights,
    const vector_t& state,
    const vector_t& input) const
  {
    const vector_t stateInput = (vector_t(state.rows() + input.rows()) << state, input).finished();
    const matrix_t hessian = torqueApproxCppAdInterfacePtr_->getHessian(weights, stateInput);
    const matrix_t hessianStateState = hessian.block(0, 0, state.rows(), state.rows());
    const matrix_t hessianInputInput = hessian.block(state.rows(), state.rows(), input.rows(), input.rows());
    const matrix_t hessianInputState = hessian.block(state.rows(), 0, input.rows(), state.rows());
    
    return {hessianStateState, hessianInputInput, hessianInputState};
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ad_vector_t PinocchioTorqueApproximationCppAd::getValueCppAd(
    PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
    const PinocchioStateInputMapping<ad_scalar_t>& mapping,
    const ad_vector_t& state,
    const ad_vector_t& input)
  {
    const auto& infoCppAd = info_.toCppAd();

    const ad_vector_t q = mapping.getPinocchioJointPosition(state);

    const auto& model = pinocchioInterfaceCppAd.getModel();
    auto& data = pinocchioInterfaceCppAd.getData();

    pinocchio::forwardKinematics(model, data, q);

    using Force = pinocchio::ForceTpl<scalar_t, 0>;
    Force force;
    force.linear() = Eigen::Vector<scalar_t, 3>::Zero();
    force.angular() = Eigen::Vector<scalar_t, 3>::Zero();
    pinocchio::container::aligned_vector<Force> fext(model.njoints, force);

    using floating_base_model::model_helper_functions;
    computeSpatialForces(pinocchioInterfaceCppAd, infoCppAd, input, fext);

    return computeApproxActuatedJointGeneralizedTorques(pinocchioInterfaceCppAd, 
      infoCppAd, q, fext);
  }
} // namespace legged_locomotion_mpc