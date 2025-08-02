#include <legged_locomotion_mpc/torque_approx/PinocchioTorqueApproximationCppAd.hpp>


namespace legged_locomotion_mpc 
{
  PinocchioTorqueApproximationCppAd::PinocchioTorqueApproximationCppAd(
    const ocs2::PinocchioInterface& pinocchioInterface,
    const floating_base_model::FloatingBaseModelInfo info,
    const std::string& modelName,
    const std::string& modelFolder = "/tmp/ocs2",
    bool recompileLibraries = true, bool verbose = false): info_(info)
  {

    // torque approximation function
    auto torqueApproxFunc = [&, this](const ad_vector_t& x, ad_vector_t& y) {

      // initialize CppAD interface
      auto pinocchioInterfaceCppAd = pinocchioInterface.toCppAd();
  
      // mapping
      FloatingBaseModelPinocchioMappingCppAd mappingCppAd(info.toCppAd());
      mappingCppAd.setPinocchioInterface(pinocchioInterfaceCppAd);
  
      ocs2::ad_vector_t state = x.head(info.stateDim);
      ocs2::ad_vector_t input = x.tail(info.inputDim);
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


  PinocchioTorqueApproximationCppAd::PinocchioTorqueApproximationCppAd(
    const PinocchioTorqueApproximationCppAd& rhs)
      :torqueApproxCppAdInterfacePtr_(new CppAdInterface(*rhs.torqueApproxCppAdInterfacePtr_)),
      info_(rhs.info_) {}

  PinocchioTorqueApproximationCppAd* PinocchioTorqueApproximationCppAd::clone() const 
  {
    return new PinocchioTorqueApproximationCppAd(*this);
  }

  ocs2::vector_t PinocchioTorqueApproximationCppAd::getValue(
    const ocs2::vector_t& state, const ocs2::vector_t& input) const
  {
    const ocs2::vector_t stateInput = (ocs2::vector_t(state.rows() + input.rows()) << state, input).finished();
    return torqueApproxCppAdInterfacePtr_->getFunctionValue(stateInput);
  }

  ocs2::VectorFunctionLinearApproximation PinocchioTorqueApproximationCppAd::getLinearApproximation(
    const ocs2::vector_t& state, const ocs2::vector_t& input) const
  {
    const ocs2::vector_t stateInput = (ocs2::vector_t(state.rows() + input.rows()) << state, input).finished();
    ocs2::VectorFunctionLinearApproximation approx;
    approx.f = torqueApproxCppAdInterfacePtr_->getFunctionValue(stateInput, disturbance);
    const ocs2::matrix_t torqueApproxJacobian = torqueApproxCppAdInterfacePtr_->getJacobian(stateInput, disturbance);
    approx.dfdx = torqueApproxJacobian.leftCols(state.rows());
    approx.dfdu = torqueApproxJacobian.middleCols(state.rows(), input.rows());
    return approx;
  }


  ocs2::ad_vector_t PinocchioTorqueApproximationCppAd::getValueCppAd(
    ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
    const ocs2::PinocchioStateInputMapping<ocs2::ad_scalar_t>& mapping,
    const ocs2::ad_vector_t& state,
    const ocs2::ad_vector_t& input)
  {
    const auto& infoCppAd = info_.toCppAd();

    const ad_vector_t q = mapping.getPinocchioJointPosition(state);

    const auto& model = pinocchioInterfaceCppAd.getModel();
    auto& data = pinocchioInterfaceCppAd.getData();

    pinocchio::forwardKinematics(model, data, q);

    using Force = pinocchio::ForceTpl<ocs2::scalar_t, 0>;
    Force force;
    force.linear() = Eigen::Vector<ocs2::scalar_t, 3>::Zero();
    force.angular() = Eigen::Vector<ocs2::scalar_t, 3>::Zero();
    pinocchio::container::aligned_vector<Force> fext(model.njoints, force);

    using floating_base_model::model_helper_functions;
    computeSpatialForces(pinocchioInterfaceCppAd, infoCppAd, input, fext);

    return computeApproxActuatedJointGeneralizedTorques(pinocchioInterfaceCppAd, 
      infoCppAd, q, fext);
  }



} // namespace legged_locomotion_mpc