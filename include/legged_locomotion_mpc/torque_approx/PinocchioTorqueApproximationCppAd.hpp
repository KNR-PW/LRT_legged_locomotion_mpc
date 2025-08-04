#ifndef __PINOCCHIO_TORQUE_APPROXIMATION_CPP_AD__
#define __PINOCCHIO_TORQUE_APPROXIMATION_CPP_AD__


#include <floating_base_model/FloatingBaseModelInfo.hpp>
#include <floating_base_model/AccessHelperFunctions.hpp>
#include <floating_base_model/ModelHelperFunctions.hpp>

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/common/ModelHelperFunctions.hpp>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>


#include <functional>
#include <string>
#include <vector>

namespace legged_locomotion_mpc 
{

  /**
   * This class provides the CppAD implementation of the torque approximation
   * based on pinocchio. No pre-computation is required. 
   */
  class PinocchioTorqueApproximationCppAd
  {
    public:

    /** Constructor
     * @param [in] pinocchioInterface: pinocchio interface.
     * @param [in] mapping: mapping from OCS2 to pinocchio state.
     * @param [in] info: info of kinematics model
     * @param [in] modelName: name of the generate model library
     * @param [in] modelFolder: folder to save the model library files to
     * @param [in] recompileLibraries: If true, the model library will be newly
     * compiled. If false, an existing library will be loaded if available.
     * @param [in] verbose : print information.
     */
    PinocchioTorqueApproximationCppAd(
      const ocs2::vector& torqueDynamicsError,
      const ocs2::PinocchioInterface& pinocchioInterface,
      const ocs2::PinocchioStateInputMapping<ocs2::ad_scalar_t>& mapping,
      const floating_base_model::FloatingBaseModelInfo info,
      const std::string& modelName,
      const std::string& modelFolder = "/tmp/ocs2",
      bool recompileLibraries = true, bool verbose = false);

    ~PinocchioTorqueApproximationCppAd() = default;

    PinocchioTorqueApproximationCppAd* clone() const;

    PinocchioTorqueApproximationCppAd& operator =(
      const PinocchioTorqueApproximationCppAd&) = delete;

    ocs2::vector_t getValue(const ocs2::vector_t& state, const ocs2::vector_t& input) const;

    ocs2::VectorFunctionLinearApproximation getLinearApproximation(const ocs2::vector_t& state,
      const ocs2::vector_t& input) const;

    std::tuple<ocs2::matrix_t, ocs2::matrix_t, ocs2::matrix_t> getWeightedHessians(
      const ocs2::vector_t& weights,
      const ocs2::vector_t& state,
      const ocs2::vector_t& input) const;

   private:
    PinocchioTorqueApproximationCppAd(
        const PinocchioTorqueApproximationCppAd& rhs);

    ocs2::ad_vector_t getValueCppAd(ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
      const ocs2::PinocchioStateInputMapping<ocs2::ad_scalar_t>& mapping,
      const ocs2::ad_vector_t& state,
      const ocs2::ad_vector_t& input);

    std::unique_ptr<ocs2::CppAdInterface> torqueApproxCppAdInterfacePtr_;

    const ocs2::vector torqueDynamicsError_;

    const floating_base_model::FloatingBaseModelInfo info_;
  };

}; // namespace legged_locomotion_mpc

#endif