#ifndef __PINOCCHIO_6DOF_END_EFFECTOR_KINEMATICS_CPP_AD__
#define __PINOCCHIO_6DOF_END_EFFECTOR_KINEMATICS_CPP_AD__


#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>


#include <floating_base_model/QuaterionEulerTransforms.hpp>

#include <legged_locomotion_mpc/common/Types.hpp>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>


#include <functional>
#include <string>
#include <vector>

namespace legged_locomotion_mpc 
{

  /**
   * This class provides the CppAD implementation the 6 DoF end-effector Kinematics
   * based on pinocchio. No pre-computation is required. The class has two
   * constructors. The constructor with an additional argument, "updateCallback",
   * is meant for cases where PinocchioStateInputMapping requires some extra
   * update calls on PinocchioInterface, such as the centroidal model mapping
   * (refer to CentroidalModelPinocchioMapping).
   *
   * See also PinocchioEndEffectorKinematics, which uses analytical computation
   * and caching.
   */
  class Pinocchio6DofEndEffectorKinematicsCppAd
  {
    public:
    using update_pinocchio_interface_callback = std::function<void(
      const ocs2::ad_vector_t& state,
      ocs2::PinocchioInterfaceTpl<ocs2::ad_scalar_t>& pinocchioInterface)>;

    /** Constructor
     * @param [in] pinocchioInterface pinocchio interface.
     * @param [in] mapping mapping from OCS2 to pinocchio state.
     * @param [in] endEffectorIds array of end effector names.
     * @param [in] stateDim : size of state vector
     * @param [in] inputDim : size of input vector
     * @param [in] modelName : name of the generate model library
     * @param [in] modelFolder : folder to save the model library files to
     * @param [in] recompileLibraries : If true, the model library will be newly
     * compiled. If false, an existing library will be loaded if available.
     * @param [in] verbose : print information.
     */
    Pinocchio6DofEndEffectorKinematicsCppAd(
        const ocs2::PinocchioInterface& pinocchioInterface,
        const ocs2::PinocchioStateInputMapping<ocs2::ad_scalar_t>& mapping,
        std::vector<std::string> endEffectorIds, size_t stateDim, size_t inputDim,
        const std::string& modelName,
        const std::string& modelFolder = "/tmp/ocs2",
        bool recompileLibraries = true, bool verbose = false);

    /** Constructor
     * @param [in] pinocchioInterface pinocchio interface.
     * @param [in] mapping mapping from OCS2 to pinocchio state.
     * @param [in] endEffectorIds array of end effector names.
     * @param [in] stateDim : size of state vector
     * @param [in] inputDim : size of input vector
     * @param [in] updateCallback : In the cases that PinocchioStateInputMapping
     * requires some additional update calls on PinocchioInterface, use this
     * callback.
     * @param [in] modelName : name of the generate model library
     * @param [in] modelFolder : folder to save the model library files to
     * @param [in] recompileLibraries : If true, the model library will be newly
     * compiled. If false, an existing library will be loaded if available.
     * @param [in] verbose : print information.
     */
    Pinocchio6DofEndEffectorKinematicsCppAd(
        const ocs2::PinocchioInterface& pinocchioInterface,
        const ocs2::PinocchioStateInputMapping<ocs2::ad_scalar_t>& mapping,
        std::vector<std::string> endEffectorIds, size_t stateDim, size_t inputDim,
        update_pinocchio_interface_callback updateCallback,
        const std::string& modelName,
        const std::string& modelFolder = "/tmp/ocs2",
        bool recompileLibraries = true, bool verbose = false);

    ~Pinocchio6DofEndEffectorKinematicsCppAd() = default;
    Pinocchio6DofEndEffectorKinematicsCppAd* clone() const;
    Pinocchio6DofEndEffectorKinematicsCppAd& operator =(
        const Pinocchio6DofEndEffectorKinematicsCppAd&) = delete;

    const std::vector<std::string>& getIds() const;

    std::vector<vector3_t> getPosition(const ocs2::vector_t& state) const;

    std::vector<vector3_t> getOrientation(const ocs2::vector_t& state) const;

    std::vector<vector3_t> getLinearVelocity(
      const ocs2::vector_t& state, const ocs2::vector_t& input) const;

      std::vector<vector3_t> getAngularVelocity(
        const ocs2::vector_t& state, const ocs2::vector_t& input) const;

    std::vector<ocs2::VectorFunctionLinearApproximation> getPositionLinearApproximation(
        const ocs2::vector_t& state) const;

    std::vector<ocs2::VectorFunctionLinearApproximation> getOrientationLinearApproximation(
      const ocs2::vector_t& state) const;

    std::vector<ocs2::VectorFunctionLinearApproximation> getLinearVelocityLinearApproximation(
        const ocs2::vector_t& state, const ocs2::vector_t& input) const;

    std::vector<ocs2::VectorFunctionLinearApproximation> getAngularVelocityLinearApproximation(
      const ocs2::vector_t& state, const ocs2::vector_t& input) const;

   private:
    Pinocchio6DofEndEffectorKinematicsCppAd(
        const Pinocchio6DofEndEffectorKinematicsCppAd& rhs);

    ocs2::ad_vector_t getPositionCppAd(
      ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
        const ocs2::PinocchioStateInputMapping<ocs2::ad_scalar_t>& mapping,
        const ocs2::ad_vector_t& state);
    
    ocs2::ad_vector_t getOrientationCppAd(
      ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
        const ocs2::PinocchioStateInputMapping<ocs2::ad_scalar_t>& mapping,
        const ocs2::ad_vector_t& state);


    ocs2::ad_vector_t getLinearVelocityCppAd(
      ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
        const ocs2::PinocchioStateInputMapping<ocs2::ad_scalar_t>& mapping,
        const ocs2::ad_vector_t& state, const ocs2::ad_vector_t& input);
    
    ocs2::ad_vector_t getAngularVelocityCppAd(
      ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
        const ocs2::PinocchioStateInputMapping<ocs2::ad_scalar_t>& mapping,
        const ocs2::ad_vector_t& state, const ocs2::ad_vector_t& input);


    std::unique_ptr<ocs2::CppAdInterface> positionCppAdInterfacePtr_;
    std::unique_ptr<ocs2::CppAdInterface> orientationCppAdInterfacePtr_;

    std::unique_ptr<ocs2::CppAdInterface> linearVelocityCppAdInterfacePtr_;
    std::unique_ptr<ocs2::CppAdInterface> angularVelocityCppAdInterfacePtr_;

    const std::vector<std::string> endEffectorIds_;
    std::vector<size_t> endEffectorFrameIds_;
  };

};  // namespace legged_locomotion_mpc

#endif