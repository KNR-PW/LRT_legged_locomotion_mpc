#include <gtest/gtest.h>

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/common/EulerTransforms.hpp>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

using namespace ocs2;
using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::euler_transforms;

const scalar_t tolerance = 1e-9;
const size_t NUM_TEST = 100;

class RotationVectorMultiplicationAD final 
  {
    public:

    RotationVectorMultiplicationAD(
      const std::string& modelFolder = "/tmp/ocs2",
      bool recompileLibraries = true,
      bool verbose = false)
    {
      auto systemFlowMapFunc = [&](const ocs2::ad_vector_t& x, const ocs2::ad_vector_t& p, 
        ocs2::ad_vector_t& y) {
        const Eigen::Matrix<ocs2::ad_scalar_t, 3, 1> eulerAnglesAD = x;
        const Eigen::Matrix<ocs2::ad_scalar_t, 3, 1> vector = p;
        y = getValueCppAd(eulerAnglesAD, vector);
      };
    
      systemFlowMapCppAdInterfacePtr_.reset(
          new ocs2::CppAdInterface(systemFlowMapFunc, 3, 3, "euler_to_matrix_systemFlowMap", modelFolder));
    
      if (recompileLibraries) {
        systemFlowMapCppAdInterfacePtr_->createModels(ocs2::CppAdInterface::ApproximationOrder::First, verbose);
      } else {
        systemFlowMapCppAdInterfacePtr_->loadModelsIfAvailable(ocs2::CppAdInterface::ApproximationOrder::First, verbose);
      }
    };

    vector3_t getValue(vector3_t euler, vector3_t vector) const
    {
      const ocs2::vector_t x = (ocs2::vector_t(3) << euler).finished();
      const ocs2::vector_t p = (ocs2::vector_t(3) << vector).finished();
      const auto value = systemFlowMapCppAdInterfacePtr_->getFunctionValue(x, p);
      const vector3_t resultVector = vector3_t::Map(value.data());
      return resultVector;
    };

    matrix3_t getLinearApproximation(vector3_t euler, 
      vector3_t vector) const
    {
      const ocs2::vector_t x = (ocs2::vector_t(3) << euler).finished();
      const ocs2::vector_t p = (ocs2::vector_t(3) << vector).finished();
      const ocs2::matrix_t dynamicsJacobian = systemFlowMapCppAdInterfacePtr_->getJacobian(x, p);
      const matrix3_t approx = matrix3_t::Map(dynamicsJacobian.data());

      return approx;
    };

   private:

   ocs2::ad_vector_t getValueCppAd(const Eigen::Matrix<ocs2::ad_scalar_t, 3, 1>& eulerAnglesAD, 
    const Eigen::Matrix<ocs2::ad_scalar_t, 3, 1>& vector)
    {
      const Eigen::Matrix<ocs2::ad_scalar_t, 3, 3> rotationMatrix = getRotationMatrixFromZyxEulerAngles(
        eulerAnglesAD);
      return rotationMatrix * vector;
    };

    std::unique_ptr<ocs2::CppAdInterface> systemFlowMapCppAdInterfacePtr_;
};




TEST(ModelHelperFunctions, getRotationMatrixEulerZyxGradient)
{
  for(size_t i = 0; i < NUM_TEST; ++i)
  {
    const vector3_t vector = vector3_t::Random();
    const vector3_t euler = vector3_t::Random();

    const auto analyticalMatrix = getRotationMatrixFromZyxEulerAngles(euler);

    const auto analyticalGradient = getRotationMatrixEulerZyxGradient(euler);

    matrix3_t analyticalResult;

    for(size_t j = 0; j < 3; ++j)
    {
      analyticalResult.row(j) = (analyticalGradient[j][0] * vector[0] + 
        analyticalGradient[j][1] * vector[1] + analyticalGradient[j][2] * vector[2]).transpose();
    }

    const auto adMultiplication = RotationVectorMultiplicationAD();
    const auto adResult = adMultiplication.getLinearApproximation(euler, vector);
    const auto adValue = adMultiplication.getValue(euler, vector);
    
    EXPECT_TRUE((analyticalMatrix * vector - adValue).norm() < tolerance);
    EXPECT_TRUE((adResult - analyticalResult).norm() < tolerance);
  }
}