#include <ocs2_core/Types.h>
#include <chrono>
#include <iostream>


class ConstLocalObject
{
  public:
  ocs2::VectorFunctionLinearApproximation get(size_t inputDim, size_t stateDim) const
  {
    ocs2::VectorFunctionLinearApproximation linearApprox;
    linearApprox.f = ocs2::vector_t::Zero(inputDim);
    linearApprox.dfdx = ocs2::matrix_t::Zero(stateDim, stateDim);
    linearApprox.dfdu = ocs2::matrix_t::Zero(inputDim, inputDim);
    linearApprox.f = ocs2::vector_t::Random(inputDim);
    return linearApprox;
  }
};

class NonConstClassObject
{
  public:

  NonConstClassObject(size_t inputDim, size_t stateDim)
  {
    inputDim_ = inputDim;
    a_.f = ocs2::vector_t::Zero(inputDim);
    a_.dfdx = ocs2::matrix_t::Zero(stateDim, stateDim);
    a_.dfdu = ocs2::matrix_t::Zero(inputDim, inputDim);
  }

  ocs2::VectorFunctionLinearApproximation get() 
  {
    a_.f = ocs2::vector_t::Random(inputDim_); 
    return a_;
  }

  private:
  size_t inputDim_;
  ocs2::VectorFunctionLinearApproximation a_;

};

class ConstMutableClassObject
{
  public:

  ConstMutableClassObject(size_t inputDim, size_t stateDim)
  {
    inputDim_ = inputDim;
    a_.f = ocs2::vector_t::Zero(inputDim);
    a_.dfdx = ocs2::matrix_t::Zero(stateDim, stateDim);
    a_.dfdu = ocs2::matrix_t::Zero(inputDim, inputDim);
  }

  ocs2::VectorFunctionLinearApproximation get() const
  {
    a_.f = ocs2::vector_t::Random(inputDim_); 
    return a_;
  }

  private:
  size_t inputDim_;
  mutable ocs2::VectorFunctionLinearApproximation a_;
};


ocs2::VectorFunctionLinearApproximation test_func_1(size_t inputDim, size_t stateDim)
{
  ocs2::VectorFunctionLinearApproximation linearApprox;
  linearApprox.f = ocs2::vector_t::Zero(inputDim);
  linearApprox.dfdx = ocs2::matrix_t::Zero(stateDim, stateDim);
  linearApprox.dfdu = ocs2::matrix_t::Zero(inputDim, inputDim);
  linearApprox.f = ocs2::vector_t::Random(inputDim); 
  return linearApprox;
};


ocs2::VectorFunctionLinearApproximation test_func_2(size_t inputDim, ocs2::VectorFunctionLinearApproximation& a)
{
  a.f = ocs2::vector_t::Random(inputDim); 
  return a;
};

void test_func_3(size_t inputDim, ocs2::VectorFunctionLinearApproximation& a)
{
  a.f = ocs2::vector_t::Random(inputDim); 
};

int main()
{
  size_t inputDim = 100;
  size_t stateDim = 50;
  std::cout << "Type input dimension: " << std::endl;
  std::cin >> inputDim;
  std::cout << "Type state dimension: " << std::endl;
  std::cin >> stateDim;
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  for(size_t i = 0; i < 10000; ++i)
  {
    const auto output = test_func_1(inputDim, stateDim);
  }
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  std::cout << "Time difference: first test = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;

  int firstTime = std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count();

  begin = std::chrono::steady_clock::now();

  ocs2::VectorFunctionLinearApproximation linearApprox;
  linearApprox.f = ocs2::vector_t::Zero(inputDim);
  linearApprox.dfdx = ocs2::matrix_t::Zero(stateDim, stateDim);
  linearApprox.dfdu = ocs2::matrix_t::Zero(inputDim, inputDim);
  for(size_t i = 0; i < 10000; ++i)
  {
    const auto output = test_func_2(inputDim, linearApprox);
  }
  end = std::chrono::steady_clock::now();
  std::cout << "Time difference: second test = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;

  int secondTime = std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count();
  begin = std::chrono::steady_clock::now();

  linearApprox.f = ocs2::vector_t::Zero(inputDim);
  linearApprox.dfdx = ocs2::matrix_t::Zero(stateDim, stateDim);
  linearApprox.dfdu = ocs2::matrix_t::Zero(inputDim, inputDim);
  for(size_t i = 0; i < 10000; ++i)
  {
    test_func_3(inputDim, linearApprox);
  }
  end = std::chrono::steady_clock::now();
  std::cout << "Time difference: third test = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;
  int thirdTime = std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count();

  std::cout << "second/first: " << (double) firstTime / (double) secondTime << std::endl;
  std::cout << "third/first: " << (double) firstTime / (double) thirdTime << std::endl;
  Eigen::VectorXd testVector = Eigen::VectorXd::Random(100);

  Eigen::VectorXd vector = Eigen::VectorXd::Random(100);

  Eigen::MatrixXd testMatrix = Eigen::MatrixXd::Zero(100, 100);
  testMatrix.diagonal() = Eigen::VectorXd::Random(100);

  begin = std::chrono::steady_clock::now();
  for(int i = 0; i < 10000; ++i)
  {
    Eigen::VectorXd result = testVector.asDiagonal() * vector;
  }
  end = std::chrono::steady_clock::now();
  std::cout << "asDiagonal(): " << std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count() << "[us]" << std::endl;
  
  int diagonalTime = std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count();

  begin = std::chrono::steady_clock::now();
  for(int i = 0; i < 10000; ++i)
  {
    Eigen::VectorXd result = testMatrix * vector;
  }
  end = std::chrono::steady_clock::now();
  std::cout << "Matrix: " << std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count() << "[us]" << std::endl;
  
  int matrixTime = std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count();
  
  std::cout << "matrix/diagonal: " << (double) matrixTime / (double) diagonalTime << std::endl;
  
  ConstLocalObject constLocalObject;

  begin = std::chrono::steady_clock::now();

  for(size_t i = 0; i < 10000; ++i)
  {
    const ocs2::VectorFunctionLinearApproximation output = constLocalObject.get(inputDim, stateDim);
  }
  end = std::chrono::steady_clock::now();

  int constLocalTime = std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count();

  NonConstClassObject nonConstClassObject(inputDim, stateDim);

  begin = std::chrono::steady_clock::now();

  for(size_t i = 0; i < 10000; ++i)
  {
    const ocs2::VectorFunctionLinearApproximation output = nonConstClassObject.get();
  }
  end = std::chrono::steady_clock::now();

  int nonConstClassTime = std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count();


  ConstMutableClassObject constMutableClassObject(inputDim, stateDim);

  begin = std::chrono::steady_clock::now();

  for(size_t i = 0; i < 10000; ++i)
  {
    const ocs2::VectorFunctionLinearApproximation output = constMutableClassObject.get();
  }
  end = std::chrono::steady_clock::now();

  int constMutableClassTime = std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count();


  std::cout << "constLocalObject/nonConstClassObject: " << (double) constLocalTime / (double) nonConstClassTime << std::endl;
  std::cout << "constLocalObject/mutableConstClassObject: " << (double) constLocalTime / (double) constMutableClassTime << std::endl;
  return 0;


}
