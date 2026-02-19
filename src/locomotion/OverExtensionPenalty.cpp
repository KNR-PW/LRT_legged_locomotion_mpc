#include <legged_locomotion_mpc/locomotion/OverExtensionPenalty.hpp>

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace legged_locomotion_mpc 
{
  namespace locomotion
  {
    using namespace ocs2;
    using namespace floating_base_model;
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    OverExtensionPenalty::OverExtensionPenalty(
      const ocs2::PinocchioInterface& pinocchioInterface, 
      const ModelSettings& modelSettings, const Settings penaltySettings, 
      const floating_base_model::FloatingBaseModelInfo& info,
      const std::string& modelName, const std::string& modelFolder, 
      bool recompileLibraries, bool verbose): 
        settings_(std::move(penaltySettings))
    {
      // Get hip frames
      const auto& model = pinocchioInterface.getModel();

      for(const auto& name : modelSettings.hipFrameNames) 
      {
        const size_t hipFrameIndex = model.getFrameId(name);
        if(hipFrameIndex == model.frames.size())
        {
          throw std::invalid_argument("[OverExtensionPenalty]: Could not find contact frame with name: " + name);
        }
      
        hipIndices_.push_back(hipFrameIndex);
      }

      if(hipIndices_.size() != info.endEffectorFrameIndices.size())
      {
        throw std::invalid_argument("[OverExtensionPenalty]: Wrong amount of hip frames!");
      }
      
      hipNum_ = hipIndices_.size();

      // initialize CppAD interface
      auto pinocchioInterfaceCppAd = pinocchioInterface.toCppAd();
    
      // mapping
      FloatingBaseModelPinocchioMappingCppAd mappingCppAd(info.toCppAd());
      mappingCppAd.setPinocchioInterface(pinocchioInterfaceCppAd);

      // position function
      auto positionFunc = [&, this](const ad_vector_t& x, ad_vector_t& y) 
      {
        y = getHipPositionsCppAd(pinocchioInterfaceCppAd, mappingCppAd, x);
      };
      positionCppAdInterfacePtr_.reset(new CppAdInterface(positionFunc, info.stateDim, modelName + "_position", modelFolder));

      if(recompileLibraries) 
      {
        positionCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::Zero, verbose);
      } 
      else 
      {
        positionCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::Zero, verbose);
      }
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    OverExtensionPenalty::OverExtensionPenalty(const OverExtensionPenalty& rhs)
      :settings_(rhs.settings_), hipNum_(hipNum_), hipIndices_(rhs.hipIndices_),
       positionCppAdInterfacePtr_(new CppAdInterface(*rhs.positionCppAdInterfacePtr_)) {}

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    OverExtensionPenalty* OverExtensionPenalty::clone() const 
    {
      return new OverExtensionPenalty(*this);
    }
    
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    ad_vector_t OverExtensionPenalty::getHipPositionsCppAd(
      PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
      const FloatingBaseModelPinocchioMappingCppAd& mapping,
      const ad_vector_t& state) 
    {
      const auto& model = pinocchioInterfaceCppAd.getModel();
      auto& data = pinocchioInterfaceCppAd.getData();
      const ad_vector_t q = mapping.getPinocchioJointPosition(state);

      pinocchio::forwardKinematics(model, data, q);
      pinocchio::updateFramePlacements(model, data);

      ad_vector_t positions(3 * hipNum_);
      for (size_t i = 0; i < hipNum_; i++) {
        const size_t frameId = hipIndices_[i];
        positions.segment<3>(3 * i) = data.oMf[frameId].translation();
      }
      return positions;
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    auto OverExtensionPenalty::getHipPositions(const vector_t& state) const -> std::vector<vector3_t> 
    {
      const vector_t positionValues = positionCppAdInterfacePtr_->getFunctionValue(state);

      std::vector<vector3_t> positions;
      for (size_t i = 0; i < hipNum_; i++) {
        positions.emplace_back(positionValues.segment<3>(3 * i));
      }
      return positions;
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    using PenaltyFunction = std::function<scalar_t(const vector3_t &)>;
    std::vector<PenaltyFunction> OverExtensionPenalty::getPenalties(const ocs2::vector_t& state) const
    {
      const std::vector<vector3_t> hipPositions = getHipPositions(state);

      std::vector<PenaltyFunction> penalties;

      for(size_t i = 0; i < hipNum_; ++i)
      {
        const vector3_t& hipPosition = hipPositions[i];
        PenaltyFunction penalty = [hipPosition, this](const vector3_t & queryPosition)
        {
          const scalar_t extension = std::max(0.0, 
            (queryPosition - hipPosition).norm() - this->settings_.nominalLegExtension);
          return this->settings_.legOverExtensionWeight * extension * extension;
        };
        penalties.push_back(std::move(penalty));
      }
      return penalties;
    }

    PenaltyFunction OverExtensionPenalty::getPenalty(size_t endEffectorIndex, const ocs2::vector_t& state) const
    {
      assert(endEffectorIndex < hipNum_);
      const vector3_t hipPosition = getHipPositions(state)[endEffectorIndex];
      PenaltyFunction penalty = [hipPosition, this](const vector3_t & queryPosition)
      {
        const scalar_t extension = std::max(0.0, 
          (queryPosition - hipPosition).norm() - this->settings_.nominalLegExtension);
        return this->settings_.legOverExtensionWeight * extension * extension;
      };
      return penalty;
    }
  } // namespace locomotion
} // namespace legged_locomotion_mpc