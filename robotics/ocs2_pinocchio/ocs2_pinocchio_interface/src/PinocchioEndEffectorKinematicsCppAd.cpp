/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <memory>
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace
{
    void defaultUpdatePinocchioInterface(const ocs2::ad_vector_t&, ocs2::PinocchioInterfaceTpl<ocs2::ad_scalar_t>&)
    {
    }
} // unnamed namespace

namespace ocs2
{
    PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd(
        const PinocchioInterface& pinocchioInterface,
        const PinocchioStateInputMapping<ad_scalar_t>& mapping,
        std::vector<std::string> endEffectorIds, size_t stateDim,
        size_t inputDim, const std::string& modelName,
        const std::string& modelFolder, bool recompileLibraries,
        bool verbose)

        : PinocchioEndEffectorKinematicsCppAd(pinocchioInterface, mapping, std::move(endEffectorIds), stateDim,
                                              inputDim,
                                              &defaultUpdatePinocchioInterface, modelName, modelFolder,
                                              recompileLibraries, verbose)
    {
    }

    PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd(
        const PinocchioInterface& pinocchioInterface, const PinocchioStateInputMapping<ad_scalar_t>& mapping,
        std::vector<std::string> endEffectorIds, size_t stateDim, size_t inputDim,
        const update_pinocchio_interface_callback& updateCallback,
        const std::string& modelName, const std::string& modelFolder, bool recompileLibraries, bool verbose)
        : endEffectorIds_(std::move(endEffectorIds))
    {
        for (const auto& bodyName : endEffectorIds_)
        {
            endEffectorFrameIds_.push_back(pinocchioInterface.getModel().getBodyId(bodyName));
        }

        // initialize CppAD interface
        auto pinocchioInterfaceCppAd = pinocchioInterface.toCppAd();

        // set pinocchioInterface to mapping
        std::unique_ptr<PinocchioStateInputMapping<ad_scalar_t>> mappingPtr(mapping.clone());
        mappingPtr->setPinocchioInterface(pinocchioInterfaceCppAd);

        // position function
        auto positionFunc = [&, this](const ad_vector_t& x, ad_vector_t& y)
        {
            updateCallback(x, pinocchioInterfaceCppAd);
            y = getPositionCppAd(pinocchioInterfaceCppAd, *mappingPtr, x);
        };
        positionCppAdInterfacePtr_ = std::make_unique<CppAdInterface>(
            positionFunc, stateDim, modelName + "_position", modelFolder);

        // velocity function
        auto velocityFunc = [&, this](const ad_vector_t& x, ad_vector_t& y)
        {
            const ad_vector_t state = x.head(stateDim);
            const ad_vector_t input = x.tail(inputDim);
            updateCallback(state, pinocchioInterfaceCppAd);
            y = getVelocityCppAd(pinocchioInterfaceCppAd, *mappingPtr, state, input);
        };
        velocityCppAdInterfacePtr_ = std::make_unique<CppAdInterface>(
            velocityFunc, stateDim + inputDim, modelName + "_velocity", modelFolder);

        // orientation function
        auto orientationFunc = [&, this](const ad_vector_t& x, ad_vector_t& y)
        {
            updateCallback(x, pinocchioInterfaceCppAd);
            y = getOrientationCppAd(pinocchioInterfaceCppAd, *mappingPtr, x);
        };
        orientationCppAdInterfacePtr_ = std::make_unique<CppAdInterface>(
            orientationFunc, stateDim, modelName + "_orientation", modelFolder);

        // orientation error function
        auto orientationErrorFunc = [&, this](const ad_vector_t& x, const ad_vector_t& params, ad_vector_t& y)
        {
            updateCallback(x, pinocchioInterfaceCppAd);
            y = getOrientationErrorCppAd(pinocchioInterfaceCppAd, *mappingPtr, x, params);
        };
        orientationErrorCppAdInterfacePtr_.reset(
            new CppAdInterface(orientationErrorFunc, stateDim, 4 * endEffectorFrameIds_.size(),
                               modelName + "_orientationError", modelFolder));

        // orientation error w.r.t. plane function
        auto orientationWrtPlaneFunc = [&, this](const ad_vector_t& x, const ad_vector_t& params, ad_vector_t& y)
        {
            updateCallback(x, pinocchioInterfaceCppAd);
            y = getOrientationErrorWrtPlaneCppAd(pinocchioInterfaceCppAd, *mappingPtr, x, params);
        };
        orientationErrorWrtPlaneCppAdInterfacePtr_.reset(
            new CppAdInterface(orientationWrtPlaneFunc, stateDim, 3 * endEffectorFrameIds_.size(),
                               modelName + "_orientation_wrt_plane", modelFolder));

        // angular velocity function
        auto angularVelocityFunc = [&, this](const ad_vector_t& x, ad_vector_t& y)
        {
            const ad_vector_t state = x.head(stateDim);
            const ad_vector_t input = x.tail(inputDim);
            updateCallback(state, pinocchioInterfaceCppAd);
            y = getAngularVelocityCppAd(pinocchioInterfaceCppAd, *mappingPtr, state, input);
        };
        angularVelocityCppAdInterfacePtr_ = std::make_unique<CppAdInterface>(
            angularVelocityFunc, stateDim + inputDim, modelName + "_angular_velocity", modelFolder);

        // twist function
        auto twistFunc = [&, this](const ad_vector_t& x, ad_vector_t& y)
        {
            const ad_vector_t state = x.head(stateDim);
            const ad_vector_t input = x.tail(inputDim);
            updateCallback(state, pinocchioInterfaceCppAd);
            y = getTwistCppAd(pinocchioInterfaceCppAd, *mappingPtr, state, input);
        };
        twistCppAdInterfacePtr_ = std::make_unique<CppAdInterface>(
            twistFunc, stateDim + inputDim, modelName + "_twist", modelFolder);

        if (recompileLibraries)
        {
            positionCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
            velocityCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
            orientationCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::Zero, verbose);
            orientationErrorCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
            orientationErrorWrtPlaneCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First,
                                                                     verbose);
            angularVelocityCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
            twistCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
        }
        else
        {
            positionCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
            velocityCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
            orientationCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::Zero, verbose);
            orientationErrorCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First,
                                                                      verbose);
            orientationErrorWrtPlaneCppAdInterfacePtr_->loadModelsIfAvailable(
                CppAdInterface::ApproximationOrder::First, verbose);
            angularVelocityCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First,
                                                                     verbose);
            twistCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
        }
    }

    PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd(
        const PinocchioEndEffectorKinematicsCppAd& rhs)
        : EndEffectorKinematics(rhs),
          positionCppAdInterfacePtr_(new CppAdInterface(*rhs.positionCppAdInterfacePtr_)),
          velocityCppAdInterfacePtr_(new CppAdInterface(*rhs.velocityCppAdInterfacePtr_)),
          orientationCppAdInterfacePtr_(new CppAdInterface(*rhs.orientationCppAdInterfacePtr_)),
          orientationErrorCppAdInterfacePtr_(new CppAdInterface(*rhs.orientationErrorCppAdInterfacePtr_)),
          orientationErrorWrtPlaneCppAdInterfacePtr_(
              new CppAdInterface(*rhs.orientationErrorWrtPlaneCppAdInterfacePtr_)),
          angularVelocityCppAdInterfacePtr_(new CppAdInterface(*rhs.angularVelocityCppAdInterfacePtr_)),
          twistCppAdInterfacePtr_(new CppAdInterface(*rhs.twistCppAdInterfacePtr_)),
          endEffectorIds_(rhs.endEffectorIds_),
          endEffectorFrameIds_(rhs.endEffectorFrameIds_)
    {
    }

    PinocchioEndEffectorKinematicsCppAd* PinocchioEndEffectorKinematicsCppAd::clone() const
    {
        return new PinocchioEndEffectorKinematicsCppAd(*this);
    }

    const std::vector<std::string>& PinocchioEndEffectorKinematicsCppAd::getIds() const
    {
        return endEffectorIds_;
    }

    ad_vector_t PinocchioEndEffectorKinematicsCppAd::getPositionCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                                      const PinocchioStateInputMapping<ad_scalar_t>&
                                                                      mapping,
                                                                      const ad_vector_t& state)
    {
        const auto& model = pinocchioInterfaceCppAd.getModel();
        auto& data = pinocchioInterfaceCppAd.getData();
        const ad_vector_t q = mapping.getPinocchioJointPosition(state);

        forwardKinematics(model, data, q);
        updateFramePlacements(model, data);

        ad_vector_t positions(3 * endEffectorFrameIds_.size());
        for (int i = 0; i < endEffectorFrameIds_.size(); i++)
        {
            const size_t frameId = endEffectorFrameIds_[i];
            positions.segment<3>(3 * i) = data.oMf[frameId].translation();
        }
        return positions;
    }

    auto PinocchioEndEffectorKinematicsCppAd::getPosition(const vector_t& state) const -> std::vector<vector3_t>
    {
        const vector_t positionValues = positionCppAdInterfacePtr_->getFunctionValue(state);

        std::vector<vector3_t> positions;
        for (int i = 0; i < endEffectorIds_.size(); i++)
        {
            positions.emplace_back(positionValues.segment<3>(3 * i));
        }
        return positions;
    }

    std::vector<VectorFunctionLinearApproximation> PinocchioEndEffectorKinematicsCppAd::getPositionLinearApproximation(
        const vector_t& state) const
    {
        const vector_t positionValues = positionCppAdInterfacePtr_->getFunctionValue(state);
        const matrix_t positionJacobian = positionCppAdInterfacePtr_->getJacobian(state);

        std::vector<VectorFunctionLinearApproximation> positions;
        for (int i = 0; i < endEffectorIds_.size(); i++)
        {
            VectorFunctionLinearApproximation pos;
            pos.f = positionValues.segment<3>(3 * i);
            pos.dfdx = positionJacobian.block(3 * i, 0, 3, state.rows());
            positions.emplace_back(std::move(pos));
        }
        return positions;
    }

    ad_vector_t PinocchioEndEffectorKinematicsCppAd::getVelocityCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                                      const PinocchioStateInputMapping<ad_scalar_t>&
                                                                      mapping,
                                                                      const ad_vector_t& state,
                                                                      const ad_vector_t& input)
    {
        const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
        const auto& model = pinocchioInterfaceCppAd.getModel();
        auto& data = pinocchioInterfaceCppAd.getData();
        const ad_vector_t q = mapping.getPinocchioJointPosition(state);
        const ad_vector_t v = mapping.getPinocchioJointVelocity(state, input);

        forwardKinematics(model, data, q, v);

        ad_vector_t velocities(3 * endEffectorFrameIds_.size());
        for (int i = 0; i < endEffectorFrameIds_.size(); i++)
        {
            const size_t frameId = endEffectorFrameIds_[i];
            velocities.segment<3>(3 * i) = getFrameVelocity(model, data, frameId, rf).linear();
        }
        return velocities;
    }

    auto PinocchioEndEffectorKinematicsCppAd::getVelocity(const vector_t& state,
                                                          const vector_t& input) const -> std::vector<vector3_t>
    {
        vector_t stateInput(state.rows() + input.rows());
        stateInput << state, input;
        const vector_t velocityValues = velocityCppAdInterfacePtr_->getFunctionValue(stateInput);

        std::vector<vector3_t> velocities;
        for (int i = 0; i < endEffectorIds_.size(); i++)
        {
            velocities.emplace_back(velocityValues.segment<3>(3 * i));
        }
        return velocities;
    }

    std::vector<VectorFunctionLinearApproximation> PinocchioEndEffectorKinematicsCppAd::getVelocityLinearApproximation(
        const vector_t& state, const vector_t& input) const
    {
        vector_t stateInput(state.rows() + input.rows());
        stateInput << state, input;
        const vector_t velocityValues = velocityCppAdInterfacePtr_->getFunctionValue(stateInput);
        const matrix_t velocityJacobian = velocityCppAdInterfacePtr_->getJacobian(stateInput);

        std::vector<VectorFunctionLinearApproximation> velocities;
        for (int i = 0; i < endEffectorIds_.size(); i++)
        {
            VectorFunctionLinearApproximation vel;
            vel.f = velocityValues.segment<3>(3 * i);
            vel.dfdx = velocityJacobian.block(3 * i, 0, 3, state.rows());
            vel.dfdu = velocityJacobian.block(3 * i, state.rows(), 3, input.rows());
            velocities.emplace_back(std::move(vel));
        }
        return velocities;
    }

    auto PinocchioEndEffectorKinematicsCppAd::getOrientationError(const vector_t& state,
                                                                  const std::vector<quaternion_t>&
                                                                  referenceOrientations) const
        -> std::vector<vector3_t>
    {
        vector_t params(4 * endEffectorIds_.size());
        for (int i = 0; i < endEffectorIds_.size(); i++)
        {
            params.segment<4>(4 * i) = referenceOrientations[i].coeffs();
        }

        const vector_t errorValues = orientationErrorCppAdInterfacePtr_->getFunctionValue(state, params);

        std::vector<vector3_t> errors;
        for (int i = 0; i < endEffectorIds_.size(); i++)
        {
            errors.emplace_back(errorValues.segment<3>(3 * i));
        }
        return errors;
    }

    std::vector<VectorFunctionLinearApproximation>
    PinocchioEndEffectorKinematicsCppAd::getOrientationErrorLinearApproximation(
        const vector_t& state, const std::vector<quaternion_t>& referenceOrientations) const
    {
        vector_t params(4 * endEffectorIds_.size());
        for (int i = 0; i < endEffectorIds_.size(); i++)
        {
            params.segment<4>(4 * i) = referenceOrientations[i].coeffs();
        }

        const vector_t errorValues = orientationErrorCppAdInterfacePtr_->getFunctionValue(state, params);
        const matrix_t errorJacobian = orientationErrorCppAdInterfacePtr_->getJacobian(state, params);

        std::vector<VectorFunctionLinearApproximation> errors;
        for (int i = 0; i < endEffectorIds_.size(); i++)
        {
            VectorFunctionLinearApproximation err;
            err.f = errorValues.segment<3>(3 * i);
            err.dfdx = errorJacobian.block(3 * i, 0, 3, state.rows());
            errors.emplace_back(std::move(err));
        }
        return errors;
    }

    ad_vector_t PinocchioEndEffectorKinematicsCppAd::getOrientationErrorCppAd(
        PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
        const PinocchioStateInputMapping<ad_scalar_t>& mapping,
        const ad_vector_t& state, const ad_vector_t& params)
    {
        using ad_quaternion_t = Eigen::Quaternion<ad_scalar_t>;

        const auto& model = pinocchioInterfaceCppAd.getModel();
        auto& data = pinocchioInterfaceCppAd.getData();
        const ad_vector_t q = mapping.getPinocchioJointPosition(state);

        forwardKinematics(model, data, q);
        updateFramePlacements(model, data);

        ad_vector_t errors(3 * endEffectorFrameIds_.size());
        for (int i = 0; i < endEffectorFrameIds_.size(); i++)
        {
            const size_t frameId = endEffectorFrameIds_[i];
            const ad_quaternion_t eeOrientation = matrixToQuaternion(data.oMf[frameId].rotation());
            ad_quaternion_t eeReferenceOrientation;
            eeReferenceOrientation.coeffs() = params.segment<4>(4 * i);
            errors.segment<3>(3 * i) = quaternionDistance(eeOrientation, eeReferenceOrientation);
        }
        return errors;
    }

    ad_vector_t PinocchioEndEffectorKinematicsCppAd::getOrientationCppAd(
        PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
        const PinocchioStateInputMapping<ad_scalar_t>& mapping,
        const ad_vector_t& state)
    {
        const auto& model = pinocchioInterfaceCppAd.getModel();
        auto& data = pinocchioInterfaceCppAd.getData();
        const ad_vector_t q = mapping.getPinocchioJointPosition(state);

        forwardKinematics(model, data, q);
        updateFramePlacements(model, data);

        ad_vector_t orientations(4 * endEffectorFrameIds_.size());
        for (int i = 0; i < endEffectorFrameIds_.size(); i++)
        {
            orientations.segment<4>(4 * i) = matrixToQuaternion(data.oMf[endEffectorFrameIds_[i]].rotation()).coeffs();
        }
        return orientations;
    }

    auto PinocchioEndEffectorKinematicsCppAd::getOrientation(const vector_t& state) const -> std::vector<quaternion_t>
    {
        const vector_t orientationValues = orientationCppAdInterfacePtr_->getFunctionValue(state);

        std::vector<quaternion_t> orientations;
        orientations.reserve(endEffectorIds_.size());
        for (int i = 0; i < endEffectorIds_.size(); i++)
        {
            orientations.emplace_back(quaternion_t(orientationValues.segment<4>(4 * i)));
        }
        return orientations;
    }

    ad_vector_t PinocchioEndEffectorKinematicsCppAd::getOrientationErrorWrtPlaneCppAd(
        PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
        const PinocchioStateInputMapping<ad_scalar_t>& mapping,
        const ad_vector_t& state, const ad_vector_t& params)
    {
        const auto& model = pinocchioInterfaceCppAd.getModel();
        auto& data = pinocchioInterfaceCppAd.getData();
        const ad_vector_t q = mapping.getPinocchioJointPosition(state);

        forwardKinematics(model, data, q);
        updateFramePlacements(model, data);

        ad_vector_t errors(3 * endEffectorFrameIds_.size());
        for (int i = 0; i < endEffectorFrameIds_.size(); i++)
        {
            const size_t frameId = endEffectorFrameIds_[i];
            const Eigen::Matrix<ad_scalar_t, 3, 1> planeNormal = params.segment<3>(3 * i);
            errors.segment<3>(3 * i) = rotationMatrixDistanceToPlane<ad_scalar_t>(data.oMf[frameId].rotation(),
                                                                                  planeNormal);
        }
        return errors;
    }

    auto PinocchioEndEffectorKinematicsCppAd::getOrientationErrorWrtPlane(const vector_t& state,
                                                                          const std::vector<vector3_t>& planeNormals)
    const
        -> std::vector<vector3_t>
    {
        vector_t params(3 * endEffectorIds_.size());
        for (int i = 0; i < endEffectorIds_.size(); i++)
        {
            params.segment<3>(3 * i) = planeNormals[i];
        }

        const vector_t errorValues = orientationErrorWrtPlaneCppAdInterfacePtr_->getFunctionValue(state, params);

        std::vector<vector3_t> errors;
        errors.reserve(endEffectorIds_.size());
        for (int i = 0; i < endEffectorIds_.size(); i++)
        {
            errors.emplace_back(errorValues.segment<3>(3 * i));
        }
        return errors;
    }

    std::vector<VectorFunctionLinearApproximation>
    PinocchioEndEffectorKinematicsCppAd::getOrientationErrorWrtPlaneLinearApproximation(
        const vector_t& state, const std::vector<vector3_t>& planeNormals) const
    {
        vector_t params(3 * endEffectorIds_.size());
        for (int i = 0; i < endEffectorIds_.size(); i++)
        {
            params.segment<3>(3 * i) = planeNormals[i];
        }

        const vector_t errorValues = orientationErrorWrtPlaneCppAdInterfacePtr_->getFunctionValue(state, params);
        const matrix_t errorJacobian = orientationErrorWrtPlaneCppAdInterfacePtr_->getJacobian(state, params);

        std::vector<VectorFunctionLinearApproximation> errors;
        errors.reserve(endEffectorIds_.size());
        for (int i = 0; i < endEffectorIds_.size(); i++)
        {
            VectorFunctionLinearApproximation err;
            err.f = errorValues.segment<3>(3 * i);
            err.dfdx = errorJacobian.block(3 * i, 0, 3, state.rows());
            errors.emplace_back(std::move(err));
        }
        return errors;
    }

    ad_vector_t PinocchioEndEffectorKinematicsCppAd::getAngularVelocityCppAd(
        PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
        const PinocchioStateInputMapping<ad_scalar_t>& mapping,
        const ad_vector_t& state, const ad_vector_t& input)
    {
        const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
        const auto& model = pinocchioInterfaceCppAd.getModel();
        auto& data = pinocchioInterfaceCppAd.getData();
        const ad_vector_t q = mapping.getPinocchioJointPosition(state);
        const ad_vector_t v = mapping.getPinocchioJointVelocity(state, input);

        forwardKinematics(model, data, q, v);

        ad_vector_t angularVelocities(3 * endEffectorFrameIds_.size());
        for (int i = 0; i < endEffectorFrameIds_.size(); i++)
        {
            const size_t frameId = endEffectorFrameIds_[i];
            angularVelocities.segment<3>(3 * i) = getFrameVelocity(model, data, frameId, rf).angular();
        }
        return angularVelocities;
    }

    auto PinocchioEndEffectorKinematicsCppAd::getAngularVelocity(const vector_t& state,
                                                                 const vector_t& input) const -> std::vector<vector3_t>
    {
        vector_t stateInput(state.rows() + input.rows());
        stateInput << state, input;
        const vector_t velocityValues = angularVelocityCppAdInterfacePtr_->getFunctionValue(stateInput);

        std::vector<vector3_t> velocities;
        velocities.reserve(endEffectorIds_.size());
        for (int i = 0; i < endEffectorIds_.size(); i++)
        {
            velocities.emplace_back(velocityValues.segment<3>(3 * i));
        }
        return velocities;
    }

    std::vector<VectorFunctionLinearApproximation>
    PinocchioEndEffectorKinematicsCppAd::getAngularVelocityLinearApproximation(
        const vector_t& state, const vector_t& input) const
    {
        vector_t stateInput(state.rows() + input.rows());
        stateInput << state, input;
        const vector_t velocityValues = angularVelocityCppAdInterfacePtr_->getFunctionValue(stateInput);
        const matrix_t velocityJacobian = angularVelocityCppAdInterfacePtr_->getJacobian(stateInput);

        std::vector<VectorFunctionLinearApproximation> velocities;
        velocities.reserve(endEffectorIds_.size());
        for (int i = 0; i < endEffectorIds_.size(); i++)
        {
            VectorFunctionLinearApproximation vel;
            vel.f = velocityValues.segment<3>(3 * i);
            vel.dfdx = velocityJacobian.block(3 * i, 0, 3, state.rows());
            vel.dfdu = velocityJacobian.block(3 * i, state.rows(), 3, input.rows());
            velocities.emplace_back(std::move(vel));
        }
        return velocities;
    }

    ad_vector_t PinocchioEndEffectorKinematicsCppAd::getTwistCppAd(
        PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
        const PinocchioStateInputMapping<ad_scalar_t>& mapping,
        const ad_vector_t& state, const ad_vector_t& input)
    {
        const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
        const auto& model = pinocchioInterfaceCppAd.getModel();
        auto& data = pinocchioInterfaceCppAd.getData();
        const ad_vector_t q = mapping.getPinocchioJointPosition(state);
        const ad_vector_t v = mapping.getPinocchioJointVelocity(state, input);

        forwardKinematics(model, data, q, v);

        ad_vector_t twists(6 * endEffectorFrameIds_.size());
        for (int i = 0; i < endEffectorFrameIds_.size(); i++)
        {
            const size_t frameId = endEffectorFrameIds_[i];
            const auto motion = getFrameVelocity(model, data, frameId, rf);
            twists.segment<3>(6 * i) = motion.linear();
            twists.segment<3>(6 * i + 3) = motion.angular();
        }
        return twists;
    }

    auto PinocchioEndEffectorKinematicsCppAd::getTwist(const vector_t& state,
                                                       const vector_t& input) const -> std::vector<vector6_t>
    {
        vector_t stateInput(state.rows() + input.rows());
        stateInput << state, input;
        const vector_t twistValues = twistCppAdInterfacePtr_->getFunctionValue(stateInput);

        std::vector<vector6_t> twists;
        twists.reserve(endEffectorIds_.size());
        for (int i = 0; i < endEffectorIds_.size(); i++)
        {
            twists.emplace_back(twistValues.segment<6>(6 * i));
        }
        return twists;
    }

    std::vector<VectorFunctionLinearApproximation> PinocchioEndEffectorKinematicsCppAd::getTwistLinearApproximation(
        const vector_t& state, const vector_t& input) const
    {
        vector_t stateInput(state.rows() + input.rows());
        stateInput << state, input;
        const vector_t twistValues = twistCppAdInterfacePtr_->getFunctionValue(stateInput);
        const matrix_t twistJacobian = twistCppAdInterfacePtr_->getJacobian(stateInput);

        std::vector<VectorFunctionLinearApproximation> twists;
        twists.reserve(endEffectorIds_.size());
        for (int i = 0; i < endEffectorIds_.size(); i++)
        {
            VectorFunctionLinearApproximation vel;
            vel.f = twistValues.segment<6>(6 * i);
            vel.dfdx = twistJacobian.block(6 * i, 0, 6, state.rows());
            vel.dfdu = twistJacobian.block(6 * i, state.rows(), 6, input.rows());
            twists.emplace_back(std::move(vel));
        }
        return twists;
    }
} // namespace ocs2
