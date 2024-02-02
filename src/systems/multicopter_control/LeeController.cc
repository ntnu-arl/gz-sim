/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "LeeController.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
namespace multicopter_control
{
//////////////////////////////////////////////////
std::unique_ptr<LeeController> LeeController::MakeController(
    const LeeControllerParameters &_controllerParams,
    const VehicleParameters &_vehicleParams)
{
  // auto controller = std::make_unique<LeeController>();
  // Can't use make_unique here because the constructor is private
  std::unique_ptr<LeeController> controller(
      new LeeController());
  controller->controllerParameters = _controllerParams;
  controller->vehicleParameters = _vehicleParams;
  if (controller->InitializeParameters())
  {
    return controller;
  }
  else
  {
    return nullptr;
  }
}

//////////////////////////////////////////////////
void LeeController::CalculateRotorVelocities(
    const FrameData &_frameData, const EigenTwist &_cmdVel,
    Eigen::VectorXd &_rotorVelocities) const
{
  Eigen::Vector4d angularAccelerationThrust;
  if (this->controller->controllerParameters.mode != ControllerMode::kAttitudeControl)
  {
    Eigen::Vector3d acceleration =
      this->ComputeDesiredAcceleration(_frameData, _cmdVel);
  
    Eigen::Matrix3d rot = _frameData.pose.linear();
    Eigen::Vector3d bodyRateDes;
    Eigen::Matrix3d rotDes;
    
    this->ComputeDesiredRotMatBodyRate(acceleration, rot, rotDes, bodyRateDes);

    Eigen::Vector3d angularAcceleration =
        this->ComputeDesiredAngularAcc(_frameData, rotDes, bodyRateDes);

    // Project thrust onto body z axis.
    double thrust = -this->vehicleParameters.mass *
                    acceleration.dot(_frameData.pose.linear().col(2));

    angularAccelerationThrust.block<3, 1>(0, 0) = angularAcceleration;
    angularAccelerationThrust(3) = thrust;
  }
  else
  {
    // TODO @mihirk284
    

  }

  
  
  this->CalculateRotorVelocitiesFromAngAccThrust(angularAccelerationThrust, _rotorVelocities}
}

//////////////////////////////////////////////////
void LeeController::ComputeDesiredRotMatBodyRate(Eigen::Vector3d &_acceleration, Eigen::Matrix3d &_rot, Eigen::Matrix3d &_rotDes, Eigen::Vector3d &_bodyRateDes)
{
  Eigen::Matrix3d rot = _frameData.pose.linear();

  // Get current yaw. Need to convert to math::Quaterniond to use the Yaw() since Eigen::eulerAngles has a weird behavior.
  Eigen::Quaterniond currentEigenQuat = Eigen::Quaterniond(_rot);
  math::Quaterniond currentQuat(currentEigenQuat.w(), currentEigenQuat.x(), currentEigenQuat.y(), currentEigenQuat.z());
  double yawCurrent = currentQuat.Yaw();

  // Desired yaw
  double yawDes = _posYawDes.yaw;

  // Get the desired rotation matrix.
  Eigen::Vector3d b1Des = _rot.col(0);

  Eigen::Vector3d b3Des;
  b3Des = -_acceleration / _acceleration.norm();

  // Check if b1 and b3 are parallel. If so, choose a different b1 vector. This
  // could happen if the UAV is rotated by 90 degrees w.r.t the horizontal
  // plane.
  const double tol = 1e-3;
  if (b1Des.cross(b3Des).squaredNorm() < tol)
  {
    // acceleration and b1 are parallel. Choose a different vector
    b1Des = _rot.col(1);

    if (b1Des.cross(b3Des).squaredNorm() < tol)
    {
      b1Des = _rot.col(2);
    }
  }

  Eigen::Vector3d b2Des;
  b2Des = b3Des.cross(b1Des);
  b2Des.normalize();

  _rotDes.col(0) = b2Des.cross(b3Des);
  _rotDes.col(1) = b2Des;
  _rotDes.col(2) = b3Des;

  // Angle error according to lee et al.
  Eigen::Matrix3d angleErrorMatrix =
      0.5 * (_rotDes.transpose() * _rot - _rot.transpose() * _rotDes);
  Eigen::Vector3d angleError = vectorFromSkewMatrix(angleErrorMatrix);

  double yawrate_error = yawDes - yawCurrent;

  // Debug
  if (yawrate_error > M_PI || yawrate_error < -M_PI)
  {
    gzerr << "Yaw rate error is greater than pi. yawrate_error: " << yawrate_error << std::endl;
  }
  
  Eigen::Vector3d angularRateDes(Eigen::Vector3d::Zero());
  // current yaw angle
  
  angularRateDes[2] = yawrate_error;
}

//////////////////////////////////////////////////
void LeeController::CalculateRotorVelocitiesFromAngAccThrust(const EigenVector4d &_angAccThrust,
    Eigen::VectorXd &_rotorVelocities) const
{
  _rotorVelocities =
      this->angularAccToRotorVelocities * _angAccThrust;

  _rotorVelocities =
      _rotorVelocities.cwiseMax(Eigen::VectorXd::Zero(_rotorVelocities.rows()));
  _rotorVelocities = _rotorVelocities.cwiseSqrt();
}

//////////////////////////////////////////////////
bool LeeController::InitializeParameters()
{
  auto allocationMatrix =
      calculateAllocationMatrix(this->vehicleParameters.rotorConfiguration);
  if (!allocationMatrix.has_value())
  {
    // Error should already be printed by function
    return false;
  }

  // To make the tuning independent of the inertia matrix we divide here.
  this->normalizedAttitudeGain =
      this->controllerParameters.attitudeGain.transpose() *
      this->vehicleParameters.inertia.inverse();

  this->normalizedAngularRateGain =
      this->controllerParameters.angularRateGain.transpose() *
      this->vehicleParameters.inertia.inverse();

  Eigen::Matrix4d moi;
  moi.setZero();
  moi.block<3, 3>(0, 0) = this->vehicleParameters.inertia;
  moi(3, 3) = 1;

  this->angularAccToRotorVelocities.resize(
      this->vehicleParameters.rotorConfiguration.size(), 4);
  // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia
  // matrix I. A^{ \dagger} = A^T*(A*A^T)^{-1}
  const auto &aMat = *allocationMatrix;
  this->angularAccToRotorVelocities =
      aMat.transpose() * (aMat * aMat.transpose()).inverse() * moi;

  return true;
}

//////////////////////////////////////////////////
Eigen::Vector3d LeeController::ComputeDesiredAcceleration(
                 const FrameData &_frameData, const EigenTwist &_cmdVel) const
{
  Eigen::Vector3d positionError = _frameData.pose.translation() -
                                  _posYawDes.position;

  Eigen::Vector3d velocityError = _frameData.linearVelocityWorld -
                                  _frameData.pose.linear() * _cmdVel.linear;

  // This is a generalized acceleration command that can be used by both LeeVelocityController and Lee PositionController.
  // The initialization from SDFormat ensures that the coefficients of the unselected controller mode is always zero.
  Eigen::Vector3d accelCommand =
      (positionError.cwiseProduct(this->controllerParameters.positionGain) +
      velocityError.cwiseProduct(this->controllerParameters.velocityGain)) /
      this->vehicleParameters.mass;

  accelCommand = accelCommand.cwiseAbs()
      .cwiseMin(this->controllerParameters.maxLinearAcceleration)
      .cwiseProduct(accelCommand.cwiseSign());

  return accelCommand + this->vehicleParameters.gravity;
}

//////////////////////////////////////////////////
// Implementation from the T. Lee et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on
// SE(3)
Eigen::Vector3d LeeController::ComputeDesiredAngularAcc(
    const Eigen::Matrix3d &_rot, const Eigen::Matrix3d &_rotDes,
    const Eigen::Vector3d &_bodyRateDes) const
{
  // Angle error according to lee et al.
  Eigen::Matrix3d angleErrorMatrix =
      0.5 * (_rotDes.transpose() * _rot - _rot.transpose() * _rotDes);
  Eigen::Vector3d angleError = vectorFromSkewMatrix(angleErrorMatrix);

  // The paper shows
  // e_omega = omega - R.T * R_d * omega_des
  // The code in the RotorS implementation has
  // e_omega = omega - R_d.T * R * omega_des
  Eigen::Vector3d angularRateError = _frameData.angularVelocityBody -
                                     _rot.transpose() * _rotDes * _bodyRateDes;

  // The following MOI terms are computed in the paper, but the RotorS
  // implementation ignores them. They don't appear to make much of a
  // difference.
  // Eigen::Matrix3d moi = this->vehicleParameters.inertia;
  // const Eigen::Vector3d &omega = _frameData.angularVelocityBody;

  // Eigen::Vector3d moiTerm = omega.cross(moi * omega);

  // Eigen::Vector3d moiTerm2 = moi * (skewMatrixFromVector(omega) *
  //                            _rot.transpose() * _rotDes * angularRateDes);

  // std::cout << moiTerm2.transpose() << std::endl;
  // return -1 * angleError.cwiseProduct(this->normalizedAttitudeGain) -
  //         angularRateError.cwiseProduct(this->normalizedAngularRateGain) +
  //         moiTerm - moiTerm2;
  return -1 * angleError.cwiseProduct(this->normalizedAttitudeGain) -
          angularRateError.cwiseProduct(this->normalizedAngularRateGain);
}
}  // namespace multicopter_control
}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz
