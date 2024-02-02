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

#include "LeePositionController.hh"

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
void LeePositionController::CalculateRotorVelocities(
    const FrameData &_frameData, const EigenTwist &_cmd,
    Eigen::VectorXd &_rotorVelocities) const
{
  Eigen::Vector3d acceleration =
      this->ComputeDesiredAcceleration(_frameData, _cmd);
  
  Eigen::Matrix3d rot = _frameData.pose.linear();
  Eigen::Vector3d b1Des = rot.col(0);

  Eigen::Vector3d b3Des;
  b3Des = -_acceleration / _acceleration.norm();

  // Check if b1 and b3 are parallel. If so, choose a different b1 vector. This
  // could happen if the UAV is rotated by 90 degrees w.r.t the horizontal
  // plane.
  const double tol = 1e-3;
  if (b1Des.cross(b3Des).squaredNorm() < tol)
  {
    // acceleration and b1 are parallel. Choose a different vector
    b1Des = rot.col(1);

    if (b1Des.cross(b3Des).squaredNorm() < tol)
    {
      b1Des = rot.col(2);
    }
  }

  Eigen::Vector3d b2Des;
  b2Des = b3Des.cross(b1Des);
  b2Des.normalize();

  Eigen::Matrix3d rotDes;
  rotDes.col(0) = b2Des.cross(b3Des);
  rotDes.col(1) = b2Des;
  rotDes.col(2) = b3Des;
  
  double desAngRate = this->controllerParameters.angularRateGain[2];

  Eigen::Vector3d angularAcceleration =
      this->ComputeDesiredAngularAcc(rot, rotDes, desAngRate);

  // Project thrust onto body z axis.
  double thrust = -this->vehicleParameters.mass *
                  acceleration.dot(_frameData.pose.linear().col(2));

  Eigen::Vector4d angularAccelerationThrust;
  angularAccelerationThrust.block<3, 1>(0, 0) = angularAcceleration;
  angularAccelerationThrust(3) = thrust;

  _rotorVelocities =
      this->angularAccToRotorVelocities * angularAccelerationThrust;

  _rotorVelocities =
      _rotorVelocities.cwiseMax(Eigen::VectorXd::Zero(_rotorVelocities.rows()));
  _rotorVelocities = _rotorVelocities.cwiseSqrt();
}

//////////////////////////////////////////////////
// Implementation from the T. Lee et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on
// SE(3)
Eigen::Vector3d LeePositionController::ComputeDesiredAngularAcc(
    const FrameData &_frameData, const EigenPositionYaw &_posYawDes,
    const Eigen::Vector3d &_acceleration) const
{
  Eigen::Matrix3d rot = _frameData.pose.linear();

  // Get current yaw. Need to convert to math::Quaterniond to use the Yaw() since Eigen::eulerAngles has a weird behavior.
  Eigen::Quaterniond currentEigenQuat = Eigen::Quaterniond(rot);
  math::Quaterniond currentQuat(currentEigenQuat.w(), currentEigenQuat.x(), currentEigenQuat.y(), currentEigenQuat.z());
  double yawCurrent = currentQuat.Yaw();

  // Desired yaw
  double yawDes = _posYawDes.yaw;

  // Get the desired rotation matrix.
  Eigen::Vector3d b1Des = rot.col(0);

  Eigen::Vector3d b3Des;
  b3Des = -_acceleration / _acceleration.norm();

  // Check if b1 and b3 are parallel. If so, choose a different b1 vector. This
  // could happen if the UAV is rotated by 90 degrees w.r.t the horizontal
  // plane.
  const double tol = 1e-3;
  if (b1Des.cross(b3Des).squaredNorm() < tol)
  {
    // acceleration and b1 are parallel. Choose a different vector
    b1Des = rot.col(1);

    if (b1Des.cross(b3Des).squaredNorm() < tol)
    {
      b1Des = rot.col(2);
    }
  }

  Eigen::Vector3d b2Des;
  b2Des = b3Des.cross(b1Des);
  b2Des.normalize();

  Eigen::Matrix3d rotDes;
  rotDes.col(0) = b2Des.cross(b3Des);
  rotDes.col(1) = b2Des;
  rotDes.col(2) = b3Des;

  // Angle error according to lee et al.
  Eigen::Matrix3d angleErrorMatrix =
      0.5 * (rotDes.transpose() * rot - rot.transpose() * rotDes);
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

  // The paper shows
  // e_omega = omega - R.T * R_d * omega_des
  // The code in the RotorS implementation has
  // e_omega = omega - R_d.T * R * omega_des
  Eigen::Vector3d angularRateError = _frameData.angularVelocityBody -
                                     rot.transpose() * rotDes * angularRateDes;

  // The following MOI terms are computed in the paper, but the RotorS
  // implementation ignores them. They don't appear to make much of a
  // difference.
  // Eigen::Matrix3d moi = this->vehicleParameters.inertia;
  // const Eigen::Vector3d &omega = _frameData.angularVelocityBody;

  // Eigen::Vector3d moiTerm = omega.cross(moi * omega);

  // Eigen::Vector3d moiTerm2 = moi * (skewMatrixFromVector(omega) *
  //                            rot.transpose() * rotDes * angularRateDes);

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
