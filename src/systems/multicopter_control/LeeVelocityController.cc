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

#include "LeeVelocityController.hh"

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
void LeeVelocityController::CalculateRotorVelocities(
    const FrameData &_frameData, const EigenTwist &_cmdVel,
    Eigen::VectorXd &_rotorVelocities) const
{
  Eigen::Vector3d acceleration =
      this->ComputeDesiredAcceleration(_frameData, _cmdVel);

  Eigen::Vector3d angularAcceleration =
      this->ComputeDesiredAngularAcc(_frameData, _cmdVel, acceleration);

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
}  // namespace multicopter_control
}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz
