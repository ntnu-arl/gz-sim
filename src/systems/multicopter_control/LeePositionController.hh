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

#ifndef GZ_SIM_SYSTEMS_MULTICOPTER_CONTROL_LEEPOSITIONCONTROLLER_HH_
#define GZ_SIM_SYSTEMS_MULTICOPTER_CONTROL_LEEPOSITIONCONTROLLER_HH_

#include <Eigen/Geometry>
#include <memory>
#include "gz/sim/config.hh"

#include "Common.hh"
#include "LeeController.hh"
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
  /// This controller is inspired by the LeePositionController from RotorS
  /// https://github.com/ethz-asl/rotors_simulator/blob/master/rotors_control/include/rotors_control/lee_position_controller.h
  /// The controller can be used to command linear velocity and yaw angle
  /// velocity expressed in the body frame.
  class LeePositionController : public LeeController
  {
    /// \brief Calculate rotor velocities given the current frame data and the
    /// commanded velocity
    /// \param[in] _frameData Frame data including pose and linear and angular
    /// velocities
    /// \param[in] _cmdVel Commanded velocity
    /// \param[out] _rotorVelocities Computed rotor velocities.
    public: void CalculateRotorVelocities(
                 const FrameData &_frameData,
                 const EigenPositionYaw &_posYawDes,
                 Eigen::VectorXd &_rotorVelocities) const;

    /// \brief Private constructor. Use MakeController to create an instance of
    /// this class
    private: LeePositionController() = default;
  };
}  // namespace multicopter_control
}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif
