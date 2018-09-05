/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include "ignition/gazebo/systems/Physics.hh"

#include <ignition/math/Pose3.hh>
#include <ignition/plugin/RegisterMore.hh>

#include "ignition/gazebo/EntityComponentManager.hh"

#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/components/WorldStatistics.hh"

using namespace ignition::gazebo::systems;

using namespace std::chrono_literals;

// Private data class.
class ignition::gazebo::systems::PhysicsPrivate
{
  /// \brief Query callback for entity that has physics components.
  /// \param[in] _manager Entity component manager.
  public: void OnUpdate(EntityComponentManager &_manager);

  /// \brief Query callback to update time.
  /// \param[in] _manager Entity component manager.
  public: void OnUpdateTime(EntityComponentManager &_manager);
};

//////////////////////////////////////////////////
Physics::Physics()
  : System(), dataPtr(new PhysicsPrivate)
{
}

//////////////////////////////////////////////////
Physics::~Physics()
{
}

//////////////////////////////////////////////////
void Physics::Init(std::vector<EntityQueryCallback> &_cbs)
{
  _cbs.push_back(
      std::bind(&PhysicsPrivate::OnUpdate, this->dataPtr.get(),
        std::placeholders::_1));

  _cbs.push_back(
      std::bind(&PhysicsPrivate::OnUpdateTime, this->dataPtr.get(),
        std::placeholders::_1));
}

//////////////////////////////////////////////////
void PhysicsPrivate::OnUpdateTime(EntityComponentManager &_manager)
{
  auto *worldStats = _manager.First<components::WorldStatistics>();

  const auto *worldComponent = _manager.First<components::World>();

  /// \todo(nkoenig) We might want to prevent all systems from modifying
  /// simulation time.
  worldStats->AddSimTime(worldComponent->MaxStep());

  worldStats->AddIterations(1u);
}

//////////////////////////////////////////////////
void PhysicsPrivate::OnUpdate(EntityComponentManager &_manager)
{
  // Sleep for some amount of time to simulate the computation needed to
  // update physics.
  _manager.Each<components::Pose>(
    [&](const EntityId &/*_entity*/, const components::Pose *_pose)
    {
      if (_pose)
      {
        std::cout << "Pose[" << _pose->Data() << "]\n";
        std::this_thread::sleep_for(50us);
      }
    });

  // \todo(nkoenig) AcutallyUpdate dynamics

  // \todo(nkoenig) Update collisions

  // \todo(nkoenig) Update entity pose information.
}


IGNITION_ADD_PLUGIN(ignition::gazebo::systems::Physics,
                    ignition::gazebo::System)
