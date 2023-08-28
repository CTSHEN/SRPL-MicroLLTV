// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// This file is modified for H2O2 mono-thruster.
// Modified by C.T. Shen, 2023 Aug.

/// \brief Model plugin for description of the mono-thruster dynamics

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <sdf/sdf.hh>

#include <MonoThrusterPlugin.hh>

GZ_REGISTER_MODEL_PLUGIN(gazebo::MonoThrusterPlugin)

namespace gazebo
{

MonoThrusterPlugin::MonoThrusterPlugin() : inputCommand(false),
    thrustForce(0),
    thrustMax(20),
    thrusterID(-1)
    {
    }

MonoThrusterPlugin::~MonoThrusterPlugin()
{
    if (this->updateConnection)
    {
        thsi->updateConnection.reset();
    }
}

void MonoThrusterPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
} // namespace gazebo
