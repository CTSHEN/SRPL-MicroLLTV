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

#include <iostream>

#include <boost/shared_ptr.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
//#include <gazebo/transport/TransportTypes.hh>
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
        this->updateConnection.reset();
    }
}

void MonoThrusterPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model != NULL, "Invalid model pointer");

    // Initializing the transport node
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(_model->GetWorld()->Name());
    this ->rosNode.reset(new ros::NodeHandle(""));

    // Retrieve the link name on which the thrust will be applied
    GZ_ASSERT(_sdf->HasElement("linkName"), "Could not find linkName.");
    std::string linkName = _sdf->Get<std::string>("linkName");
    this->thrusterLink = _model->GetLink(linkName);
    GZ_ASSERT(this->thrusterLink, "thruster link is invalid");

    // Reading thruster ID
    GZ_ASSERT(_sdf->HasElement("thrusterID"), "Thruster ID was not provided");
    this->thrusterID = _sdf->Get<int>("thrusterID");

    // Dynamic config
    //TODO

    if (_sdf->HasElement("thrustMax"))
        this->thrustMax = _sdf->Get<double>("thrustMax");

    // Root string for topics
    std::stringstream strs;
    strs << "/" << _model->GetName() << "/thrusters/" << this->thrusterID << "/";
    this->topicPrefix = strs.str();

    // Advertise the thrust topic
    this->thrusterTopicPublisher =
        this->node->Advertise<msgs::Vector3d>(this->topicPrefix + "thrust");

    // Subscribe to the input signal topic
    /*
    this->commandSubscriber =
        this->node->Subscribe(this->topicPrefix + "input",
            &MonoThrusterPlugin::UpdateInput,
            this); */
    this->commandSubscriber = this->rosNode->subscribe<
        srpl_micro_lltv::ThrusterCmdStamped>(this->commandSubscriber.getTopic(),
        10, boost::bind(&MonoThrusterPlugin::UpdateInput, this, _1));

    // Connect the update event
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&MonoThrusterPlugin::Update,
                    this, _1));

    this->thrusterAxis = this->joint->WorldPose().Rot().RotateVectorReverse(this->joint->GlobalAxis(0));
    
}

void MonoThrusterPlugin::Update(const common::UpdateInfo &_info)
{
    GZ_ASSERT(!std::isnan(this->inputCommand),
            "nan in this->inputCommand");

    double ThrusterCmd;
    double dynamicState;

    ThrusterCmd = static_cast<double>(this->inputCommand);

    dynamicState = ThrusterCmd; //TODO: thruster dynamics
    this->thrustForce = dynamicState * this->thrustMax;

    this->thrustForceStamp = _info.simTime;
    ignition::math::Vector3d force(this->thrustForce*this->thrusterAxis);

    this->thrusterLink->AddRelativeForce(force);
}

void MonoThrusterPlugin::UpdateInput(ConstTCmdPtr &_msg)
{
    this->inputCommand = _msg->thrusterCmd;
}
} // namespace gazebo
