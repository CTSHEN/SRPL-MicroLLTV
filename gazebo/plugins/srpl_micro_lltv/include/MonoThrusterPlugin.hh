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

// This header file is modified for H2O2 mono-thruster.
// Modified by C.T. Shen, 2023 Aug.

#ifndef __GAZEBO_MONO_THRUSTER_PLUGIN_HH__
#define __GAZEBO_MONO_THRUSTER_PLUGIN_HH__

#include<boost/scoped_ptr.hpp>

#include<string>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/transport/TransportTypes.hh>

#include <sdf/sdf.hh>

#include "srpl_micro_lltv/ThrusterCmdStamped.h" // Subscribe custom msg with thruster cmd and time

namespace gazebo
{
    /// \brief Definition of a pointer to the thruster command messages
    typedef const boost::shared_ptr<const srpl_micro_lltv::ThrusterCmdStamped>
    ConstTCmdPtr;

    /// \brief Class for mono thruster plugin
    class MonoThrusterPlugin : public ModelPlugin
    {
        /// \brief Constructor
        public: MonoThrusterPlugin();

        /// \brief Destructor
        public: virtual ~MonoThrusterPlugin();

        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        /// \brief Update the simulation state.
        /// \param[in] _info Information used in the update event.
        public: void Update(const common::UpdateInfo &_info);

        /// \brief Callback for the input topic subscriber
        protected: void UpdateInput(ConstTCmdPtr &_msg);

        // TODO: Thruster dynamic model

        /// \brief Update event
        protected: event::ConnectionPtr updateConnection;

        /// \brief Pointer to the thruster link
        protected: physics::LinkPtr thrusterLink;

        /// \brief Gazebo node
        protected: transport::NodePtr node;

        /// \brief subscriber to the thruster command
        protected: ros::Subscriber commandSubscriber;

        /// \brief Publisher to output thruster topic
        protected: transport::PublisherPtr thrusterTopicPublisher;

        /// \brief Input command: on or off
        protected: bool inputCommand;

        /// \brief Latest thrust force in N
        protected: double thrustForce;

        /// \brief Timestamp of the thrust force
        protected: common::Time thrustForceStamp;

        /// \brief Thruster joint, used to define thruster direction.
        protected: physics::JointPtr joint;

        /// \brief Maximum thrust force output in N
        protected: double thrustMax;

        /// \brief Minimum thrust force output in N, for mono thruster is often 0
        protected: double thrustMin;

        /// \brief Thruster ID, used to generated topic names automatically.
        protected: int thrusterID;

        /// \brief Thruster topics prefix
        protected: std::string topicPrefix;

        /// \brief The axis about the thrusters
        protected: ignition::math::Vector3d thrusterAxis;

        /// \brief Pointer to this ROS node's handle
        private: boost::scoped_ptr<ros::NodeHandle> rosNode;

    };
}
#endif  // __GAZEBO_MONO_THRUSTER_PLUGIN_HH__