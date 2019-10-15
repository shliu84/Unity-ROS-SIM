/*
© CentraleSupelec, 2017
Author: Dr. Jeremy Fix (jeremy.fix@centralesupelec.fr)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

// Adjustments to new Publication Timing and Execution Framework
// © Siemens AG, 2018, Dr. Martin Bischoff (martin.bischoff@siemens.com)

using UnityEngine;
namespace RosSharp.RosBridgeClient
{
    public class MyTwistSubscriber : UnitySubscriber<MessageTypes.Geometry.Twist>
    {
        private Vector3 linearVelocity;
        private Vector3 angularVelocity;
        private bool isMessageReceived;

        public Rigidbody SubscribedRigidbody;
        private Vector3 m_EulerAngleVelocity;

        protected override void Start()
        {
            base.Start();
        }

        protected override void ReceiveMessage(MessageTypes.Geometry.Twist message)
        {
            // Debug.Log("Message received");
            linearVelocity = linearVelocityToVector3(message.linear).Ros2Unity();
            angularVelocity = angularVelocityToVector3(message.angular).Ros2Unity();
            isMessageReceived = true;
        }

        private static Vector3 linearVelocityToVector3(MessageTypes.Geometry.Vector3 geometryVector3)
        {
            return new Vector3((float)geometryVector3.y, -(float)geometryVector3.x, 0);
        }

         private static Vector3 angularVelocityToVector3(MessageTypes.Geometry.Vector3 geometryVector3)
        {
            return new Vector3(0, 0, -(float)geometryVector3.z);
        }


        private void FixedUpdate()
        {
            if (isMessageReceived)
                ProcessMessage();
        }
        private void ProcessMessage()
        {
            SubscribedRigidbody.velocity = linearVelocity;
            SubscribedRigidbody.angularVelocity = angularVelocity;

            isMessageReceived = false;
        }
    }
}