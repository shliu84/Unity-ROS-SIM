/*
© Siemens AG, 2017-2018
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

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

using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class MyTwistPublisher : UnityPublisher<MessageTypes.Geometry.Twist>
    {
        private Vector3 linearVelocity;
        private Vector3 angularVelocity;
        public Rigidbody SubscribedRigidbody;

        private MessageTypes.Geometry.Twist message;

        protected override void Start()
        {
            base.Start();
            InitializeMessage();
        }

        private void FixedUpdate()
        {
            UpdateMessage();
        }

        private void InitializeMessage()
        {
            message = new MessageTypes.Geometry.Twist();
            message.linear = new MessageTypes.Geometry.Vector3();
            message.angular = new MessageTypes.Geometry.Vector3();
        }
        private void UpdateMessage()
        {
            message.linear = linearVelocityToGeometryVector3(SubscribedRigidbody.velocity);
            message.angular = angularVelocityToGeometryVector3(SubscribedRigidbody.angularVelocity);  
            Publish(message);
        }

        private static MessageTypes.Geometry.Vector3 linearVelocityToGeometryVector3(Vector3 vector3)
        {
            MessageTypes.Geometry.Vector3 geometryVector3 = new MessageTypes.Geometry.Vector3();
            geometryVector3.x = vector3.x;
            geometryVector3.y = vector3.z;
            geometryVector3.z = 0;
            return geometryVector3;
        }

         private static MessageTypes.Geometry.Vector3 angularVelocityToGeometryVector3(Vector3 vector3)
        {
            MessageTypes.Geometry.Vector3 geometryVector3 = new MessageTypes.Geometry.Vector3();
            geometryVector3.x = 0;
            geometryVector3.y = 0;
            geometryVector3.z = -vector3.z;
            return geometryVector3;
        }
        

    }
}
