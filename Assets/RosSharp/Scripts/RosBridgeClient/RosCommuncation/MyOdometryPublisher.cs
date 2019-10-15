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
/* */
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class MyOdometryPublisher : Publisher<Messages.Navigation.Odometry>
    {
        // public Transform PublishedTransform;
        public Rigidbody SubscribedRigidbody;
        public string FrameId = "odom";
        public string ChildFrameId = "base_link";

        private Messages.Navigation.Odometry message;
        // private Vector3 position;
        // private Quaternion rotation;

        // private Messages.Geometry.PoseWithCovariance pose;
        // private Messages.Geometry.TwistWithCovariance twist;


        protected override void Start()
		{
			base.Start();
            InitializeMessage();
		}
		
        private void Update()
        {
            UpdateMessage();
        }

        private void InitializeMessage()
        {
            message = new Messages.Navigation.Odometry();
            message.header.frame_id = FrameId;
            message.child_frame_id = ChildFrameId;
        }

        private void UpdateMessage()
        {
            message.header.Update();
            message.pose.pose.position = GetGeometryPoint(SubscribedRigidbody.position.Unity2Ros());
            message.pose.pose.orientation = GetGeometryQuaternion(SubscribedRigidbody.rotation.Unity2Ros());

            message.twist.twist.linear = linearVelocityToGeometryVector3(SubscribedRigidbody.velocity);
            message.twist.twist.angular = angularVelocityToGeometryVector3(SubscribedRigidbody.angularVelocity);  
            
            Publish(message);
        }
        private static Messages.Geometry.Vector3 linearVelocityToGeometryVector3(Vector3 vector3)
        {
            Messages.Geometry.Vector3 geometryVector3 = new Messages.Geometry.Vector3();
            geometryVector3.x = vector3.x;
            geometryVector3.y = vector3.z;
            geometryVector3.z = 0;
            return geometryVector3;
        }

         private static Messages.Geometry.Vector3 angularVelocityToGeometryVector3(Vector3 vector3)
        {
            Messages.Geometry.Vector3 geometryVector3 = new Messages.Geometry.Vector3();
            geometryVector3.x = 0;
            geometryVector3.y = 0;
            geometryVector3.z = -vector3.z;
            return geometryVector3;
        }

        private Messages.Geometry.Point GetGeometryPoint(Vector3 position)
        {
            Messages.Geometry.Point geometryPoint = new Messages.Geometry.Point();
            geometryPoint.x = position.x;
            geometryPoint.y = -position.y;
            geometryPoint.z = position.z;
            return geometryPoint;
        }

        private Messages.Geometry.Quaternion GetGeometryQuaternion(Quaternion quaternion)
        {
            Messages.Geometry.Quaternion geometryQuaternion = new Messages.Geometry.Quaternion();
            geometryQuaternion.x = quaternion.x;
            geometryQuaternion.y = -quaternion.y;
            geometryQuaternion.z = quaternion.z;
            geometryQuaternion.w = quaternion.w;
            return geometryQuaternion;
        }

        // protected override void ReceiveMessage(Messages.Navigation.Odometry message)
        // {
        //     position = GetPosition(message).Ros2Unity();
        //     rotation = GetRotation(message).Ros2Unity();
        //     isMessageReceived = true;
        // }
        // private void ProcessMessage()
        // {
        //     PublishedTransform.position = position;
        //     PublishedTransform.rotation = rotation;
        // }

        // private Vector3 GetPosition(Messages.Navigation.Odometry message)
        // {
        //     return new Vector3(
        //         message.pose.pose.position.x,
        //         message.pose.pose.position.y,
        //         message.pose.pose.position.z);
        // }

        // private Quaternion GetRotation(Messages.Navigation.Odometry message)
        // {
        //     return new Quaternion(
        //         message.pose.pose.orientation.x,
        //         message.pose.pose.orientation.y,
        //         message.pose.pose.orientation.z,
        //         message.pose.pose.orientation.w);
        // }
    }
}
