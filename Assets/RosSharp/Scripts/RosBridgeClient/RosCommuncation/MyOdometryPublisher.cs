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
    public class MyOdometryPublisher : UnityPublisher<MessageTypes.Nav.Odometry>
    {
        // public Transform PublishedTransform;
        public Rigidbody SubscribedRigidbody;
        public string FrameId = "odom";
        public string ChildFrameId = "base_link";

        private float x = 0.0f;
        private float y = 0.0f;
        private float th = 0.0f;
        private float previousRealTime;

        private MessageTypes.Nav.Odometry message;
        // private Vector3 position;
        // private Quaternion rotation;

        // private MessageTypes.Geometry.PoseWithCovariance pose;
        // private MessageTypes.Geometry.TwistWithCovariance twist;


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
            message = new MessageTypes.Nav.Odometry();
            message.header.frame_id = FrameId;
            message.child_frame_id = ChildFrameId;
        }

        private void UpdateMessage()
        {
            float vx = SubscribedRigidbody.velocity.x;
            float vy = SubscribedRigidbody.velocity.z;
            float vth = -SubscribedRigidbody.angularVelocity.y;
            // Debug.Log(SubscribedRigidbody.angularVelocity);

            float dt = Time.realtimeSinceStartup - previousRealTime;//tosec??
            
            float delta_x = (vx * Mathf.Cos(th) - vy * Mathf.Sin(th)) * dt;
            float delta_y = (vx * Mathf.Sin(th) + vy * Mathf.Cos(th)) * dt;
            float delta_th = vth * dt;

            x += delta_x;
            y += delta_y;
            th += delta_th;

            previousRealTime = Time.realtimeSinceStartup;
            

            createQuaternionMsgFromYaw(th);
            
            message.header.Update();
            message.pose.pose.position = createPosition(x,y,0);
            message.pose.pose.orientation = createQuaternionMsgFromYaw(th);;

            message.twist.twist.linear = linearVelocityToGeometryVector3(SubscribedRigidbody.velocity);
            message.twist.twist.angular = angularVelocityToGeometryVector3(SubscribedRigidbody.angularVelocity);  
            
            Publish(message);
        }

        private MessageTypes.Geometry.Point createPosition(float x,float y,float z)
        {
            MessageTypes.Geometry.Point v3 = new MessageTypes.Geometry.Point();
            v3.x = x;//x;
            v3.y = y;//y;
            v3.z = 0;//z;
            // Debug.Log(v3);
            return v3;
        }

        private MessageTypes.Geometry.Quaternion createQuaternionMsgFromYaw(float th){
            Quaternion odom_quat = Quaternion.Euler(0, th*180/Mathf.PI, 0);
            // Debug.Log(odom_quat);
            MessageTypes.Geometry.Quaternion q = new MessageTypes.Geometry.Quaternion();
            q.x = 0;
            q.y = 0;
            q.z = odom_quat.y;
            q.w = odom_quat.w;
            // Debug.Log(q);
            return q;   
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

        private MessageTypes.Geometry.Point GetGeometryPoint(Vector3 position)
        {
            MessageTypes.Geometry.Point geometryPoint = new MessageTypes.Geometry.Point();
            geometryPoint.x = position.x;
            geometryPoint.y = -position.y;
            geometryPoint.z = position.z;
            return geometryPoint;
        }

        private MessageTypes.Geometry.Quaternion GetGeometryQuaternion(Quaternion quaternion)
        {
            MessageTypes.Geometry.Quaternion geometryQuaternion = new MessageTypes.Geometry.Quaternion();
            geometryQuaternion.x = quaternion.x;
            geometryQuaternion.y = -quaternion.y;
            geometryQuaternion.z = quaternion.z;
            geometryQuaternion.w = quaternion.w;
            return geometryQuaternion;
        }

        // protected override void ReceiveMessage(MessageTypes.Nav.Odometry message)
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

        // private Vector3 GetPosition(MessageTypes.Nav.Odometry message)
        // {
        //     return new Vector3(
        //         message.pose.pose.position.x,
        //         message.pose.pose.position.y,
        //         message.pose.pose.position.z);
        // }

        // private Quaternion GetRotation(MessageTypes.Nav.Odometry message)
        // {
        //     return new Quaternion(
        //         message.pose.pose.orientation.x,
        //         message.pose.pose.orientation.y,
        //         message.pose.pose.orientation.z,
        //         message.pose.pose.orientation.w);
        // }
    }
}
