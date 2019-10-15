using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class MyTFPublisher : UnityPublisher<MessageTypes.Tf2.TFMessage>{
        public string FrameId = "odom";
        public string ChildFrameId = "base_link";

        private MessageTypes.Tf2.TFMessage message;
        private MessageTypes.Geometry.TransformStamped[] transforms;

        public Rigidbody SubscribedRigidbody;
        
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
            transforms = new MessageTypes.Geometry.TransformStamped[]{
                new MessageTypes.Geometry.TransformStamped()
            };
            transforms[0].header.frame_id = FrameId;
            transforms[0].child_frame_id = ChildFrameId;
            message = new MessageTypes.Tf2.TFMessage(transforms);
        }
        private void UpdateMessage()
        {
            message.transforms[0].header.Update();
            message.transforms[0].transform.translation = GetGeometryPoint(SubscribedRigidbody.position.Unity2Ros());
            message.transforms[0].transform.rotation = GetGeometryQuaternion(SubscribedRigidbody.rotation.Unity2Ros());

            // message.twist.twist.linear = linearVelocityToGeometryVector3(SubscribedRigidbody.velocity);
            // message.twist.twist.angular = angularVelocityToGeometryVector3(SubscribedRigidbody.angularVelocity);  
            
            Publish(message);
        }
        // private static MessageTypes.Geometry.Vector3 linearVelocityToGeometryVector3(Vector3 vector3)
        // {
        //     MessageTypes.Geometry.Vector3 geometryVector3 = new MessageTypes.Geometry.Vector3();
        //     geometryVector3.x = vector3.x;
        //     geometryVector3.y = vector3.z;
        //     geometryVector3.z = 0;
        //     return geometryVector3;
        // }

        //  private static MessageTypes.Geometry.Vector3 angularVelocityToGeometryVector3(Vector3 vector3)
        // {
        //     MessageTypes.Geometry.Vector3 geometryVector3 = new MessageTypes.Geometry.Vector3();
        //     geometryVector3.x = 0;
        //     geometryVector3.y = 0;
        //     geometryVector3.z = -vector3.z;
        //     return geometryVector3;
        // }

        private MessageTypes.Geometry.Vector3 GetGeometryPoint(Vector3 position)
        {
            MessageTypes.Geometry.Vector3 geometryPoint = new MessageTypes.Geometry.Vector3();
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

    }
}