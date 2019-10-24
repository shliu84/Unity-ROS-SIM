

using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class MyClockPublisher : UnityPublisher<MessageTypes.Rosgraph.Clock>{
        private MessageTypes.Std.Time stamp;

        private MessageTypes.Rosgraph.Clock clock;

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
            clock = new MessageTypes.Rosgraph.Clock();
            stamp = new MessageTypes.Std.Time();

            // Debug.Log(stamp);
        }
        private void UpdateMessage()
        {
            float time = Time.realtimeSinceStartup;
            stamp.secs = (uint)time;
            stamp.nsecs = (uint)(1e9 * (time - stamp.secs));
            clock.clock = stamp;
            Publish(clock);
            
        }
    }

}