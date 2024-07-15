//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.UnityRoboticsDemo
{
    [Serializable]
    public class DriveMsg : Message
    {
        public const string k_RosMessageName = "unity_robotics_demo_msgs/Drive";
        public override string RosMessageName => k_RosMessageName;

        //  Command thrust amount to Kingfisher thruster,
        //  transmitted from higher-level software to the MCU 
        //  on the /cmd_drive topic.
        //  Thrust amount ranges from [-1.0..1.0], where 1.0 pushes Kingfisher forward.
        public float left;
        public float right;

        public DriveMsg()
        {
            this.left = 0.0f;
            this.right = 0.0f;
        }

        public DriveMsg(float left, float right)
        {
            this.left = left;
            this.right = right;
        }

        public static DriveMsg Deserialize(MessageDeserializer deserializer) => new DriveMsg(deserializer);

        private DriveMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.left);
            deserializer.Read(out this.right);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.left);
            serializer.Write(this.right);
        }

        public override string ToString()
        {
            return "DriveMsg: " +
            "\nleft: " + left.ToString() +
            "\nright: " + right.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}