//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.UnityRoboticsDemo
{
    [Serializable]
    public class AckermannDriveMsg : Message
    {
        public const string k_RosMessageName = "unity_robotics_demo_msgs/AckermannDrive";
        public override string RosMessageName => k_RosMessageName;

        // # Driving command for a car-like vehicle using Ackermann steering.
        //   $Id$
        //  Assumes Ackermann front-wheel steering. The left and right front
        //  wheels are generally at different angles. To simplify, the commanded
        //  angle corresponds to the yaw of a virtual wheel located at the
        //  center of the front axle, like on a tricycle.  Positive yaw is to
        //  the left. (This is *not* the angle of the steering wheel inside the
        //  passenger compartment.)
        // 
        //  Zero steering angle velocity means change the steering angle as
        //  quickly as possible. Positive velocity indicates a desired absolute
        //  rate of change either left or right. The controller tries not to
        //  exceed this limit in either direction, but sometimes it might.
        // 
        public float steering_angle;
        //  desired virtual angle (radians)
        public float steering_angle_velocity;
        //  desired rate of change (radians/s)
        //  Drive at requested speed, acceleration and jerk (the 1st, 2nd and
        //  3rd derivatives of position). All are measured at the vehicle's
        //  center of rotation, typically the center of the rear axle. The
        //  controller tries not to exceed these limits in either direction, but
        //  sometimes it might.
        // 
        //  Speed is the desired scalar magnitude of the velocity vector.
        //  Direction is forward unless the sign is negative, indicating reverse.
        // 
        //  Zero acceleration means change speed as quickly as
        //  possible. Positive acceleration indicates a desired absolute
        //  magnitude; that includes deceleration.
        // 
        //  Zero jerk means change acceleration as quickly as possible. Positive
        //  jerk indicates a desired absolute rate of acceleration change in
        //  either direction (increasing or decreasing).
        // 
        public float speed;
        //  desired forward speed (m/s)
        public float acceleration;
        //  desired acceleration (m/s^2)
        public float jerk;
        //  desired jerk (m/s^3)

        public AckermannDriveMsg()
        {
            this.steering_angle = 0.0f;
            this.steering_angle_velocity = 0.0f;
            this.speed = 0.0f;
            this.acceleration = 0.0f;
            this.jerk = 0.0f;
        }

        public AckermannDriveMsg(float steering_angle, float steering_angle_velocity, float speed, float acceleration, float jerk)
        {
            this.steering_angle = steering_angle;
            this.steering_angle_velocity = steering_angle_velocity;
            this.speed = speed;
            this.acceleration = acceleration;
            this.jerk = jerk;
        }

        public static AckermannDriveMsg Deserialize(MessageDeserializer deserializer) => new AckermannDriveMsg(deserializer);

        private AckermannDriveMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.steering_angle);
            deserializer.Read(out this.steering_angle_velocity);
            deserializer.Read(out this.speed);
            deserializer.Read(out this.acceleration);
            deserializer.Read(out this.jerk);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.steering_angle);
            serializer.Write(this.steering_angle_velocity);
            serializer.Write(this.speed);
            serializer.Write(this.acceleration);
            serializer.Write(this.jerk);
        }

        public override string ToString()
        {
            return "AckermannDriveMsg: " +
            "\nsteering_angle: " + steering_angle.ToString() +
            "\nsteering_angle_velocity: " + steering_angle_velocity.ToString() +
            "\nspeed: " + speed.ToString() +
            "\nacceleration: " + acceleration.ToString() +
            "\njerk: " + jerk.ToString();
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
