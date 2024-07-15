using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.UnityRoboticsDemo;



public class WASP_DriveInterface : MonoBehaviour
{
    public GameObject car;
    private VehicleControl car_control;
    ROSConnection ros;
    public float publishMessageFrequency = 0.5f;
    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;
    public string topicName = "drive_input";

    public float wasp_steer = 0.0f;
    public float wasp_speed = 0.0f;
    public float wasp_accel = 0.0f;
    // Start is called before the first frame update
    void Start()
    {
        // start the ROS connection
        ros = ROSConnection.GetOrCreateInstance();

        car_control = car.GetComponent<VehicleControl>();

        // For publishing of data
        //ros.RegisterPublisher<AckermannDriveMsg>(topicName);
        ros.Subscribe<AckermannDriveMsg>("ackermanncontrol",AckermannControl);
    }

    void AckermannControl(AckermannDriveMsg ackermannMessage){
        wasp_accel = ackermannMessage.acceleration;
        wasp_steer = ackermannMessage.steering_angle;
        Debug.Log(wasp_steer);
        //car_control.steer = ackermannMessage.steering_angle;
        //car_control.accel = ackermannMessage.acceleration;
        //car_control.speed = ackermannMessage.speed;
    }

    // Update is called once per frame
    void Update()
    {   
        // only for publisher ---->

        /*
        
        car_control = car.GetComponent<VehicleControl>();

        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            //cube.transform.rotation = Random.rotation;
            
            AckermannDriveMsg carAckermann = new AckermannDriveMsg(
                car_control.steer,
                0.0f,
                car_control.speed,
                car_control.accel,
                0.0f
            );
            

            // Finally send the message to server_endpoint.py running in ROS
            ros.Publish(topicName, carAckermann);

            timeElapsed = 0;
        }
        */
        

    }
}
