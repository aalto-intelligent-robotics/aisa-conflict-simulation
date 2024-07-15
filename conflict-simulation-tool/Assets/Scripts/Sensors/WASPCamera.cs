using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.UnityRoboticsDemo;
using System;

public class WASPCamera : MonoBehaviour
{
    ROSConnection ros;

    public Camera camera;
    private Camera sensorcam;
    public float publishMessageFrequency = 0.04f;
    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;
    public string topicName = "wasp_image/compressed";

    private RawImage display;

    int count;


    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<CompressedImageMsg>(topicName);

        count = 0;

        sensorcam = camera.GetComponent<Camera>();

        //display = camera.GetComponent<RawImage>();

    }

    // Update is called once per frame
    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {       
            // Finally send the message to server_endpoint.py running in ROS
            //ros.Publish(topicName, display);

            var oldRT = RenderTexture.active;
            RenderTexture.active = sensorcam.targetTexture;
            sensorcam.Render();

            // Copy the pixels from the GPU into a texture so we can work with them
            // For more efficiency you should reuse this texture, instead of creating a new one every time
            Texture2D camText = new Texture2D(sensorcam.targetTexture.width, sensorcam.targetTexture.height);
            camText.ReadPixels(new Rect(0, 0, sensorcam.targetTexture.width, sensorcam.targetTexture.height), 0, 0);
            camText.Apply();
            RenderTexture.active = oldRT;

            //var timeMessage = new TimeMsg(timeSinceStart.Seconds, timeSinceStart.Milliseconds);
            //var headerMessage = new HeaderMsg(count, timeMessage, "camera");

            var headerMessage = new RosMessageTypes.Std.HeaderMsg( //header
                    (uint)Time.frameCount,
                    new RosMessageTypes.BuiltinInterfaces.TimeMsg(
                        (uint)Math.Floor(Time.realtimeSinceStartup),
                        (uint)((Time.realtimeSinceStartup - Mathf.Floor(Time.realtimeSinceStartup)) * 1000000000)),
                    "map");
            

              // Encode the texture as a PNG, and send to ROS
            byte[] imageBytes = camText.EncodeToJPG();

            string picString = Convert.ToBase64String(imageBytes);
            byte[] array = System.Text.Encoding.UTF8.GetBytes(picString);

            var message = new CompressedImageMsg(headerMessage, "jpeg", imageBytes);
            //var message2 = new ImageMsg(headerMessage, (uint)sensorcam.targetTexture.height, (uint)sensorcam.targetTexture.width, "uint8", 1, (uint)sensorcam.targetTexture.width*3,  imageBytes);
            //ros.Send(topicName, message);
            ros.Publish(topicName, message);

            timeElapsed = 0;
            count++;
        }
        
    }
}
