using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WASP_Streaming : MonoBehaviour
{
    int n_tick;
    // Start is called before the first frame update
    void Start()
    {
        n_tick = 0;
    }

    // Update is called once per frame
    void Update()
    {
        ScreenCapture.CaptureScreenshot("/home/reiti/Videos/"+n_tick+".png");
        n_tick++;
    }
}
