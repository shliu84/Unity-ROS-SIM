using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Lidar : MonoBehaviour
{
    public int numberOfPoint = 8;

    void FixedUpdate()
    {
        float deltaAngle = 2f * Mathf.PI/(float)numberOfPoint;

        for (float i = 0;i < 2f * Mathf.PI; i = i + deltaAngle){
            RaycastHit hit;
            float x = Mathf.Sin(i);
            float z = Mathf.Cos(i);
            Vector3 direction = new Vector3(x, 0, z);
            Physics.Raycast(transform.position, direction, out hit);
            // if (Physics.Raycast(transform.position, direction, out hit))
                 // print("Found an object - distance: " + hit.distance);
            Debug.DrawRay(transform.position, direction*hit.distance, Color.green);
        }
        
    }
}
