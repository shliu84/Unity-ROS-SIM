using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Shooting : MonoBehaviour
{
    public GameObject bullet;
    public Transform shotSpawn;
        public float fireRate;
    private float nextFire;
    // Start is called before the first frame update
    void Start()
    {
        
    }
    
    // Update is called once per frame
    public void Update()
    {
        if (Input.GetButton("Fire1") && Time.time > nextFire)
        {
        	nextFire = Time.time + fireRate;
            //    GameObject clone =
            Instantiate(bullet, shotSpawn.position, shotSpawn.rotation); //as GameObject;
            // GetComponent<AudioSource>().Play();
        }
    }
}
