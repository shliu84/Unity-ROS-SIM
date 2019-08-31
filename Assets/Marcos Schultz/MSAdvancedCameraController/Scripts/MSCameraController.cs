using System;
using UnityEngine;
using UnityEngine.UI;

[Serializable]
public class MSACC_CameraType {
	[Tooltip("A camera must be associated with this variable. The camera that is associated here, will receive the settings of this index.")]
	public Camera _camera;
	public enum TipoRotac{LookAtThePlayer, FirstPerson, FollowPlayer, Orbital, Stop, StraightStop, OrbitalThatFollows, ETS_StyleCamera, FlyCamera_OnlyWindows}
	[Tooltip("Here you must select the type of rotation and movement that camera will possess.")]
	public TipoRotac rotationType = TipoRotac.LookAtThePlayer;
	[Range(0.01f,1.0f)][Tooltip("Here you must adjust the volume that the camera attached to this element can perceive. In this way, each camera can perceive a different volume.")]
	public float volume = 1.0f;
}
[Serializable]
public class MSACC_CameraSetting {
	[Header("Configure Inputs")]
	[Tooltip("The input that will define the horizontal movement of the cameras.")]
	public string inputMouseX = "Mouse X";
	[Tooltip("The input that will define the vertical movement of the cameras.")]
	public string inputMouseY = "Mouse Y";
	[Tooltip("The input that allows you to zoom in or out of the camera.")]
	public string inputMouseScrollWheel = "Mouse ScrollWheel";
	[Tooltip("In this variable you can configure the key responsible for switching cameras.")]
	public KeyCode cameraSwitchKey = KeyCode.C;

	public enum UpdateMode {Update, FixedUpdate, LateUpdate};
	[Header("Update mode")]
	[Tooltip("Here it is possible to decide whether the motion of the cameras will be processed in the void Update, FixedUpdate or LateUpdate. The mode that best suits most situations is the 'LateUpdate'.")]
	public UpdateMode camerasUpdateMode = UpdateMode.LateUpdate;

	[Header("General settings")]
	[Tooltip("If this variable is checked, the script will automatically place the 'IgnoreRaycast' layer on the player when needed.")]
	public bool ajustTheLayers = true;
	[Tooltip("In this class you can configure the 'FirstPerson' style cameras.")]
	public MSACC_SettingsCameraFirstPerson firstPerson;
	[Tooltip("In this class you can configure the 'FollowPlayer' style cameras.")]
	public MSACC_SettingsCameraFollow followPlayer;
	[Tooltip("In this class you can configure the 'Orbital' style cameras.")]
	public MSACC_SettingsCameraOrbital orbital;
	[Tooltip("In this class you can configure the 'OrbitalThatFollows' style cameras.")]
	public MSACC_SettingsCameraOrbitalThatFollows OrbitalThatFollows;
	[Tooltip("In this class you can configure the 'ETS_StyleCamera' style cameras.")]
	public MSACC_SettingsCameraETS_StyleCamera ETS_StyleCamera;
	[Tooltip("In this class you can configure the 'FlyCamera' style cameras.")]
	public MSACC_SettingsFlyCamera FlyCamera_OnlyWindows;
}
[Serializable]
public class MSACC_SettingsCameraFirstPerson {
	[Header("Sensibility")]
	[Range(1,20)][Tooltip("Horizontal camera rotation sensitivity.")]
	public float sensibilityX = 10.0f;
	[Range(1,20)][Tooltip("Vertical camera rotation sensitivity.")]
	public float sensibilityY = 10.0f;
	[Range(0,1)][Tooltip("The speed with which the camera can approach your vision through the mouseScrool.")]
	public float speedScroolZoom = 0.5f;
	[Header("Limits")]
	[Range(0,360)][Tooltip("The highest horizontal angle that camera style 'FistPerson' camera can achieve.")]
	public float horizontalAngle = 65.0f;
	[Range(0,85)][Tooltip("The highest vertical angle that camera style 'FistPerson' camera can achieve.")]
	public float verticalAngle = 20.0f;
	[Range(0,40)][Tooltip("The maximum the camera can approximate your vision.")]
	public float maxScroolZoom = 30.0f;

	[Header("Custom Rotation Input")]
	[Tooltip("If this variable is true, the camera will only rotate when the key selected in the 'KeyToRotate' variable is pressed. If this variable is false, the camera can rotate freely, even without pressing any key.")]
	public bool rotateWhenClick = false;
	[Tooltip("Here you can select the button that must be pressed in order to rotate the camera.")]
	public string keyToRotate = "mouse 0";

	[Space(7)]
	[Tooltip("If this variable is true, the X-axis input will be inverted.")]
	public bool invertXInput = false;
	[Tooltip("If this variable is true, the X-axis input will be inverted.")]
	public bool invertYInput = false;
}
[Serializable]
public class MSACC_SettingsCameraFollow {
	[Header("Collision")]
	[Tooltip("If this variable is true, the camera ignores the colliders and crosses the walls freely.")]
	public bool ignoreCollision = false;

	[Header("Movement")]
	[Range(1,20)][Tooltip("The speed at which the camera can follow the player.")]
	public float displacementSpeed = 3.0f;

	[Header("Rotation")]
	[Tooltip("If this variable is true, the code makes a lookAt using quaternions.")]
	public bool customLookAt = false;
	[Range(1,30)][Tooltip("The speed at which the camera rotates as it follows and looks at the player.")]
	public float spinSpeedCustomLookAt = 15.0f;

	[Header("Use Scrool")]
	[Tooltip("If this variable is true, the 'FollowPlayer' camera style will have the player's distance affected by the mouse scrool. This will allow the player to zoom in or out of the camera.")]
	public bool useScrool = false;
	[Range(0.01f,2.0f)][Tooltip("The speed at which the player can zoom in and out of the camera.")]
	public float scroolSpeed = 1.0f;
	[Range(1,30)][Tooltip("The minimum distance the camera can be relative to the player.")]
	public float minDistance = 7.0f;
	[Range(1,200)][Tooltip("The maximum distance the camera can be relative to the player.")]
	public float maxDistance = 40.0f;
}
[Serializable]
public class MSACC_SettingsCameraOrbital {
	[Header("Settings")]
	[Range(0.01f,2.0f)][Tooltip("In this variable you can configure the sensitivity with which the script will perceive the movement of the X and Y inputs. ")]
	public float sensibility = 0.8f;
	[Range(0.01f,2.0f)][Tooltip("In this variable, you can configure the speed at which the orbital camera will approach or distance itself from the player when the mouse scrool is used.")]
	public float speedScrool = 1.0f;
	[Range(0.01f,2.0f)][Tooltip("In this variable, you can configure the speed at which the orbital camera moves up or down.")]
	public float speedYAxis = 0.5f;

	[Header("Limits")]
	[Range(3.0f,20.0f)][Tooltip("In this variable, you can set the minimum distance that the orbital camera can stay from the player.")]
	public float minDistance = 5.0f;
	[Range(20.0f,1000.0f)][Tooltip("In this variable, you can set the maximum distance that the orbital camera can stay from the player.")]
	public float maxDistance = 50.0f;
	[Range(-85,0)][Tooltip("In this variable it is possible to define the minimum angle that the camera can reach on the Y axis")]
	public float minAngleY = 0.0f;
	[Range(0,85)][Tooltip("In this variable it is possible to define the maximum angle that the camera can reach on the Y axis")]
	public float maxAngleY = 80.0f;
	[Tooltip("If this variable is true, the camera ignores the colliders and crosses the walls freely.")]
	public bool ignoreCollision = false;

	[Header("Custom Rotation Input")]
	[Tooltip("If this variable is true, the camera will only rotate when the key selected in the 'KeyToRotate' variable is pressed. If this variable is false, the camera can rotate freely, even without pressing any key.")]
	public bool rotateWhenClick = false;
	[Tooltip("Here you can select the button that must be pressed in order to rotate the camera.")]
	public string keyToRotate = "mouse 0";

	[Space(7)]
	[Tooltip("If this variable is true, the X-axis input will be inverted.")]
	public bool invertXInput = false;
	[Tooltip("If this variable is true, the X-axis input will be inverted.")]
	public bool invertYInput = false;
}
[Serializable]
public class MSACC_SettingsCameraOrbitalThatFollows {
	[Header("Settings(Follow)")]
	[Range(1,20)][Tooltip("The speed at which the camera can follow the player.")]
	public float displacementSpeed = 5.0f;
	[Tooltip("If this variable is true, the code makes a lookAt using quaternions.")]
	public bool customLookAt = false;
	[Range(1,30)][Tooltip("The speed at which the camera rotates as it follows and looks at the player.")]
	public float spinSpeedCustomLookAt = 15.0f;

	[Header("Settings(Orbital)")]
	[Range(0.01f,2.0f)][Tooltip("In this variable you can configure the sensitivity with which the script will perceive the movement of the X and Y inputs. ")]
	public float sensibility = 0.8f;
	[Range(0.01f,2.0f)][Tooltip("In this variable, you can configure the speed at which the orbital camera will approach or distance itself from the player when the mouse scrool is used.")]
	public float speedScrool = 1.0f;
	[Range(0.01f,2.0f)][Tooltip("In this variable, you can configure the speed at which the orbital camera moves up or down.")]
	public float speedYAxis = 0.5f;
	[Range(3.0f,20.0f)][Tooltip("In this variable, you can set the minimum distance that the orbital camera can stay from the player.")]
	public float minDistance = 5.0f;
	[Range(20.0f,1000.0f)][Tooltip("In this variable, you can set the maximum distance that the orbital camera can stay from the player.")]
	public float maxDistance = 50.0f;
	[Range(-85,0)][Tooltip("In this variable it is possible to define the minimum angle that the camera can reach on the Y axis")]
	public float minAngleY = 0.0f;
	[Range(0,85)][Tooltip("In this variable it is possible to define the maximum angle that the camera can reach on the Y axis")]
	public float maxAngleY = 80.0f;
	[Space(7)]
	[Tooltip("If this variable is true, the camera will only rotate when the key selected in the 'KeyToRotate' variable is pressed. If this variable is false, the camera can rotate freely, even without pressing any key.")]
	public bool rotateWhenClick = false;
	[Tooltip("Here you can select the button that must be pressed in order to rotate the camera.")]
	public string keyToRotate = "mouse 0";
	[Tooltip("If this variable is true, the X-axis input will be inverted.")]
	public bool invertXInput = false;
	[Tooltip("If this variable is true, the X-axis input will be inverted.")]
	public bool invertYInput = false;
	//
	public enum ResetTimeType { Time, Input_OnlyWindows }
	[Header("Settings(General)")]
	[Tooltip("In this variable it is possible to define how the control will be redefined for the camera that follows the player, through input or through a time.")]
	public ResetTimeType ResetControlType = ResetTimeType.Time;
	[Tooltip("If 'ResetControlType' is set to 'Input_OnlyWindows', the key that must be pressed to reset the control will be set by this variable.")]
	public KeyCode resetKey = KeyCode.Z;
	[Range(1.0f,50.0f)][Tooltip("If 'ResetControlType' is set to 'Time', the wait time for the camera to reset the control will be set by this variable.")]
	public float timeToReset = 8.0f;
	[Tooltip("If this variable is true, the camera ignores the colliders and crosses the walls freely.")]
	public bool ignoreCollision = false;
}
[Serializable]
public class MSACC_SettingsCameraETS_StyleCamera {
	[Header("Settings")]
	[Range(1,20)][Tooltip("Horizontal camera rotation sensitivity.")]
	public float sensibilityX = 10.0f;
	[Range(1,20)][Tooltip("Vertical camera rotation sensitivity.")]
	public float sensibilityY = 10.0f;
	[Range(0.5f,3.0f)][Tooltip("The distance the camera will move to the left when the mouse is also shifted to the left. This option applies only to cameras that have the 'ETS_StyleCamera' option selected.")]
	public float ETS_CameraShift = 1.0f;
	[Range(0,40)][Tooltip("The maximum the camera can approximate your vision.")]
	public float maxScroolZoom = 30.0f;
	[Range(0,1)][Tooltip("The speed with which the camera can approach your vision through the mouseScrool.")]
	public float speedScroolZoom = 0.5f;

	[Header("Custom Rotation Input")]
	[Tooltip("If this variable is true, the camera will only rotate when the key selected in the 'KeyToRotate' variable is pressed. If this variable is false, the camera can rotate freely, even without pressing any key.")]
	public bool rotateWhenClick = false;
	[Tooltip("Here you can select the button that must be pressed in order to rotate the camera.")]
	public string keyToRotate = "mouse 0";
	[Space(7)]
	[Tooltip("If this variable is true, the X-axis input will be inverted.")]
	public bool invertXInput = false;
	[Tooltip("If this variable is true, the X-axis input will be inverted.")]
	public bool invertYInput = false;
}
[Serializable]
public class MSACC_SettingsFlyCamera {
	[Header("Inputs")]
	[Tooltip("Here you can configure the 'Horizontal' inputs that should be used to move the camera 'CameraFly'.")]
	public string horizontalMove = "Horizontal";
	[Tooltip("Here you can configure the 'Vertical' inputs that should be used to move the camera 'CameraFly'.")]
	public string verticalMove = "Vertical";
	[Tooltip("Here you can configure the keys that must be pressed to accelerate the movement of the camera 'CameraFly'.")]
	public KeyCode speedKeyCode = KeyCode.LeftShift;
	[Tooltip("Here you can configure the key that must be pressed to move the camera 'CameraFly' up.")]
	public KeyCode moveUp = KeyCode.E;
	[Tooltip("Here you can configure the key that must be pressed to move the camera 'CameraFly' down.")]
	public KeyCode moveDown = KeyCode.Q;
	//
	[Header("Settings")]
	[Range(1,20)][Tooltip("Horizontal camera rotation sensitivity.")]
	public float sensibilityX = 10.0f;
	[Range(1,20)][Tooltip("Vertical camera rotation sensitivity.")]
	public float sensibilityY = 10.0f;
	[Range(1,100)][Tooltip("The speed of movement of this camera.")]
	public float movementSpeed = 20.0f;
	[Space(7)]
	[Tooltip("If this variable is true, the X-axis input will be inverted.")]
	public bool invertXInput = false;
	[Tooltip("If this variable is true, the X-axis input will be inverted.")]
	public bool invertYInput = false;
}

public class MSCameraController : MonoBehaviour {

	[Tooltip("Here you must associate the object that the cameras will follow. If you leave this variable empty, the cameras will follow the object in which this script was placed.")]
	public Transform target;

	[Space(7)][Tooltip("Here you must associate all the cameras that you want to control by this script, associating each one with an index and selecting your preferences.")]
	public MSACC_CameraType[] cameras = new MSACC_CameraType[0];
	[Tooltip("Here you can configure the cameras, deciding their speed of movement, rotation, zoom, among other options.")]
	public MSACC_CameraSetting CameraSettings;

	bool orbitalAtiv;
	bool orbital_AtivTemp;
	float rotacX = 0.0f;
	float rotacY = 0.0f;
	float tempoOrbit = 0.0f;
	float rotacXETS = 0.0f;
	float rotacYETS = 0.0f;

	Vector2 cameraRotationFly;
	bool changeCam;

	GameObject[] objPosicStopCameras;
	Quaternion[] originalRotation;
	GameObject[] originalPosition;
	Vector3[] originalPositionETS;
	float[] xOrbit;
	float[] yOrbit;
	float[] distanceFromOrbitalCamera;
	float[] initialFieldOfView;
	float[] camFollowPlayerDistance;

	int index = 0;
	int lastIndex = 0;

	Transform targetTransform;
	GameObject playerCamsObj;


	//global inputs
	[HideInInspector]
	public float _horizontalInputMSACC;
	[HideInInspector]
	public float _verticalInputMSACC;
	[HideInInspector]
	public float _scrollInputMSACC;
	[HideInInspector]
	public bool _enableMobileInputs;
	[HideInInspector]
	public int _mobileInputsIndex; // 0 = off,   1 = all,   2 = scroll buttons only

	void OnValidate (){
		for (int x = 0; x < cameras.Length; x++) {
			if (cameras [x].volume == 0) {
				cameras [x].volume = 1;
			}
		}
	}

	void Awake(){

		if (target) {
			targetTransform = target;
		} else {
			targetTransform = transform;
		}

		GameObject temp = new GameObject ("PlayerCams");
		temp.transform.parent = targetTransform;
		objPosicStopCameras = new GameObject[cameras.Length];
		originalRotation = new Quaternion[cameras.Length];
		originalPosition = new GameObject[cameras.Length];
		originalPositionETS = new Vector3[cameras.Length];
		xOrbit = new float[cameras.Length];
		yOrbit = new float[cameras.Length];

		distanceFromOrbitalCamera = new float[cameras.Length];
		initialFieldOfView = new float[cameras.Length];
		camFollowPlayerDistance = new float[cameras.Length];

		changeCam = false;
		orbitalAtiv = false;
		orbital_AtivTemp = false;

		for (int x = 0; x < cameras.Length; x++) {
			if (cameras [x]._camera) {
				if (cameras [x].volume == 0) {
					cameras [x].volume = 1;
				}
				initialFieldOfView [x] = cameras [x]._camera.fieldOfView;

				if (cameras [x].rotationType == MSACC_CameraType.TipoRotac.FirstPerson) {
					cameras [x]._camera.transform.parent = temp.transform;
					originalRotation [x] = cameras [x]._camera.transform.localRotation;
				}


				if (cameras [x].rotationType == MSACC_CameraType.TipoRotac.FollowPlayer) {
					cameras [x]._camera.transform.parent = temp.transform;
					originalPosition [x] = new GameObject ("positionFollowPlayerCamera" + x);
					originalPosition [x].transform.parent = temp.transform;
					originalPosition [x].transform.position = cameras [x]._camera.transform.position;
					if (CameraSettings.ajustTheLayers) {
						targetTransform.gameObject.layer = 2;
						foreach (Transform trans in targetTransform.gameObject.GetComponentsInChildren<Transform>(true)) {
							trans.gameObject.layer = 2;
						}
					}
					camFollowPlayerDistance [x] = Vector3.Distance (cameras [x]._camera.transform.position, targetTransform.position);
				}

				if (cameras [x].rotationType == MSACC_CameraType.TipoRotac.Orbital) {
					cameras [x]._camera.transform.parent = temp.transform;
					cameras [x]._camera.transform.LookAt (target);
					xOrbit [x] = cameras [x]._camera.transform.eulerAngles.y;
					yOrbit [x] = cameras [x]._camera.transform.eulerAngles.x;
					if (CameraSettings.ajustTheLayers) {
						targetTransform.gameObject.layer = 2;
						foreach (Transform trans in targetTransform.gameObject.GetComponentsInChildren<Transform>(true)) {
							trans.gameObject.layer = 2;
						}
					}
				}
				distanceFromOrbitalCamera [x] = Vector3.Distance (cameras [x]._camera.transform.position, targetTransform.position);
				distanceFromOrbitalCamera [x] = Mathf.Clamp (distanceFromOrbitalCamera [x], CameraSettings.orbital.minDistance, CameraSettings.orbital.maxDistance);

				if (cameras [x].rotationType == MSACC_CameraType.TipoRotac.Stop) {
					cameras [x]._camera.transform.parent = temp.transform;
				}

				if (cameras [x].rotationType == MSACC_CameraType.TipoRotac.StraightStop) {
					cameras [x]._camera.transform.parent = temp.transform;
					objPosicStopCameras [x] = new GameObject ("positionStraightStopCamera" + x);
					objPosicStopCameras [x].transform.parent = cameras [x]._camera.transform;
					objPosicStopCameras [x].transform.localPosition = new Vector3 (0, 0, 1.0f);
					objPosicStopCameras [x].transform.parent = temp.transform;
				}

				if (cameras [x].rotationType == MSACC_CameraType.TipoRotac.OrbitalThatFollows) {
					cameras [x]._camera.transform.parent = temp.transform;
					xOrbit [x] = cameras [x]._camera.transform.eulerAngles.x;
					yOrbit [x] = cameras [x]._camera.transform.eulerAngles.y;

					originalPosition [x] = new GameObject ("positionCameraFollowPlayer" + x);
					originalPosition [x].transform.parent = temp.transform;
					originalPosition [x].transform.position = cameras [x]._camera.transform.position;

					if (CameraSettings.ajustTheLayers) {
						targetTransform.gameObject.layer = 2;
						foreach (Transform trans in targetTransform.gameObject.GetComponentsInChildren<Transform>(true)) {
							trans.gameObject.layer = 2;
						}
					}
				}

				if (cameras [x].rotationType == MSACC_CameraType.TipoRotac.ETS_StyleCamera) {
					cameras [x]._camera.transform.parent = temp.transform;
					originalRotation [x] = cameras [x]._camera.transform.localRotation;
					originalPositionETS [x] = cameras [x]._camera.transform.localPosition;
				}

				if (cameras [x].rotationType == MSACC_CameraType.TipoRotac.FlyCamera_OnlyWindows) {
					cameras [x]._camera.transform.parent = null;
				}

				if (cameras [x].rotationType == MSACC_CameraType.TipoRotac.LookAtThePlayer) {
					cameras [x]._camera.transform.parent = null;
				}

				AudioListener audListner = cameras [x]._camera.GetComponent<AudioListener> ();
				if (audListner == null) {
					cameras [x]._camera.transform.gameObject.AddComponent (typeof(AudioListener));
				}

			} else {
				Debug.LogWarning ("There is no camera associated with the index " + x);
			}
		}
		playerCamsObj = temp;
		//
		if (CameraSettings.followPlayer.minDistance > CameraSettings.followPlayer.maxDistance) {
			CameraSettings.followPlayer.minDistance = 15;
			CameraSettings.followPlayer.maxDistance = 15;
			Debug.LogWarning ("The minimum distance from the 'FollowPlayer' camera is greater than the maximum distance, and this may cause errors.");
		}
	}

	void Start(){
		index = 0;
		lastIndex = 0;
		_enableMobileInputs = false;
		EnableCameras (index);
	}

	void EnableCameras (int index){
		if (cameras.Length > 0) {
			changeCam = true;
			for (int x = 0; x < cameras.Length; x++) {
				if (cameras [x]._camera) {
					if (x == index) {
						cameras [x]._camera.gameObject.SetActive (true);
						//
						if (cameras [x].rotationType == MSACC_CameraType.TipoRotac.FirstPerson) {
							rotacX = 0.0f;
							rotacY = 0.0f;
						}
						if (cameras [x].rotationType == MSACC_CameraType.TipoRotac.OrbitalThatFollows) {
							tempoOrbit = 0.0f;
						}
						if (cameras [x].rotationType == MSACC_CameraType.TipoRotac.ETS_StyleCamera) {
							rotacXETS = 0.0f;
							rotacYETS = 0.0f;
						}
						if (cameras [x].rotationType == MSACC_CameraType.TipoRotac.FlyCamera_OnlyWindows) {
							cameras [x]._camera.transform.position = cameras [lastIndex]._camera.transform.position;
							cameraRotationFly = new Vector2(cameras [lastIndex]._camera.transform.eulerAngles.y, 0);
						}
					} else {
						cameras [x]._camera.gameObject.SetActive (false);
					}
				}
			}
		}
	}

	void ManageCameras(){
		if (changeCam) {
			changeCam = false;
			for (int x = 0; x < cameras.Length; x++) {
				if (cameras [x].rotationType == MSACC_CameraType.TipoRotac.FollowPlayer) {
					if (cameras [x]._camera.isActiveAndEnabled) {
						cameras [x]._camera.transform.parent = null;
					} else {
						cameras [x]._camera.transform.parent = playerCamsObj.transform;
					}
				}
				//
				if (cameras [x].rotationType == MSACC_CameraType.TipoRotac.Orbital || cameras [x].rotationType == MSACC_CameraType.TipoRotac.OrbitalThatFollows) {
					cameras [x]._camera.transform.LookAt (target);
					xOrbit [x] = cameras [x]._camera.transform.eulerAngles.y;
					yOrbit [x] = cameras [x]._camera.transform.eulerAngles.x;
					distanceFromOrbitalCamera [x] = Vector3.Distance (cameras [x]._camera.transform.position, targetTransform.position);
					distanceFromOrbitalCamera [x] = Mathf.Clamp (distanceFromOrbitalCamera [x], CameraSettings.orbital.minDistance, CameraSettings.orbital.maxDistance);
				}
			}

			//set on/off mobile inputs
			if (cameras [index].rotationType == MSACC_CameraType.TipoRotac.Stop || cameras [index].rotationType == MSACC_CameraType.TipoRotac.StraightStop || 
				cameras [index].rotationType == MSACC_CameraType.TipoRotac.LookAtThePlayer || cameras [index].rotationType == MSACC_CameraType.TipoRotac.FlyCamera_OnlyWindows) {
				_mobileInputsIndex = 0; // 0 = all mobile inputs off
			}
			if (cameras [index].rotationType == MSACC_CameraType.TipoRotac.FirstPerson || cameras [index].rotationType == MSACC_CameraType.TipoRotac.ETS_StyleCamera ||
				cameras [index].rotationType == MSACC_CameraType.TipoRotac.Orbital || cameras [index].rotationType == MSACC_CameraType.TipoRotac.OrbitalThatFollows) {
				_mobileInputsIndex = 1; // 1 = all mobile inputs on
			}
			if (cameras [index].rotationType == MSACC_CameraType.TipoRotac.FollowPlayer) {
				_mobileInputsIndex = 2; // 2 = scroll buttons only
			}
		}

		AudioListener.volume = cameras [index].volume;
		float timeScaleSpeed = Mathf.Clamp (1.0f / Time.timeScale, 0.01f, 1);
		switch (cameras[index].rotationType ) {
		case MSACC_CameraType.TipoRotac.Stop:
			//stop camera
			break;
		case MSACC_CameraType.TipoRotac.StraightStop:
			Quaternion linearRotation = Quaternion.LookRotation(objPosicStopCameras[index].transform.position - cameras [index]._camera.transform.position, Vector3.up);
			cameras [index]._camera.transform.rotation = Quaternion.Slerp(cameras [index]._camera.transform.rotation, linearRotation, Time.deltaTime * 15.0f);
			break;
		case MSACC_CameraType.TipoRotac.LookAtThePlayer:
			cameras [index]._camera.transform.LookAt (targetTransform.position);
			break;
		case MSACC_CameraType.TipoRotac.FirstPerson:
			//getInputs
			float xInput = _horizontalInputMSACC;
			float yInput = _verticalInputMSACC;
			if (CameraSettings.firstPerson.invertXInput) {
				xInput = -_horizontalInputMSACC;
			}
			if (CameraSettings.firstPerson.invertYInput) {
				yInput = -_verticalInputMSACC;
			}
			if (CameraSettings.firstPerson.rotateWhenClick) {
				if (Input.GetKey (CameraSettings.firstPerson.keyToRotate) || _enableMobileInputs) {
					rotacX += xInput * CameraSettings.firstPerson.sensibilityX;
					rotacY += yInput * CameraSettings.firstPerson.sensibilityY;
				}
			} else {
				rotacX += xInput * CameraSettings.firstPerson.sensibilityX;
				rotacY += yInput * CameraSettings.firstPerson.sensibilityY;
			}
			//
			rotacX = ClampAngle (rotacX, -CameraSettings.firstPerson.horizontalAngle, CameraSettings.firstPerson.horizontalAngle);
			rotacY = ClampAngle (rotacY, -CameraSettings.firstPerson.verticalAngle, CameraSettings.firstPerson.verticalAngle);
			Quaternion xQuaternion = Quaternion.AngleAxis (rotacX, Vector3.up);
			Quaternion yQuaternion = Quaternion.AngleAxis (rotacY, -Vector3.right);
			Quaternion _nextRot = originalRotation [index] * xQuaternion * yQuaternion;
			cameras [index]._camera.transform.localRotation = Quaternion.Lerp (cameras [index]._camera.transform.localRotation, _nextRot, Time.deltaTime * 10.0f * timeScaleSpeed);
			//fieldOfView
			cameras [index]._camera.fieldOfView -= _scrollInputMSACC * CameraSettings.firstPerson.speedScroolZoom * 50.0f;
			if (cameras [index]._camera.fieldOfView < (initialFieldOfView [index] - CameraSettings.firstPerson.maxScroolZoom)) {
				cameras [index]._camera.fieldOfView = (initialFieldOfView [index] - CameraSettings.firstPerson.maxScroolZoom);
			}
			if (cameras [index]._camera.fieldOfView > initialFieldOfView [index]) {
				cameras [index]._camera.fieldOfView = (initialFieldOfView [index]);
			}
			break;
		case MSACC_CameraType.TipoRotac.FollowPlayer:
			//move
			RaycastHit hitCamFollow;
			if (CameraSettings.followPlayer.useScrool) {
				float camLerpSpeed = Time.deltaTime * CameraSettings.followPlayer.displacementSpeed * (camFollowPlayerDistance [index] * 0.1f);
				camFollowPlayerDistance [index] = camFollowPlayerDistance [index] - _scrollInputMSACC * (CameraSettings.followPlayer.scroolSpeed * 50.0f);
				camFollowPlayerDistance [index] = Mathf.Clamp (camFollowPlayerDistance [index], CameraSettings.followPlayer.minDistance, CameraSettings.followPlayer.maxDistance);
				Vector3 direction = (targetTransform.position - originalPosition [index].transform.position).normalized;
				Vector3 finalPos = targetTransform.position - direction * camFollowPlayerDistance [index];
				//
				if (!Physics.Linecast (targetTransform.position, finalPos)) {
					cameras [index]._camera.transform.position = Vector3.Lerp (cameras [index]._camera.transform.position, finalPos, camLerpSpeed);
				} else if (Physics.Linecast (targetTransform.position, finalPos, out hitCamFollow)) {
					if (CameraSettings.followPlayer.ignoreCollision) {
						cameras [index]._camera.transform.position = Vector3.Lerp (cameras [index]._camera.transform.position, finalPos, camLerpSpeed);
					} else {
						cameras [index]._camera.transform.position = Vector3.Lerp (cameras [index]._camera.transform.position, hitCamFollow.point, camLerpSpeed);
					}
				}
			} else {
				if (!Physics.Linecast (targetTransform.position, originalPosition [index].transform.position)) {
					cameras [index]._camera.transform.position = Vector3.Lerp (cameras [index]._camera.transform.position, originalPosition [index].transform.position, Time.deltaTime * CameraSettings.followPlayer.displacementSpeed);
				} else if (Physics.Linecast (transform.position, originalPosition [index].transform.position, out hitCamFollow)) {
					if (CameraSettings.followPlayer.ignoreCollision) {
						cameras [index]._camera.transform.position = Vector3.Lerp (cameras [index]._camera.transform.position, originalPosition [index].transform.position, Time.deltaTime * CameraSettings.followPlayer.displacementSpeed);
					} else {
						cameras [index]._camera.transform.position = Vector3.Lerp (cameras [index]._camera.transform.position, hitCamFollow.point, Time.deltaTime * CameraSettings.followPlayer.displacementSpeed);
					}
				}
			}
			//rotation
			if (CameraSettings.followPlayer.customLookAt) {
				Quaternion nextRotation = Quaternion.LookRotation (targetTransform.position - cameras [index]._camera.transform.position, Vector3.up);
				cameras [index]._camera.transform.rotation = Quaternion.Slerp (cameras [index]._camera.transform.rotation, nextRotation, Time.deltaTime * CameraSettings.followPlayer.spinSpeedCustomLookAt);
			} else {
				cameras [index]._camera.transform.LookAt (targetTransform.position);
			}
			break;
		case MSACC_CameraType.TipoRotac.Orbital:
			//raycast hit
			RaycastHit hitCamOrbital;
			float minDistance = CameraSettings.orbital.minDistance;
			if (Physics.Linecast (targetTransform.position, cameras [index]._camera.transform.position, out hitCamOrbital)) {
				if (!CameraSettings.orbital.ignoreCollision) {
					distanceFromOrbitalCamera [index] = Vector3.Distance (targetTransform.position, hitCamOrbital.point);
					minDistance = Mathf.Clamp (distanceFromOrbitalCamera [index], minDistance * 0.5f, CameraSettings.orbital.maxDistance);
				}
			}
			//getInputs
			float xInputOrb = _horizontalInputMSACC;
			float yInputOrb = _verticalInputMSACC;
			if (CameraSettings.orbital.invertXInput) {
				xInputOrb = -_horizontalInputMSACC;
			}
			if (CameraSettings.orbital.invertYInput) {
				yInputOrb = -_verticalInputMSACC;
			}
			if (CameraSettings.orbital.rotateWhenClick) {
				if (Input.GetKey (CameraSettings.orbital.keyToRotate) || _enableMobileInputs) {
					xOrbit [index] += xInputOrb * (CameraSettings.orbital.sensibility * distanceFromOrbitalCamera [index]) / (distanceFromOrbitalCamera [index] * 0.5f);
					yOrbit [index] -= yInputOrb * CameraSettings.orbital.sensibility * (CameraSettings.orbital.speedYAxis * 10.0f);
				}
			} else {
				xOrbit [index] += xInputOrb * (CameraSettings.orbital.sensibility * distanceFromOrbitalCamera [index]) / (distanceFromOrbitalCamera [index] * 0.5f);
				yOrbit [index] -= yInputOrb * CameraSettings.orbital.sensibility * (CameraSettings.orbital.speedYAxis * 10.0f);
			}
			//move - rotation
			yOrbit [index] = ClampAngle (yOrbit [index], CameraSettings.orbital.minAngleY, CameraSettings.orbital.maxAngleY);
			Quaternion quatToEuler = Quaternion.Euler (yOrbit [index], xOrbit [index], 0);
			distanceFromOrbitalCamera [index] = Mathf.Clamp (distanceFromOrbitalCamera [index] - _scrollInputMSACC * (CameraSettings.orbital.speedScrool * 50.0f), minDistance, CameraSettings.orbital.maxDistance);
			Vector3 zPosition = new Vector3 (0.0f, 0.0f, -distanceFromOrbitalCamera [index]);
			Vector3 nextPosCam = quatToEuler * zPosition + targetTransform.position;
			Vector3 currentPosCam = cameras [index]._camera.transform.position;
			Quaternion camRotation = cameras [index]._camera.transform.rotation;
			cameras [index]._camera.transform.rotation = Quaternion.Lerp(camRotation, quatToEuler, Time.deltaTime * 5.0f * timeScaleSpeed);
			cameras [index]._camera.transform.position = Vector3.Lerp(currentPosCam, nextPosCam, Time.deltaTime * 5.0f * timeScaleSpeed);
			break;
		case MSACC_CameraType.TipoRotac.OrbitalThatFollows:
			float movXInput = 0.0f;
			float movYInput = 0.0f;
			float movZInput = _scrollInputMSACC;
			if (CameraSettings.OrbitalThatFollows.rotateWhenClick) {
				if (Input.GetKey (CameraSettings.OrbitalThatFollows.keyToRotate) || _enableMobileInputs) {
					movXInput = _horizontalInputMSACC;
					movYInput = _verticalInputMSACC;
					if (CameraSettings.OrbitalThatFollows.invertXInput) {
						movXInput = -_horizontalInputMSACC;
					}
					if (CameraSettings.OrbitalThatFollows.invertYInput) {
						movYInput = -_verticalInputMSACC;
					}
				}
			} else {
				movXInput = _horizontalInputMSACC;
				movYInput = _verticalInputMSACC;
				if (CameraSettings.OrbitalThatFollows.invertXInput) {
					movXInput = -_horizontalInputMSACC;
				}
				if (CameraSettings.OrbitalThatFollows.invertYInput) {
					movYInput = -_verticalInputMSACC;
				}
			}
			//
			if (movXInput > 0.0f || movYInput > 0.0f || movZInput > 0.0f) {
				orbitalAtiv = true;
				tempoOrbit = 0.0f;
				if (!orbital_AtivTemp) {
					orbital_AtivTemp = true;
					xOrbit [index] = cameras [index]._camera.transform.eulerAngles.y;
					yOrbit [index] = cameras [index]._camera.transform.eulerAngles.x;
				}
			} else {
				tempoOrbit += Time.deltaTime;
				if (tempoOrbit > CameraSettings.OrbitalThatFollows.timeToReset) {
					tempoOrbit = CameraSettings.OrbitalThatFollows.timeToReset + 0.1f;
				}
			}
			//
			switch (CameraSettings.OrbitalThatFollows.ResetControlType) {
			case MSACC_SettingsCameraOrbitalThatFollows.ResetTimeType.Time:
				if (tempoOrbit > CameraSettings.OrbitalThatFollows.timeToReset) {
					orbitalAtiv = false;
					orbital_AtivTemp = false;
				}
				break;
			case MSACC_SettingsCameraOrbitalThatFollows.ResetTimeType.Input_OnlyWindows:
				if (Input.GetKeyDown(CameraSettings.OrbitalThatFollows.resetKey)) {
					orbitalAtiv = false;
					orbital_AtivTemp = false;
				}
				break;
			}
			//
			RaycastHit hitCamOTS;
			if(orbitalAtiv == true){
				float _minDistance = CameraSettings.OrbitalThatFollows.minDistance;
				if (Physics.Linecast (targetTransform.position, cameras [index]._camera.transform.position, out hitCamOTS)) {
					if (!CameraSettings.OrbitalThatFollows.ignoreCollision) {
						distanceFromOrbitalCamera [index] = Vector3.Distance (targetTransform.position, hitCamOTS.point);
						_minDistance = Mathf.Clamp (distanceFromOrbitalCamera [index], _minDistance * 0.5f, CameraSettings.OrbitalThatFollows.maxDistance);
					}
				}
				xOrbit [index] += movXInput * (CameraSettings.OrbitalThatFollows.sensibility * distanceFromOrbitalCamera [index]) / (distanceFromOrbitalCamera [index] * 0.5f);
				yOrbit [index] -= movYInput * CameraSettings.OrbitalThatFollows.sensibility * (CameraSettings.OrbitalThatFollows.speedYAxis * 10.0f);
				yOrbit [index] = ClampAngle (yOrbit [index], CameraSettings.OrbitalThatFollows.minAngleY, CameraSettings.OrbitalThatFollows.maxAngleY);
				Quaternion quaterToEuler = Quaternion.Euler (yOrbit [index], xOrbit [index], 0);
				distanceFromOrbitalCamera [index] = Mathf.Clamp (distanceFromOrbitalCamera [index] - movZInput * (CameraSettings.OrbitalThatFollows.speedScrool * 50.0f), _minDistance, CameraSettings.OrbitalThatFollows.maxDistance);
				Vector3 _zPosition = new Vector3 (0.0f, 0.0f, -distanceFromOrbitalCamera [index]);
				Vector3 _camNewPos = quaterToEuler * _zPosition + targetTransform.position;
				Vector3 _camCurrentPos = cameras [index]._camera.transform.position;
				Quaternion camRot = cameras [index]._camera.transform.rotation;
				cameras [index]._camera.transform.rotation = Quaternion.Lerp (camRot, quaterToEuler, Time.deltaTime * 5.0f * timeScaleSpeed);
				cameras [index]._camera.transform.position = Vector3.Lerp (_camCurrentPos, _camNewPos, Time.deltaTime * 5.0f * timeScaleSpeed);
			} else {
				if (!Physics.Linecast (targetTransform.position, originalPosition [index].transform.position)) {
					cameras [index]._camera.transform.position = Vector3.Lerp (cameras [index]._camera.transform.position, originalPosition [index].transform.position, Time.deltaTime * CameraSettings.OrbitalThatFollows.displacementSpeed);
				}
				else if(Physics.Linecast(targetTransform.position, originalPosition [index].transform.position,out hitCamOTS)){
					if (CameraSettings.OrbitalThatFollows.ignoreCollision) {
						cameras [index]._camera.transform.position = Vector3.Lerp (cameras [index]._camera.transform.position, originalPosition [index].transform.position, Time.deltaTime * CameraSettings.OrbitalThatFollows.displacementSpeed);
					} 
					else {
						cameras [index]._camera.transform.position = Vector3.Lerp(cameras [index]._camera.transform.position, hitCamOTS.point,Time.deltaTime * CameraSettings.OrbitalThatFollows.displacementSpeed);
					}
				}
				//
				if (CameraSettings.OrbitalThatFollows.customLookAt) {
					Quaternion quatLookRot = Quaternion.LookRotation (targetTransform.position - cameras [index]._camera.transform.position, Vector3.up);
					cameras [index]._camera.transform.rotation = Quaternion.Slerp (cameras [index]._camera.transform.rotation, quatLookRot, Time.deltaTime * CameraSettings.OrbitalThatFollows.spinSpeedCustomLookAt);
				} else {
					cameras [index]._camera.transform.LookAt (targetTransform.position);
				}
			}
			break;
		case MSACC_CameraType.TipoRotac.ETS_StyleCamera:
			float xInputEts = _horizontalInputMSACC;
			float yInputEts = _verticalInputMSACC; 
			if (CameraSettings.ETS_StyleCamera.invertXInput) {
				xInputEts = -_horizontalInputMSACC;
			}
			if (CameraSettings.ETS_StyleCamera.invertYInput) {
				yInputEts = -_verticalInputMSACC;
			}
			if (CameraSettings.ETS_StyleCamera.rotateWhenClick) {
				if (Input.GetKey (CameraSettings.ETS_StyleCamera.keyToRotate) || _enableMobileInputs) {
					rotacXETS += xInputEts * CameraSettings.ETS_StyleCamera.sensibilityX;
					rotacYETS += yInputEts * CameraSettings.ETS_StyleCamera.sensibilityY;
				}
			} else {
				rotacXETS += xInputEts * CameraSettings.ETS_StyleCamera.sensibilityX;
				rotacYETS += yInputEts * CameraSettings.ETS_StyleCamera.sensibilityY;
			}
			Vector3 newPositionETS = new Vector3 (originalPositionETS [index].x + Mathf.Clamp (rotacXETS / 50 + (CameraSettings.ETS_StyleCamera.ETS_CameraShift/3.0f), -CameraSettings.ETS_StyleCamera.ETS_CameraShift, 0), originalPositionETS [index].y, originalPositionETS [index].z);
			cameras [index]._camera.transform.localPosition = Vector3.Lerp (cameras [index]._camera.transform.localPosition, newPositionETS, Time.deltaTime * 10.0f);
			rotacXETS = ClampAngle (rotacXETS, -180, 80);
			rotacYETS = ClampAngle (rotacYETS, -60, 60);
			Quaternion _xQuaternion = Quaternion.AngleAxis (rotacXETS, Vector3.up);
			Quaternion _yQuaternion = Quaternion.AngleAxis (rotacYETS, -Vector3.right);
			Quaternion nextRot = originalRotation [index] * _xQuaternion * _yQuaternion;
			cameras [index]._camera.transform.localRotation = Quaternion.Lerp (cameras [index]._camera.transform.localRotation, nextRot, Time.deltaTime * 10.0f * timeScaleSpeed);
			//fieldOfView
			cameras [index]._camera.fieldOfView -= _scrollInputMSACC * CameraSettings.ETS_StyleCamera.speedScroolZoom * 50.0f;
			if (cameras [index]._camera.fieldOfView < (initialFieldOfView [index] - CameraSettings.ETS_StyleCamera.maxScroolZoom)) {
				cameras [index]._camera.fieldOfView = (initialFieldOfView [index] - CameraSettings.ETS_StyleCamera.maxScroolZoom);
			}
			if (cameras [index]._camera.fieldOfView > initialFieldOfView [index]) {
				cameras [index]._camera.fieldOfView = (initialFieldOfView [index]);
			}
			break;
		case MSACC_CameraType.TipoRotac.FlyCamera_OnlyWindows:
			float xInputFly = _horizontalInputMSACC;
			float yInputFly = _verticalInputMSACC; 
			if (CameraSettings.FlyCamera_OnlyWindows.invertXInput) {
				xInputFly = -_horizontalInputMSACC;
			}
			if (CameraSettings.FlyCamera_OnlyWindows.invertYInput) {
				yInputFly = -_verticalInputMSACC;
			}
			//
			cameraRotationFly.x += xInputFly * CameraSettings.FlyCamera_OnlyWindows.sensibilityX * 15 * Time.deltaTime;
			cameraRotationFly.y += yInputFly * CameraSettings.FlyCamera_OnlyWindows.sensibilityY * 15 * Time.deltaTime;
			cameraRotationFly.y = Mathf.Clamp (cameraRotationFly.y, -90, 90);
			cameras [index]._camera.transform.rotation = Quaternion.AngleAxis (cameraRotationFly.x, Vector3.up);
			cameras [index]._camera.transform.rotation *= Quaternion.AngleAxis (cameraRotationFly.y, Vector3.left);
			//
			float speedCamFly = CameraSettings.FlyCamera_OnlyWindows.movementSpeed;
			if (Input.GetKey (CameraSettings.FlyCamera_OnlyWindows.speedKeyCode)) {
				speedCamFly *= 3.0f;
			}
			cameras [index]._camera.transform.position += cameras [index]._camera.transform.right * speedCamFly * Input.GetAxis(CameraSettings.FlyCamera_OnlyWindows.horizontalMove) * Time.deltaTime;
			cameras [index]._camera.transform.position += cameras [index]._camera.transform.forward * speedCamFly * Input.GetAxis(CameraSettings.FlyCamera_OnlyWindows.verticalMove) * Time.deltaTime;
			//
			if(Input.GetKey(CameraSettings.FlyCamera_OnlyWindows.moveUp)){
				cameras [index]._camera.transform.position += Vector3.up * speedCamFly * Time.deltaTime;
			}
			if(Input.GetKey(CameraSettings.FlyCamera_OnlyWindows.moveDown)){
				cameras [index]._camera.transform.position -= Vector3.up * speedCamFly * Time.deltaTime;
			}
			break;
		}
	}

	public static float ClampAngle (float angle, float min, float max){
		if (angle < -360F) { angle += 360F; }
		if (angle > 360F) { angle -= 360F; }
		return Mathf.Clamp (angle, min, max);
	}
	 
	public void MSADCCChangeCameras(){ //use this void to change cameras using buttons
		if (Time.timeScale > 0) {
			if (index < (cameras.Length - 1)) {
				lastIndex = index;
				index++;
				EnableCameras (index);
			} else if (index >= (cameras.Length - 1)) {
				lastIndex = index;
				index = 0;
				EnableCameras (index);
			}
		}
	}

	void Update(){
		
		// Get inputs
		if (!_enableMobileInputs) {
			_horizontalInputMSACC = Input.GetAxis (CameraSettings.inputMouseX);
			_verticalInputMSACC = Input.GetAxis (CameraSettings.inputMouseY);
			_scrollInputMSACC = Input.GetAxis (CameraSettings.inputMouseScrollWheel);
		} else {
			//GetComponent<MSCameraController>()._horizontalInputMSACC = InputX;
			//GetComponent<MSCameraController>()._verticalInputMSACC = InputY;
			//GetComponent<MSCameraController>()._scrollInputMSACC = InputScroll;
		}
		_horizontalInputMSACC = Mathf.Clamp (_horizontalInputMSACC, -1, 1);
		_verticalInputMSACC = Mathf.Clamp (_verticalInputMSACC, -1, 1);
		_scrollInputMSACC = Mathf.Clamp (_scrollInputMSACC, -1, 1);
		//


		//camera switch key
		if (!_enableMobileInputs) {
			if (Time.timeScale > 0) {
				if (Input.GetKeyDown (CameraSettings.cameraSwitchKey) && index < (cameras.Length - 1)) {
					lastIndex = index;
					index++;
					EnableCameras (index);
				} else if (Input.GetKeyDown (CameraSettings.cameraSwitchKey) && index >= (cameras.Length - 1)) {
					lastIndex = index;
					index = 0;
					EnableCameras (index);
				}
			}
		}


		//update cameras
		if (CameraSettings.camerasUpdateMode == MSACC_CameraSetting.UpdateMode.Update) {
			if (cameras.Length > 0 && Time.timeScale > 0) {
				if (cameras [index]._camera) {
					ManageCameras ();
				}
			}
		}
	}

	void LateUpdate(){
		//update cameras
		if (CameraSettings.camerasUpdateMode == MSACC_CameraSetting.UpdateMode.LateUpdate) {
			if (cameras.Length > 0 && Time.timeScale > 0) {
				if (cameras [index]._camera) {
					ManageCameras ();
				}
			}
		}
	}

	void FixedUpdate(){
		//update cameras
		if (CameraSettings.camerasUpdateMode == MSACC_CameraSetting.UpdateMode.FixedUpdate) {
			if (cameras.Length > 0 && Time.timeScale > 0) {
				if (cameras [index]._camera) {
					ManageCameras ();
				}
			}
		}
	}
}