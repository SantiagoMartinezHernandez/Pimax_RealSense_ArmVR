#if UNITY_EDITOR
using UnityEditor;
using UnityEditor.PackageManager;
using UnityEngine;

namespace PsychophysicsVR.Tracking
{
    public static class SetupSceneMenu
    {
        [MenuItem("Psychophysics VR/Auto-Setup Arm Tracker Scene", false, 1)]
        public static void AutoSetupScene()
        {
            Camera mainCam = Camera.main != null ? Camera.main : Object.FindFirstObjectByType<Camera>();
            if (mainCam == null)
            {
                GameObject camGo = new GameObject("Main Camera");
                camGo.tag = "MainCamera";
                mainCam = camGo.AddComponent<Camera>();
                camGo.transform.position = new Vector3(0f, 1.2f, 0f);
            }

            mainCam.nearClipPlane = 0.05f;

            GameObject rShoulderGo = GameObject.Find("RightShoulderAnchor") ?? new GameObject("RightShoulderAnchor");
            rShoulderGo.transform.SetParent(mainCam.transform.parent, true);
            rShoulderGo.transform.position = mainCam.transform.position + new Vector3(0.18f, -0.15f, 0.00f);

            GameObject lShoulderGo = GameObject.Find("LeftShoulderAnchor") ?? new GameObject("LeftShoulderAnchor");
            lShoulderGo.transform.SetParent(mainCam.transform.parent, true);
            lShoulderGo.transform.position = mainCam.transform.position + new Vector3(-0.18f, -0.15f, 0.00f);

            GameObject camOrientGo = GameObject.Find("RealSenseCameraOrientation") ?? new GameObject("RealSenseCameraOrientation");

            GameObject managerGo = GameObject.Find("ArmTrackingManager") ?? new GameObject("ArmTrackingManager");
            RealSenseArmReceiver receiver = managerGo.GetComponent<RealSenseArmReceiver>() ?? Undo.AddComponent<RealSenseArmReceiver>(managerGo);
            ArmAvatarController controller = managerGo.GetComponent<ArmAvatarController>() ?? Undo.AddComponent<ArmAvatarController>(managerGo);

            receiver.avatarShoulderAnchor = rShoulderGo.transform;
            receiver.leftAvatarShoulderAnchor = lShoulderGo.transform;
            receiver.cameraOrientationReference = camOrientGo.transform;
            controller.receiver = receiver;
            controller.lockShoulderToHeadsetYaw = true;

            Selection.activeGameObject = managerGo;
            EditorUtility.SetDirty(receiver);
            EditorUtility.SetDirty(controller);

            Debug.Log("✅ [Psychophysics VR v5.0] Scene Setup Complete!");
            EditorUtility.DisplayDialog(
                "Psychophysics VR v5.0 Setup Complete!",
                "Ready with:\n" +
                "• Continuous 3D Skinned Hand & Arm Mesh (Microsoft Holo-Style, Human Skin, & Bubble Joints — Press 'M')\n" +
                "• Cross-Product-Free Trigonometric IK & Impossible-Pose Filter\n" +
                "• Right, Left, and Dual-Arm support (Press 'R')\n" +
                "• Automatic Persistent Session Memory",
                "Awesome!"
            );
        }

        [MenuItem("Psychophysics VR/Install Official OpenXR Hands Package (com.unity.xr.hands)", false, 2)]
        public static void InstallOpenXRHandsPackage()
        {
            Debug.Log("[Psychophysics VR] Installing com.unity.xr.hands via Unity Package Manager...");
            Client.Add("com.unity.xr.hands");
            EditorUtility.DisplayDialog(
                "Installing com.unity.xr.hands",
                "Unity Package Manager is now downloading 'com.unity.xr.hands'.\n\n" +
                "Once installed, you can also import the 'HandVisualizer' sample from Window -> Package Manager -> XR Hands -> Samples, " +
                "and drag its Left/Right Hand Mesh Prefabs into the 'Custom Right/Left Hand Prefab' slots on ArmAvatarController!",
                "OK"
            );
        }
    }
}
#endif
