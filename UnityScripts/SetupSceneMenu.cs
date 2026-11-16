#if UNITY_EDITOR
using System;
using System.Reflection;
using UnityEditor;
using UnityEditor.PackageManager;
using UnityEngine;

namespace PsychophysicsVR.Tracking
{
    public static class SetupSceneMenu
    {
        [InitializeOnLoadMethod]
        private static void OnEditorScriptsReloaded()
        {
            EditorApplication.delayCall += () => EnsureOpenXRControllerProfiles(silent: true);
        }

        [MenuItem("Psychophysics VR/Auto-Setup Arm Tracker Scene", false, 1)]
        public static void AutoSetupScene()
        {
            EnsureOpenXRControllerProfiles(silent: false);

            Camera mainCam = Camera.main != null ? Camera.main : UnityEngine.Object.FindFirstObjectByType<Camera>();
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
            VRDashboardAndController vrUi = managerGo.GetComponent<VRDashboardAndController>() ?? Undo.AddComponent<VRDashboardAndController>(managerGo);

            receiver.avatarShoulderAnchor = rShoulderGo.transform;
            receiver.leftAvatarShoulderAnchor = lShoulderGo.transform;
            receiver.cameraOrientationReference = camOrientGo.transform;
            controller.receiver = receiver;
            controller.lockShoulderToHeadsetYaw = true;
            vrUi.controller = controller;
            vrUi.receiver = receiver;

            Selection.activeGameObject = managerGo;
            EditorUtility.SetDirty(receiver);
            EditorUtility.SetDirty(controller);
            EditorUtility.SetDirty(vrUi);

            Debug.Log("✅ [Psychophysics VR v6.3] Scene Setup & OpenXR Controller Profiles Ready!");
            EditorUtility.DisplayDialog(
                "Psychophysics VR v6.3 Setup Complete!",
                "Ready with:\n" +
                "• Guided 8s 6-DOF 3D Path Calibration + Lag-Compensated Procrustes Solver\n" +
                "• Continuous Multi-Backend Pimax Controller Pose & Wand Tracking\n" +
                "• 3-Second Gaze-Dwell Auto-Click In-Headset 3D Dashboard\n" +
                "• Rigid-Link Kalman + CDSO PD Joint Controller",
                "Awesome!"
            );
        }

        [MenuItem("Psychophysics VR/Enable OpenXR Controller Profiles (Pimax / Oculus Touch)", false, 2)]
        public static void MenuEnableOpenXRProfiles()
        {
            int enabledCount = EnsureOpenXRControllerProfiles(silent: false);
            EditorUtility.DisplayDialog(
                "OpenXR Controller Interaction Profiles",
                enabledCount > 0
                    ? $"Enabled {enabledCount} OpenXR Controller Profile(s) (Oculus Touch / KHR / Vive / Index)!"
                    : "Checked OpenXR Settings — controller profiles are already enabled (or OpenXR package is not yet installed).",
                "OK"
            );
        }

        [MenuItem("Psychophysics VR/Install Official OpenXR Hands Package (com.unity.xr.hands)", false, 3)]
        public static void InstallOpenXRHandsPackage()
        {
            Debug.Log("[Psychophysics VR] Installing com.unity.xr.hands via Unity Package Manager...");
            Client.Add("com.unity.xr.hands");
            EditorUtility.DisplayDialog(
                "Installing com.unity.xr.hands",
                "Unity Package Manager is now downloading 'com.unity.xr.hands'.",
                "OK"
            );
        }

        /// <summary>
        /// Uses reflection so there is zero compile dependency if com.unity.xr.openxr isn't installed yet.
        /// Automatically enables OculusTouchControllerProfile, KHRSimpleControllerProfile,
        /// HTCViveControllerProfile, and ValveIndexControllerProfile when available.
        /// </summary>
        public static int EnsureOpenXRControllerProfiles(bool silent)
        {
            int newlyEnabled = 0;
            try
            {
                Type settingsType = null;
                foreach (Assembly asm in AppDomain.CurrentDomain.GetAssemblies())
                {
                    settingsType = asm.GetType("UnityEngine.XR.OpenXR.OpenXRSettings");
                    if (settingsType != null) break;
                }
                if (settingsType == null) return 0;

                PropertyInfo instProp = settingsType.GetProperty("Instance", BindingFlags.Public | BindingFlags.Static);
                object settingsObj = instProp?.GetValue(null);
                if (settingsObj == null)
                {
                    MethodInfo getSettings = settingsType.GetMethod("GetSettingsForBuildTargetGroup", BindingFlags.Public | BindingFlags.Static);
                    if (getSettings != null)
                        settingsObj = getSettings.Invoke(null, new object[] { BuildTargetGroup.Standalone });
                }
                if (settingsObj == null) return 0;

                MethodInfo getFeatures = settingsType.GetMethod("GetFeatures", new[] { typeof(Type) });
                FieldInfo featuresField = settingsType.GetField("features", BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Public);
                Array features = null;

                if (featuresField != null)
                    features = featuresField.GetValue(settingsObj) as Array;

                if (features == null && getFeatures != null)
                {
                    Type featureBaseType = null;
                    foreach (Assembly asm in AppDomain.CurrentDomain.GetAssemblies())
                    {
                        featureBaseType = asm.GetType("UnityEngine.XR.OpenXR.Features.OpenXRFeature");
                        if (featureBaseType != null) break;
                    }
                    if (featureBaseType != null)
                        features = getFeatures.Invoke(settingsObj, new object[] { featureBaseType }) as Array;
                }
                if (features == null) return 0;

                string[] targetProfiles =
                {
                    "OculusTouchControllerProfile",
                    "KHRSimpleControllerProfile",
                    "HTCViveControllerProfile",
                    "ValveIndexControllerProfile"
                };

                foreach (object feat in features)
                {
                    if (feat == null) continue;
                    string typeName = feat.GetType().Name;
                    bool isTarget = false;
                    foreach (string tp in targetProfiles)
                    {
                        if (string.Equals(typeName, tp, StringComparison.Ordinal))
                        {
                            isTarget = true;
                            break;
                        }
                    }
                    if (!isTarget) continue;

                    PropertyInfo enabledProp = feat.GetType().GetProperty("enabled", BindingFlags.Public | BindingFlags.Instance);
                    if (enabledProp != null && enabledProp.CanRead && enabledProp.CanWrite)
                    {
                        bool currentlyEnabled = (bool)enabledProp.GetValue(feat);
                        if (!currentlyEnabled)
                        {
                            enabledProp.SetValue(feat, true);
                            if (feat is UnityEngine.Object uo)
                                EditorUtility.SetDirty(uo);
                            newlyEnabled++;
                            Debug.Log($"[Psychophysics VR] Auto-enabled OpenXR Controller Profile: {typeName}");
                        }
                    }
                }

                if (newlyEnabled > 0)
                {
                    if (settingsObj is UnityEngine.Object settingsUo)
                        EditorUtility.SetDirty(settingsUo);
                    AssetDatabase.SaveAssets();
                }
            }
            catch (Exception ex)
            {
                if (!silent)
                    Debug.LogWarning($"[Psychophysics VR] OpenXR profile auto-check note: {ex.Message}");
            }
            return newlyEnabled;
        }
    }
}
#endif
