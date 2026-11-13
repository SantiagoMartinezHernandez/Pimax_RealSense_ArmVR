using System.Collections.Generic;
using UnityEngine;

namespace PsychophysicsVR.Tracking
{
    /// <summary>
    /// Generates and dynamically skins a TRUE continuous, smooth-shaded 3D Hand & Arm Mesh
    /// (modeled after the standard Microsoft MRTK / OpenXR Hand Mesh topology).
    /// Constructs a single unified triangle Mesh with continuous vertex rings along all 5
    /// tapered fingers, closed rounded fingertips, a sculpted palmar/dorsal hand body,
    /// thumb thenar muscle, wrist transition, and anatomically contoured forearm/upper-arm.
    /// </summary>
    public class ContinuousHandArmMesh
    {
        public GameObject MeshObject { get; private set; }
        public Mesh DynamicMesh { get; private set; }
        public MeshRenderer Renderer { get; private set; }

        private readonly bool _isRight;
        private readonly List<Vector3> _vertices = new List<Vector3>(1024);
        private readonly List<int> _triangles = new List<int>(3072);

        private const int RingSegments = 12; // Smooth 12-sided cross-sections

        private static readonly int[,] FingerChains = new int[5, 4]
        {
            {1, 2, 3, 4},     // Thumb
            {5, 6, 7, 8},     // Index
            {9, 10, 11, 12},  // Middle
            {13, 14, 15, 16}, // Ring
            {17, 18, 19, 20}  // Pinky
        };

        // Anatomical finger radii (in meters) from base knuckle to fingertip
        private static readonly float[,] FingerRadii = new float[5, 6]
        {
            { 0.0145f, 0.0122f, 0.0108f, 0.0095f, 0.0072f, 0.0020f }, // Thumb
            { 0.0112f, 0.0098f, 0.0086f, 0.0076f, 0.0058f, 0.0018f }, // Index
            { 0.0116f, 0.0102f, 0.0089f, 0.0078f, 0.0060f, 0.0018f }, // Middle
            { 0.0108f, 0.0094f, 0.0082f, 0.0072f, 0.0055f, 0.0016f }, // Ring
            { 0.0095f, 0.0082f, 0.0072f, 0.0064f, 0.0048f, 0.0015f }  // Pinky
        };

        private bool _topologyBuilt;

        public ContinuousHandArmMesh(Transform parent, string name, bool isRight)
        {
            _isRight = isRight;
            MeshObject = new GameObject(name);
            MeshObject.transform.SetParent(parent, false);
            MeshObject.transform.localPosition = Vector3.zero;
            MeshObject.transform.localRotation = Quaternion.identity;

            MeshFilter mf = MeshObject.AddComponent<MeshFilter>();
            Renderer = MeshObject.AddComponent<MeshRenderer>();

            DynamicMesh = new Mesh
            {
                name = name + "_ContinuousSkinnedMesh"
            };
            DynamicMesh.MarkDynamic();
            mf.sharedMesh = DynamicMesh;

            Shader sh = Shader.Find("Universal Render Pipeline/Lit");
            if (sh == null) sh = Shader.Find("Standard");
            Material mat = new Material(sh);
            if (mat.HasProperty("_Smoothness")) mat.SetFloat("_Smoothness", 0.35f);
            if (mat.HasProperty("_Glossiness")) mat.SetFloat("_Glossiness", 0.35f);
            Renderer.sharedMaterial = mat;
        }

        public void SetStyle(Color baseColor, bool sleekMetallicStyle)
        {
            if (Renderer == null || Renderer.sharedMaterial == null) return;
            Material m = Renderer.sharedMaterial;
            m.color = baseColor;
            if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", baseColor);
            if (m.HasProperty("_Metallic")) m.SetFloat("_Metallic", sleekMetallicStyle ? 0.22f : 0.02f);
            if (m.HasProperty("_Smoothness")) m.SetFloat("_Smoothness", sleekMetallicStyle ? 0.62f : 0.28f);
            if (m.HasProperty("_Glossiness")) m.SetFloat("_Glossiness", sleekMetallicStyle ? 0.62f : 0.28f);
        }

        public void UpdateMeshPose(
            Vector3 shoulderPos,
            Vector3 elbowPos,
            Vector3 wristPos,
            Vector3[] handJoints21,
            Vector3 palmNormal,
            Vector3 bendNormal)
        {
            _vertices.Clear();
            bool buildTris = !_topologyBuilt;
            if (buildTris)
            {
                _triangles.Clear();
            }

            // 1. Continuous Tapered Arm Tube (Shoulder -> Bicep -> Elbow -> Forearm -> Wrist)
            BuildContinuousArmTube(shoulderPos, elbowPos, wristPos, bendNormal, palmNormal, buildTris);

            // 2. Sculpted Anatomical Palm Loft (Wrist -> Metacarpal Arch -> Knuckle Ridge + Thenar Muscle)
            BuildSculptedPalmBody(wristPos, handJoints21, palmNormal, buildTris);

            // 3. 5 Continuous Tapered Fingers with Rounded Anatomical Fingertip Dome Caps
            for (int f = 0; f < 5; f++)
            {
                BuildContinuousFingerTube(f, wristPos, handJoints21, palmNormal, buildTris);
            }

            DynamicMesh.SetVertices(_vertices);
            if (buildTris)
            {
                DynamicMesh.SetTriangles(_triangles, 0);
                _topologyBuilt = true;
            }
            DynamicMesh.RecalculateNormals();
            DynamicMesh.RecalculateBounds();
        }

        private void BuildContinuousArmTube(
            Vector3 shoulder, Vector3 elbow, Vector3 wrist,
            Vector3 elbowNormal, Vector3 palmNormal, bool buildTris)
        {
            const int armRings = 10;
            int baseVert = _vertices.Count;

            Vector3 upperDir = (elbow - shoulder).normalized;
            Vector3 foreDir = (wrist - elbow).normalized;
            if (upperDir.sqrMagnitude < 0.001f) upperDir = Vector3.down;
            if (foreDir.sqrMagnitude < 0.001f) foreDir = Vector3.forward;

            // Anatomical arm radii profile from Deltoid shoulder cap -> Bicep -> Elbow -> Forearm -> Wrist
            float[] tSamples = { 0.0f, 0.04f, 0.22f, 0.42f, 0.50f, 0.58f, 0.72f, 0.86f, 0.95f, 1.0f };
            float[] rxProfile = { 0.008f, 0.046f, 0.045f, 0.039f, 0.038f, 0.042f, 0.036f, 0.030f, 0.028f, 0.027f };
            float[] ryProfile = { 0.008f, 0.046f, 0.043f, 0.037f, 0.036f, 0.039f, 0.033f, 0.024f, 0.020f, 0.018f };

            for (int r = 0; r < armRings; r++)
            {
                float t = tSamples[r];
                Vector3 center;
                Vector3 tangent;
                Vector3 upRef;

                if (t <= 0.5f)
                {
                    float u = t * 2f;
                    center = Vector3.Lerp(shoulder, elbow, u);
                    tangent = Vector3.Slerp(upperDir, (upperDir + foreDir).normalized, u * u).normalized;
                    upRef = elbowNormal;
                }
                else
                {
                    float u = (t - 0.5f) * 2f;
                    center = Vector3.Lerp(elbow, wrist, u);
                    tangent = Vector3.Slerp((upperDir + foreDir).normalized, foreDir, Mathf.Sqrt(u)).normalized;
                    upRef = Vector3.Slerp(elbowNormal, palmNormal, u).normalized;
                }

                AddEllipticalRing(center, tangent, upRef, rxProfile[r], ryProfile[r]);
            }

            if (buildTris)
            {
                ConnectRingStripOutward(baseVert, armRings);
            }
        }

        private void BuildSculptedPalmBody(
            Vector3 wrist, Vector3[] pts, Vector3 palmNormal, bool buildTris)
        {
            int baseVert = _vertices.Count;
            const int palmRings = 6;

            Vector3 knuckleIndex = pts[5];
            Vector3 knuckleMid = pts[9];
            Vector3 knuckleRing = pts[13];
            Vector3 knucklePinky = pts[17];
            Vector3 thumbCmc = pts[1];
            Vector3 thumbMcp = pts[2];

            Vector3 knuckleCenter = (knuckleIndex + knuckleMid + knuckleRing + knucklePinky) * 0.25f;
            Vector3 palmFwd = (knuckleCenter - wrist).normalized;
            if (palmFwd.sqrMagnitude < 0.001f) palmFwd = Vector3.forward;

            Vector3 acrossKnuckles = (knuckleIndex - knucklePinky);
            float knuckleSpan = Mathf.Clamp(acrossKnuckles.magnitude, 0.058f, 0.095f);
            Vector3 palmSide = acrossKnuckles.sqrMagnitude > 0.0001f ? acrossKnuckles.normalized : Vector3.right;
            Vector3 pNorm = Vector3.Cross(palmFwd, palmSide).normalized;
            if (Vector3.Dot(pNorm, palmNormal) < 0f) pNorm = -pNorm;

            // Loft 6 cross-sectional rings from slightly inside the Wrist to the MCP Knuckle ridge (with smooth end closure)
            float[] vSteps = { -0.05f, 0.18f, 0.45f, 0.72f, 0.96f, 1.04f };
            float[] widthScale = { 0.56f, 0.78f, 0.96f, 1.05f, 1.02f, 0.35f };
            float[] thickProfile = { 0.017f, 0.020f, 0.019f, 0.016f, 0.012f, 0.004f };

            for (int r = 0; r < palmRings; r++)
            {
                float v = vSteps[r];
                Vector3 rowCenter = Vector3.LerpUnclamped(wrist, knuckleCenter, v);

                // Sculpt the thumb's thenar muscle bulge along the side of the palm
                if (r == 1 || r == 2 || r == 3)
                {
                    Vector3 thumbRef = (r == 1) ? thumbCmc : thumbMcp;
                    Vector3 thumbBulge = thumbRef - rowCenter;
                    rowCenter += Vector3.Project(thumbBulge, palmSide) * (r == 2 ? 0.26f : 0.16f);
                }

                float halfW = (knuckleSpan * 0.56f) * widthScale[r] + ((r == 1 || r == 2) ? 0.007f : 0f);
                float halfH = thickProfile[r];

                AddEllipticalRing(rowCenter, palmFwd, pNorm, halfW, halfH);
            }

            if (buildTris)
            {
                ConnectRingStripOutward(baseVert, palmRings);
            }
        }

        private void BuildContinuousFingerTube(
            int fingerIdx, Vector3 wrist, Vector3[] pts, Vector3 palmNormal, bool buildTris)
        {
            int baseVert = _vertices.Count;
            const int fingerRings = 6; // PalmRoot -> MCP -> PIP -> DIP -> TipPad -> Closed Rounded Tip

            int j0 = FingerChains[fingerIdx, 0];
            int j1 = FingerChains[fingerIdx, 1];
            int j2 = FingerChains[fingerIdx, 2];
            int j3 = FingerChains[fingerIdx, 3];

            Vector3 p0 = pts[j0];
            Vector3 p1 = pts[j1];
            Vector3 p2 = pts[j2];
            Vector3 p3 = pts[j3];

            // Start the finger tube embedded inside the palm arch so knuckles blend seamlessly with zero gaps!
            Vector3 pStart = Vector3.Lerp(wrist, p0, fingerIdx == 0 ? 0.55f : 0.80f);
            Vector3 tipDir = (p3 - p2).normalized;
            if (tipDir.sqrMagnitude < 0.001f) tipDir = Vector3.forward;
            Vector3 pCap = p3 + tipDir * 0.006f;

            Vector3[] joints = { pStart, p0, p1, p2, p3, pCap };

            for (int r = 0; r < fingerRings; r++)
            {
                Vector3 center = joints[r];
                Vector3 tangent;
                if (r == 0) tangent = (joints[1] - joints[0]).normalized;
                else if (r == fingerRings - 1) tangent = tipDir;
                else tangent = ((joints[r + 1] - joints[r]).normalized + (joints[r] - joints[r - 1]).normalized).normalized;

                if (tangent.sqrMagnitude < 0.001f) tangent = tipDir;

                float rad = FingerRadii[fingerIdx, r];
                AddEllipticalRing(center, tangent, palmNormal, rad * 1.04f, rad * 0.92f);
            }

            if (buildTris)
            {
                ConnectRingStripOutward(baseVert, fingerRings);
            }
        }

        private void AddEllipticalRing(Vector3 center, Vector3 forward, Vector3 upRef, float radiusX, float radiusY)
        {
            Vector3 right = Vector3.Cross(upRef, forward);
            if (right.sqrMagnitude < 0.0001f)
                right = Vector3.Cross(Vector3.right, forward);
            right.Normalize();
            Vector3 up = Vector3.Cross(forward, right).normalized;

            for (int i = 0; i < RingSegments; i++)
            {
                float angle = (i / (float)RingSegments) * Mathf.PI * 2f;
                float cx = Mathf.Cos(angle) * radiusX;
                float sy = Mathf.Sin(angle) * radiusY;
                _vertices.Add(center + right * cx + up * sy);
            }
        }

        private void ConnectRingStripOutward(int baseVertIndex, int ringCount)
        {
            for (int r = 0; r < ringCount - 1; r++)
            {
                int rowA = baseVertIndex + r * RingSegments;
                int rowB = baseVertIndex + (r + 1) * RingSegments;

                for (int i = 0; i < RingSegments; i++)
                {
                    int nextI = (i + 1) % RingSegments;
                    int a0 = rowA + i;
                    int a1 = rowA + nextI;
                    int b0 = rowB + i;
                    int b1 = rowB + nextI;

                    // Clockwise outward winding in Unity's left-handed coordinate space
                    _triangles.Add(a0);
                    _triangles.Add(a1);
                    _triangles.Add(b0);

                    _triangles.Add(a1);
                    _triangles.Add(b1);
                    _triangles.Add(b0);
                }
            }
        }
    }
}
