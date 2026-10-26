using System;
using System.Collections.Generic;
using UnityEngine;

namespace Microduck
{
    /// <summary>
    /// Drives the imported microduck.glb like the real robot: 14 joint angles (radians, MuJoCo order)
    /// plus an optional floating-base pose expressed in the MuJoCo world frame.
    /// Put it on the root "Microduck" GameObject of the imported GLB.
    /// The same API is what a live stream (sim now, real robot later) will call every frame.
    /// </summary>
    [ExecuteAlways]
    [DisallowMultipleComponent]
    public class MicroduckRig : MonoBehaviour
    {
        [Serializable]
        public struct JointSlider
        {
            [HideInInspector] public string name;
            [Range(-180f, 180f)] public float degrees;
        }

        [Tooltip("Pose applied in the editor and at start. Values are clamped to the robot's joint limits.")]
        public JointSlider[] joints = new JointSlider[0];

        [Tooltip("If on, the base (trunk_base) follows the keyframe / streamed base pose. " +
                 "Leave off to keep the duck where you placed it and only animate the joints.")]
        public bool driveBase = false;

        [Tooltip("Adds a BoxCollider that fits the duck, so Meta Interaction SDK grab/poke can find it.")]
        public bool addBoundsCollider = true;

        Transform[] _bodies;
        Transform _base, _headForward;
        readonly float[] _q = new float[14];   // current angles, radians

        public Transform HeadForward => _headForward;
        public Transform Base => _base;
        public int JointCount => MicroduckModel.Joints.Length;
        public static int IndexOf(string jointName) =>
            Array.FindIndex(MicroduckModel.Joints, j => j.Name == jointName);

        // ------------------------------------------------------------------ lifecycle
        void OnEnable()
        {
            Bind();
            if (joints == null || joints.Length != JointCount) ResetSlidersTo("STAND");
            ApplySliders();
            if (Application.isPlaying && addBoundsCollider) FitCollider();
        }

        void OnValidate()
        {
            if (_bodies == null) Bind();
            if (joints != null && joints.Length == JointCount) ApplySliders();
        }

        void Bind()
        {
            var all = new Dictionary<string, Transform>();
            foreach (var t in GetComponentsInChildren<Transform>(true))
                if (!all.ContainsKey(t.name)) all[t.name] = t;

            _bodies = new Transform[JointCount];
            for (int i = 0; i < JointCount; i++)
            {
                all.TryGetValue(MicroduckModel.Joints[i].Body, out _bodies[i]);
                if (_bodies[i] == null)
                    Debug.LogWarning($"[MicroduckRig] body '{MicroduckModel.Joints[i].Body}' not found under {name}. " +
                                     "Is this the root of the imported microduck.glb?", this);
            }
            all.TryGetValue(MicroduckModel.BaseBody, out _base);
            all.TryGetValue(MicroduckModel.HeadForward, out _headForward);
        }

        // ------------------------------------------------------------------ public API
        /// <summary>Joint angle in radians (MuJoCo convention), clamped to limits.</summary>
        public void SetJoint(int i, float radians)
        {
            var j = MicroduckModel.Joints[i];
            _q[i] = Mathf.Clamp(radians, j.MinRad, j.MaxRad);
            if (_bodies != null && _bodies[i] != null)
                _bodies[i].localRotation = j.Rest * Quaternion.AngleAxis(_q[i] * Mathf.Rad2Deg, Vector3.back);
        }

        public float GetJoint(int i) => _q[i];

        /// <summary>All 14 angles (radians) in policy / actuator order.</summary>
        public void SetJoints(IReadOnlyList<float> radians)
        {
            for (int i = 0; i < JointCount && i < radians.Count; i++) SetJoint(i, radians[i]);
        }

        /// <summary>Floating base pose from MuJoCo: position (x,y,z) and quaternion (w,x,y,z), world frame, Z-up.</summary>
        public void SetBasePoseMuJoCo(Vector3 pos, float qw, float qx, float qy, float qz)
        {
            if (_base == null) return;
            _base.localPosition = MjToUnityPosition(pos);
            _base.localRotation = MjToUnityRotation(qw, qx, qy, qz);
        }

        /// <summary>Apply a keyframe from scene.xml (STAND, INIT, SIT, FOLD).</summary>
        public void ApplyKeyframe(string key)
        {
            if (!MicroduckModel.Keyframes.TryGetValue(key, out var q)) { Debug.LogWarning($"No keyframe {key}"); return; }
            for (int i = 0; i < JointCount; i++) SetJoint(i, q[7 + i]);
            if (driveBase) SetBasePoseMuJoCo(new Vector3(q[0], q[1], q[2]), q[3], q[4], q[5], q[6]);
            SyncSlidersFromPose();
        }

        // Frame conversions. The GLB nests everything under "mujoco_world", whose glTFast import
        // mirrors X, so inside it MuJoCo vectors map as (x,y,z) -> (-x,y,z).
        public static Vector3 MjToUnityPosition(Vector3 p) => new Vector3(-p.x, p.y, p.z);
        public static Quaternion MjToUnityRotation(float w, float x, float y, float z) => new Quaternion(x, -y, -z, w);

        // ------------------------------------------------------------------ inspector sliders
        public void ResetSlidersTo(string key)
        {
            joints = new JointSlider[JointCount];
            MicroduckModel.Keyframes.TryGetValue(key, out var q);
            for (int i = 0; i < JointCount; i++)
                joints[i] = new JointSlider { name = MicroduckModel.Joints[i].Name,
                                              degrees = q != null ? q[7 + i] * Mathf.Rad2Deg : 0f };
        }

        void ApplySliders()
        {
            for (int i = 0; i < JointCount; i++)
            {
                var j = MicroduckModel.Joints[i];
                joints[i].name = j.Name;
                joints[i].degrees = Mathf.Clamp(joints[i].degrees, j.MinRad * Mathf.Rad2Deg, j.MaxRad * Mathf.Rad2Deg);
                SetJoint(i, joints[i].degrees * Mathf.Deg2Rad);
            }
        }

        public void SyncSlidersFromPose()
        {
            if (joints == null || joints.Length != JointCount) return;
            for (int i = 0; i < JointCount; i++) joints[i].degrees = _q[i] * Mathf.Rad2Deg;
        }

        [ContextMenu("Pose/STAND")] void CtxStand() => ApplyKeyframe("STAND");
        [ContextMenu("Pose/INIT (all zero)")] void CtxInit() => ApplyKeyframe("INIT");

        // ------------------------------------------------------------------ helpers
        void FitCollider()
        {
            var rends = GetComponentsInChildren<Renderer>();
            if (rends.Length == 0) return;
            var b = rends[0].bounds;
            foreach (var r in rends) b.Encapsulate(r.bounds);
            var box = GetComponent<BoxCollider>();
            if (box == null) box = gameObject.AddComponent<BoxCollider>();
            box.center = transform.InverseTransformPoint(b.center);
            var s = transform.lossyScale;
            box.size = new Vector3(b.size.x / s.x, b.size.y / s.y, b.size.z / s.z);
        }
    }
}
