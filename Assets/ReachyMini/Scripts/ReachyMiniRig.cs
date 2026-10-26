using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ReachyMini
{
    /// <summary>
    /// Drives the imported reachy_mini.glb the way the SDK drives the real robot:
    /// a head pose (4x4, SDK frame), a body yaw and two antenna angles.
    /// The Stewart platform is solved with a port of Pollen's analytical IK
    /// (reachy_mini_rust_kinematics), so the six motor horns and rods move like the real ones,
    /// and a pose the robot can't reach is rejected exactly when the robot would reject it.
    ///
    /// SDK frame: origin at the neutral head position, x forward, y left, z up, meters / radians.
    /// Put this on the root "ReachyMini" GameObject of the imported GLB.
    /// </summary>
    [ExecuteAlways]
    [DisallowMultipleComponent]
    public class ReachyMiniRig : MonoBehaviour
    {
        [Header("Target (SDK frame, mm / degrees)")]
        public Vector3 headPositionMm;
        [Tooltip("roll (x), pitch (y), yaw (z), extrinsic xyz like the SDK's create_head_pose")]
        public Vector3 headRollPitchYawDeg;
        [Range(-160f, 160f)] public float bodyYawDeg;
        [Range(-180f, 180f)] public float rightAntennaDeg = -10f;
        [Range(-180f, 180f)] public float leftAntennaDeg = 10f;

        [Tooltip("Like the SDK default: the body turns along when the head yaws more than 65° relative to it.")]
        public bool automaticBodyYaw = true;

        [Tooltip("The real robot only refuses poses its rods can't reach. Turn this on to also refuse motor angles " +
                 "outside the MuJoCo model's ranges (tighter: the SDK's own SLEEP pose is outside them).")]
        public bool enforceSimJointRanges = false;

        [Tooltip("Adds a BoxCollider that fits the robot, so Meta Interaction SDK grab/poke can find it.")]
        public bool addBoundsCollider = true;

        [Header("Present state (read only)")]
        [SerializeField] bool reachable = true;
        [SerializeField] float[] motorAnglesDeg = new float[6];

        // ------------------------------------------------------------------ bound transforms
        Transform _bodyYawT, _head, _headForward, _neutralOrigin;
        readonly Transform[] _horns = new Transform[6], _rods = new Transform[6], _tips = new Transform[6], _anchors = new Transform[6];
        readonly Transform[] _antennas = new Transform[2];
        bool _bound;

        // ------------------------------------------------------------------ present state
        readonly double[] _pose = Identity();          // SDK head pose, row-major
        float _bodyYaw;                                // rad (after automatic body yaw)
        float _right, _left;                           // rad, SDK convention
        readonly double[] _motors = new double[6];     // rad
        readonly double[] _scratch = new double[6];

        public Transform Head => _head;                       // xl_330 (head body)
        public Transform HeadForward => _headForward;         // +Z = where the face looks, at the camera
        public Transform NeutralHeadOrigin => _neutralOrigin; // the SDK's world frame
        public bool IsReachable => reachable;
        public float BodyYaw => _bodyYaw;
        public float RightAntenna => _right;
        public float LeftAntenna => _left;
        public double MotorAngle(int i) => _motors[i];
        public Matrix4x4 HeadPose => ToMatrix(_pose);

        // ------------------------------------------------------------------ lifecycle
        void OnEnable()
        {
            Bind();
            ApplyInspector();
            if (Application.isPlaying && addBoundsCollider) FitCollider();
        }

        void OnValidate()
        {
            if (!_bound) Bind();
            if (_bound) ApplyInspector();
        }

        void Bind()
        {
            var all = new Dictionary<string, Transform>();
            foreach (var t in GetComponentsInChildren<Transform>(true))
                if (!all.ContainsKey(t.name)) all[t.name] = t;
            Transform Get(string n)
            {
                if (all.TryGetValue(n, out var t)) return t;
                Debug.LogWarning($"[ReachyMiniRig] '{n}' not found under {name}. Is this the root of the imported reachy_mini.glb?", this);
                return null;
            }
            _bodyYawT = Get(ReachyMiniModel.BodyYaw.Body);
            _head = Get(ReachyMiniModel.HeadBody);
            _headForward = Get(ReachyMiniModel.HeadForward);
            _neutralOrigin = Get(ReachyMiniModel.NeutralHeadOrigin);
            for (int k = 0; k < 6; k++)
            {
                var m = ReachyMiniModel.Motors[k];
                _horns[k] = Get(m.Hinge.Body); _rods[k] = Get(m.RodBody);
                _tips[k] = Get(m.RodTip); _anchors[k] = Get(m.RodAnchor);
            }
            for (int i = 0; i < 2; i++) _antennas[i] = Get(ReachyMiniModel.Antennas[i].Body);
            _bound = _head != null && _bodyYawT != null && _neutralOrigin != null;
        }

        // ------------------------------------------------------------------ public API (SDK semantics)

        /// <summary>Like ReachyMini.set_target(head, antennas, body_yaw). Returns false (and keeps the
        /// previous head pose) when the pose is out of reach. Antennas and body yaw are always applied.</summary>
        public bool SetTarget(Matrix4x4 headPose, float bodyYawRad, float rightAntennaRad, float leftAntennaRad)
        {
            SetAntennas(rightAntennaRad, leftAntennaRad);
            return SetHead(FromMatrix(headPose), bodyYawRad);
        }

        public bool SetHeadPose(Matrix4x4 headPose) => SetHead(FromMatrix(headPose), _requestedBodyYaw);

        public bool SetBodyYaw(float rad) => SetHead(_pose, rad);

        public void SetAntennas(float rightRad, float leftRad)
        {
            _right = Mathf.Clamp(rightRad, -Mathf.PI, Mathf.PI);
            _left = Mathf.Clamp(leftRad, -Mathf.PI, Mathf.PI);
            for (int i = 0; i < 2; i++)
            {
                var a = ReachyMiniModel.Antennas[i];
                float sdk = i == 0 ? _right : _left;
                if (_antennas[i] != null)   // MJCF joint = -SDK angle
                    _antennas[i].localRotation = a.Rest * Quaternion.AngleAxis(-sdk * Mathf.Rad2Deg, Vector3.back);
            }
            SyncInspectorAntennas();
        }

        /// <summary>Check a pose without moving the robot.</summary>
        public bool IsPoseReachable(Matrix4x4 headPose, float bodyYawRad) =>
            Solve(FromMatrix(headPose), bodyYawRad, _scratch, out _);

        /// <summary>Smooth move like ReachyMini.goto_target (minimum-jerk, linear position + slerp rotation).</summary>
        public Coroutine GotoTarget(Matrix4x4 headPose, float bodyYawRad, float rightRad, float leftRad, float duration = 1f)
        {
            if (_goto != null) StopCoroutine(_goto);
            _goto = StartCoroutine(GotoRoutine(headPose, bodyYawRad, rightRad, leftRad, duration));
            return _goto;
        }

        public void ApplyKeyframe(string key)
        {
            if (!ReachyMiniModel.Keyframes.TryGetValue(key, out var k)) { Debug.LogWarning($"No keyframe {key}"); return; }
            SetTarget(ToMatrix(k.HeadPose), k.BodyYaw, k.RightAntenna, k.LeftAntenna);
        }

        public void GotoKeyframe(string key, float duration = 1.5f)
        {
            if (!ReachyMiniModel.Keyframes.TryGetValue(key, out var k)) { Debug.LogWarning($"No keyframe {key}"); return; }
            GotoTarget(ToMatrix(k.HeadPose), k.BodyYaw, k.RightAntenna, k.LeftAntenna, duration);
        }

        [ContextMenu("Pose/NEUTRAL")] void CtxNeutral() => ApplyKeyframe("NEUTRAL");
        [ContextMenu("Pose/SLEEP")] void CtxSleep() => ApplyKeyframe("SLEEP");

        // ------------------------------------------------------------------ frames
        /// <summary>Unity world point -> SDK frame (meters, x fwd, y left, z up), e.g. for look_at_world.</summary>
        public Vector3 WorldToSdk(Vector3 world)
        {
            var l = _neutralOrigin.InverseTransformPoint(world);   // robot meters, even if you scale the robot
            return new Vector3(-l.x, l.y, l.z);
        }

        public Vector3 SdkToWorld(Vector3 sdk) => _neutralOrigin.TransformPoint(new Vector3(-sdk.x, sdk.y, sdk.z));

        public Vector3 SdkDirToWorld(Vector3 dir) => _neutralOrigin.TransformDirection(new Vector3(-dir.x, dir.y, dir.z));

        public Vector3 WorldDirToSdk(Vector3 dir)
        {
            var l = _neutralOrigin.InverseTransformDirection(dir);
            return new Vector3(-l.x, l.y, l.z);
        }

        /// <summary>Unity world rotation of a frame -> SDK rotation matrix (columns = frame axes x fwd, y left, z up).</summary>
        public Matrix4x4 WorldPoseToSdk(Vector3 worldPos, Vector3 worldForward, Vector3 worldUp)
        {
            var f = WorldDirToSdk(worldForward).normalized;
            var u = WorldDirToSdk(worldUp);
            u = (u - Vector3.Dot(u, f) * f).normalized;
            var y = Vector3.Cross(u, f);   // left = up x forward (right-handed SDK frame)
            var p = WorldToSdk(worldPos);
            var m = Matrix4x4.identity;
            m.SetColumn(0, new Vector4(f.x, f.y, f.z, 0)); m.SetColumn(1, new Vector4(y.x, y.y, y.z, 0));
            m.SetColumn(2, new Vector4(u.x, u.y, u.z, 0)); m.SetColumn(3, new Vector4(p.x, p.y, p.z, 1));
            return m;
        }

        /// <summary>The SDK's create_head_pose(x, y, z, roll, pitch, yaw) in meters / radians.</summary>
        public static Matrix4x4 CreateHeadPose(float x, float y, float z, float roll, float pitch, float yaw)
        {
            double cr = Math.Cos(roll), sr = Math.Sin(roll), cp = Math.Cos(pitch), sp = Math.Sin(pitch), cy = Math.Cos(yaw), sy = Math.Sin(yaw);
            var m = Matrix4x4.identity;   // R = Rz(yaw) Ry(pitch) Rx(roll)
            m[0, 0] = (float)(cy * cp); m[0, 1] = (float)(cy * sp * sr - sy * cr); m[0, 2] = (float)(cy * sp * cr + sy * sr);
            m[1, 0] = (float)(sy * cp); m[1, 1] = (float)(sy * sp * sr + cy * cr); m[1, 2] = (float)(sy * sp * cr - cy * sr);
            m[2, 0] = (float)(-sp); m[2, 1] = (float)(cp * sr); m[2, 2] = (float)(cp * cr);
            m[0, 3] = x; m[1, 3] = y; m[2, 3] = z;
            return m;
        }

        /// <summary>Inverse of CreateHeadPose: (x, y, z, roll, pitch, yaw).</summary>
        public static void ToXYZRPY(Matrix4x4 m, out Vector3 pos, out Vector3 rpy)
        {
            pos = new Vector3(m[0, 3], m[1, 3], m[2, 3]);
            float pitch = Mathf.Asin(Mathf.Clamp(-m[2, 0], -1f, 1f));
            rpy = new Vector3(Mathf.Atan2(m[2, 1], m[2, 2]), pitch, Mathf.Atan2(m[1, 0], m[0, 0]));
        }

        /// <summary>Linear position + slerp rotation, like the SDK's linear_pose_interpolation.</summary>
        public static Matrix4x4 InterpolatePose(Matrix4x4 a, Matrix4x4 b, float t)
        {
            var qa = RotationOf(a); var qb = RotationOf(b);
            var q = Quaternion.Slerp(qa, qb, t);
            var m = Matrix4x4.Rotate(q);
            var p = Vector3.Lerp((Vector3)a.GetColumn(3), (Vector3)b.GetColumn(3), t);
            m.SetColumn(3, new Vector4(p.x, p.y, p.z, 1));
            return m;
        }

        /// <summary>Rotation part as a quaternion, as plain math (no handedness conversion).</summary>
        public static Quaternion RotationOf(Matrix4x4 m)
        {
            var d = FromMatrix(m); var q = MatToQuat(d);   // (w, x, y, z)
            return new Quaternion((float)q[1], (float)q[2], (float)q[3], (float)q[0]);
        }

        // ------------------------------------------------------------------ solve + pose the model
        float _requestedBodyYaw;

        bool SetHead(double[] pose, float bodyYawRad)
        {
            _requestedBodyYaw = bodyYawRad;
            if (!Solve(pose, bodyYawRad, _scratch, out float yawUsed))
            {
                reachable = false;
                return false;
            }
            reachable = true;
            Array.Copy(pose, _pose, 16);
            Array.Copy(_scratch, _motors, 6);
            _bodyYaw = yawUsed;
            PoseModel();
            SyncInspectorPose();
            return true;
        }

        /// <summary>Analytical IK (port of reachy_mini_rust_kinematics inverse_kinematics[_safe]).</summary>
        bool Solve(double[] pose, float requestedYaw, double[] outMotors, out float bodyYaw)
        {
            double yaw = requestedYaw;
            if (automaticBodyYaw)
            {
                double t = -yaw;
                double current = Math.Atan2(pose[1], pose[0]);           // T[0,1], T[0,0]
                double rel = Clamp(t - current, -65 * Mathf.Deg2Rad, 65 * Mathf.Deg2Rad);
                t = Clamp(current + rel, -160 * Mathf.Deg2Rad, 160 * Mathf.Deg2Rad);
                yaw = -t;
            }
            var by = ReachyMiniModel.BodyYaw;
            yaw = Clamp(yaw, by.MinRad, by.MaxRad);
            bodyYaw = (float)yaw;

            // platform pose in the turning body's frame: Rz(-yaw) * (pose + z offset)
            double c = Math.Cos(-yaw), s = Math.Sin(-yaw);
            var P = _tp;
            for (int col = 0; col < 4; col++)
            {
                double x = pose[col], y = pose[4 + col], z = pose[8 + col] + (col == 3 ? ReachyMiniModel.HeadZOffset : 0);
                P[col] = c * x - s * y; P[4 + col] = s * x + c * y; P[8 + col] = z;
            }
            double rs = ReachyMiniModel.MotorArmLength, rp = ReachyMiniModel.RodLength;
            for (int k = 0; k < 6; k++)
            {
                var m = ReachyMiniModel.Motors[k];
                var b = m.Branch; var T = m.TMotorWorld;
                // branch point in body frame
                double wx = P[0] * b[0] + P[1] * b[1] + P[2] * b[2] + P[3];
                double wy = P[4] * b[0] + P[5] * b[1] + P[6] * b[2] + P[7];
                double wz = P[8] * b[0] + P[9] * b[1] + P[10] * b[2] + P[11];
                // ... in the motor frame
                double px = T[0] * wx + T[1] * wy + T[2] * wz + T[3];
                double py = T[4] * wx + T[5] * wy + T[6] * wz + T[7];
                double pz = T[8] * wx + T[9] * wy + T[10] * wz + T[11];
                double px2 = px * px, py2 = py * py, pz2 = pz * pz, rp2 = rp * rp, rs2 = rs * rs;
                double X = px2 + 2 * px * rs + py2 + pz2 - rp2 + rs2;
                double disc = -(px2 * px2) - 2 * px2 * py2 - 2 * px2 * pz2 + 2 * px2 * rp2 + 2 * px2 * rs2 - py2 * py2
                              - 2 * py2 * pz2 + 2 * py2 * rp2 + 2 * py2 * rs2 - pz2 * pz2 + 2 * pz2 * rp2 - 2 * pz2 * rs2
                              - rp2 * rp2 + 2 * rp2 * rs2 - rs2 * rs2;
                if (disc < 0) return false;                               // rod can't reach
                double Y = 2 * py * rs + m.Solution * Math.Sqrt(disc);
                double a = 2 * Math.Atan2(Y, X);
                a -= 2 * Math.PI * Math.Floor((a + Math.PI) / (2 * Math.PI));
                if (enforceSimJointRanges && (a < m.Hinge.MinRad || a > m.Hinge.MaxRad)) return false;
                outMotors[k] = a;
            }
            return true;
        }
        readonly double[] _tp = new double[12];

        void PoseModel()
        {
            if (!_bound) return;
            var by = ReachyMiniModel.BodyYaw;
            _bodyYawT.localRotation = by.Rest * Quaternion.AngleAxis(_bodyYaw * Mathf.Rad2Deg, Vector3.back);
            for (int k = 0; k < 6; k++)
            {
                var h = ReachyMiniModel.Motors[k].Hinge;
                if (_horns[k] != null)
                    _horns[k].localRotation = h.Rest * Quaternion.AngleAxis((float)(_motors[k] * Mathf.Rad2Deg), Vector3.back);
            }
            // head: platform pose * T_platform_head, written as MuJoCo-frame local transform under body_foot
            var P = new double[12];
            for (int i = 0; i < 12; i++) P[i] = _pose[i];
            P[11] += ReachyMiniModel.HeadZOffset;
            var Hd = Mul34(P, ReachyMiniModel.TPlatformHead);
            if (_head != null)
            {
                _head.localPosition = new Vector3((float)-Hd[3], (float)Hd[7], (float)Hd[11]);
                var q = MatToQuat(Hd);  // (w, x, y, z)
                _head.localRotation = new Quaternion((float)q[1], (float)-q[2], (float)-q[3], (float)q[0]);
            }
            // rods: from rest, swing each one so its tip lands on its anchor (ball joints at both ends)
            for (int k = 0; k < 6; k++)
            {
                if (_rods[k] == null || _tips[k] == null || _anchors[k] == null) continue;
                _rods[k].localRotation = ReachyMiniModel.Motors[k].RodRest;
                var o = _rods[k].position;
                _rods[k].rotation = Quaternion.FromToRotation(_tips[k].position - o, _anchors[k].position - o) * _rods[k].rotation;
            }
            for (int k = 0; k < 6; k++) motorAnglesDeg[k] = (float)(_motors[k] * Mathf.Rad2Deg);
        }

        // ------------------------------------------------------------------ inspector sync
        bool _syncing;
        void ApplyInspector()
        {
            if (_syncing) return;
            var hp = CreateHeadPose(headPositionMm.x / 1000f, headPositionMm.y / 1000f, headPositionMm.z / 1000f,
                headRollPitchYawDeg.x * Mathf.Deg2Rad, headRollPitchYawDeg.y * Mathf.Deg2Rad, headRollPitchYawDeg.z * Mathf.Deg2Rad);
            _syncing = true;
            SetAntennas(rightAntennaDeg * Mathf.Deg2Rad, leftAntennaDeg * Mathf.Deg2Rad);
            SetHead(FromMatrix(hp), bodyYawDeg * Mathf.Deg2Rad);
            _syncing = false;
        }

        void SyncInspectorPose()
        {
            if (_syncing) return;
            ToXYZRPY(ToMatrix(_pose), out var p, out var rpy);
            headPositionMm = p * 1000f; headRollPitchYawDeg = rpy * Mathf.Rad2Deg; bodyYawDeg = _requestedBodyYaw * Mathf.Rad2Deg;
        }

        void SyncInspectorAntennas()
        {
            if (_syncing) return;
            rightAntennaDeg = _right * Mathf.Rad2Deg; leftAntennaDeg = _left * Mathf.Rad2Deg;
        }

        // ------------------------------------------------------------------ goto
        Coroutine _goto;
        IEnumerator GotoRoutine(Matrix4x4 target, float yaw, float right, float left, float duration)
        {
            var start = HeadPose; float y0 = _requestedBodyYaw, r0 = _right, l0 = _left;
            for (float t = 0; t < duration; t += Time.deltaTime)
            {
                float u = t / duration; u = u * u * u * (10 + u * (-15 + 6 * u));   // minimum jerk
                SetTarget(InterpolatePose(start, target, u), Mathf.Lerp(y0, yaw, u), Mathf.Lerp(r0, right, u), Mathf.Lerp(l0, left, u));
                yield return null;
            }
            SetTarget(target, yaw, right, left);
            _goto = null;
        }

        // ------------------------------------------------------------------ math helpers (row-major doubles)
        static double[] Identity() => new double[] { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
        static double Clamp(double v, double lo, double hi) => v < lo ? lo : (v > hi ? hi : v);

        public static double[] FromMatrix(Matrix4x4 m)
        {
            var d = new double[16];
            for (int r = 0; r < 4; r++) for (int c = 0; c < 4; c++) d[r * 4 + c] = m[r, c];
            return d;
        }

        public static Matrix4x4 ToMatrix(double[] d)
        {
            var m = new Matrix4x4();
            for (int r = 0; r < 4; r++) for (int c = 0; c < 4; c++) m[r, c] = (float)d[r * 4 + c];
            if (d.Length == 12) m[3, 3] = 1;
            return m;
        }

        static double[] Mul34(double[] A, double[] B)   // 3x4 * 3x4 (implicit last row 0 0 0 1)
        {
            var C = new double[12];
            for (int r = 0; r < 3; r++)
            {
                for (int c = 0; c < 4; c++)
                    C[r * 4 + c] = A[r * 4] * B[c] + A[r * 4 + 1] * B[4 + c] + A[r * 4 + 2] * B[8 + c];
                C[r * 4 + 3] += A[r * 4 + 3];
            }
            return C;
        }

        /// <summary>Rotation part of a row-major 3x4/4x4 -> quaternion (w, x, y, z).</summary>
        static double[] MatToQuat(double[] m)
        {
            const int w = 4;   // row stride is 4 for both 3x4 and 4x4 row-major
            double m00 = m[0], m01 = m[1], m02 = m[2], m10 = m[w], m11 = m[w + 1], m12 = m[w + 2], m20 = m[2 * w], m21 = m[2 * w + 1], m22 = m[2 * w + 2];
            double t = m00 + m11 + m22; double[] q;
            if (t > 0) { double s = Math.Sqrt(t + 1) * 2; q = new[] { 0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s }; }
            else if (m00 > m11 && m00 > m22) { double s = Math.Sqrt(1 + m00 - m11 - m22) * 2; q = new[] { (m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s }; }
            else if (m11 > m22) { double s = Math.Sqrt(1 + m11 - m00 - m22) * 2; q = new[] { (m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s }; }
            else { double s = Math.Sqrt(1 + m22 - m00 - m11) * 2; q = new[] { (m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s }; }
            double n = Math.Sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
            for (int i = 0; i < 4; i++) q[i] /= n;
            return q;
        }

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
