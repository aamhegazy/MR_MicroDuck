using UnityEngine;

namespace ReachyMini
{
    /// <summary>
    /// First MR interaction: Reachy Mini looks at you (the Quest headset) when you come close,
    /// wiggles its antennas when it notices you, and glances around on its own when you're far.
    /// It builds the head pose exactly like the SDK's look_at_world (head re-centered, +X aimed at the
    /// point), so the same target can later be sent to the real robot unchanged.
    /// When the requested pose is out of reach it goes as far as the robot can, instead of freezing.
    /// </summary>
    [RequireComponent(typeof(ReachyMiniRig))]
    public class ReachyMiniLookAt : MonoBehaviour
    {
        [Tooltip("What to look at. Empty = Camera.main (the headset on Quest).")]
        public Transform target;

        [Tooltip("Start paying attention within this distance (m).")]
        public float attentionRadius = 1.5f;

        [Tooltip("Higher = snappier head.")]
        public float responsiveness = 5f;

        [Tooltip("Share of the head's yaw that the body turns along (0 = head only).")]
        [Range(0f, 1f)] public float bodyFollow = 0.4f;

        [Header("Antennas")]
        public bool wiggleWhenNoticed = true;
        public float restAntennaDeg = 10f;

        [Header("Idle behaviour")]
        public bool idleGlances = true;
        public Vector2 glanceEverySeconds = new Vector2(1.5f, 4f);

        ReachyMiniRig _rig;
        Matrix4x4 _cur = Matrix4x4.identity;
        float _curYaw;
        bool _attending;
        float _noticedAt = -10f, _nextGlance;
        Vector3 _idleSdk = new Vector3(1, 0, 0);

        void Start()
        {
            _rig = GetComponent<ReachyMiniRig>();
            _cur = _rig.HeadPose; _curYaw = _rig.BodyYaw;
        }

        void OnEnable()
        {
            if (_rig != null) { _cur = _rig.HeadPose; _curYaw = _rig.BodyYaw; }
        }

        void LateUpdate()
        {
            if (_rig == null || _rig.NeutralHeadOrigin == null) return;
            var tgt = target != null ? target : (Camera.main != null ? Camera.main.transform : null);

            // 1) where to look, in the SDK frame (meters from the neutral head origin; x fwd, y left, z up)
            Vector3 p; bool attending = false;
            if (tgt != null && Vector3.Distance(tgt.position, _rig.NeutralHeadOrigin.position) < attentionRadius)
            { p = _rig.WorldToSdk(tgt.position); attending = true; }
            else if (idleGlances) p = IdlePoint();
            else p = new Vector3(1, 0, 0);
            if (attending && !_attending) _noticedAt = Time.time;
            _attending = attending;

            // 2) the SDK's look_at_world pose, then ease toward it
            var desired = LookAtWorldPose(p);
            float headYaw = Mathf.Atan2(desired[1, 0], desired[0, 0]);
            float s = 1f - Mathf.Exp(-(attending ? responsiveness : responsiveness * 0.5f) * Time.deltaTime);
            var next = ReachyMiniRig.InterpolatePose(_cur, desired, s);
            float nextYaw = Mathf.Lerp(_curYaw, headYaw * bodyFollow, s);

            // 3) antennas: a happy wiggle when it notices you, otherwise a gentle sway
            float t = Time.time - _noticedAt, ant = restAntennaDeg * Mathf.Deg2Rad;
            float wig = wiggleWhenNoticed && t < 1.2f ? 0.5f * Mathf.Sin(t * 2 * Mathf.PI * 3f) * (1f - t / 1.2f) : 0.05f * Mathf.Sin(Time.time * 0.7f);
            float right = -ant - wig, left = ant + wig;   // mirror-symmetric flap

            // 4) apply; if out of reach, go as far toward it as the robot allows
            if (!_rig.SetTarget(next, nextYaw, right, left))
            {
                float lo = 0f, hi = 1f;
                for (int i = 0; i < 8; i++)
                {
                    float mid = 0.5f * (lo + hi);
                    if (_rig.IsPoseReachable(ReachyMiniRig.InterpolatePose(_cur, next, mid), nextYaw)) lo = mid; else hi = mid;
                }
                next = ReachyMiniRig.InterpolatePose(_cur, next, lo);
                _rig.SetTarget(next, nextYaw, right, left);
            }
            _cur = _rig.HeadPose; _curYaw = nextYaw;
        }

        /// <summary>Port of reachy_mini.vision.look_at.look_at_world_pose.</summary>
        public static Matrix4x4 LookAtWorldPose(Vector3 p)
        {
            if (p.sqrMagnitude < 1e-12f) return Matrix4x4.identity;
            var v = p.normalized; var x = new Vector3(1, 0, 0);
            var axis = Vector3.Cross(x, v); float n = axis.magnitude;
            Quaternion q;   // plain math in the SDK frame (Unity's AngleAxis is the standard right-hand formula)
            if (n < 1e-8f) q = Vector3.Dot(x, v) > 0 ? Quaternion.identity : Quaternion.AngleAxis(180f, new Vector3(0, 0, 1));
            else q = Quaternion.AngleAxis(Mathf.Acos(Mathf.Clamp(Vector3.Dot(x, v), -1f, 1f)) * Mathf.Rad2Deg, axis / n);
            return Matrix4x4.Rotate(q);
        }

        Vector3 IdlePoint()
        {
            if (Time.time >= _nextGlance)
            {
                _nextGlance = Time.time + Random.Range(glanceEverySeconds.x, glanceEverySeconds.y);
                float yaw = Random.Range(-50f, 50f) * Mathf.Deg2Rad, pitch = Random.Range(-15f, 10f) * Mathf.Deg2Rad;
                _idleSdk = new Vector3(Mathf.Cos(pitch) * Mathf.Cos(yaw), Mathf.Cos(pitch) * Mathf.Sin(yaw), -Mathf.Sin(pitch));
            }
            return _idleSdk;
        }
    }
}
