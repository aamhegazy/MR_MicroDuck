using UnityEngine;

namespace Microduck
{
    /// <summary>
    /// First MR interaction: the duck turns its head to look at you (the Quest headset camera)
    /// when you come close, and glances around on its own when you're far away.
    /// Pure kinematics on the real joint chain (neck_pitch, head_pitch, head_yaw) with the real joint
    /// limits, so later the same target can be sent to the policy as its head-pose command instead.
    /// </summary>
    [RequireComponent(typeof(MicroduckRig))]
    public class MicroduckLookAt : MonoBehaviour
    {
        [Tooltip("What to look at. Empty = Camera.main (the headset on Quest).")]
        public Transform target;

        [Tooltip("Start paying attention within this distance (m).")]
        public float attentionRadius = 1.5f;

        [Tooltip("Higher = snappier head.")]
        public float responsiveness = 6f;

        [Header("Comfort limits (deg) on top of the robot's own joint limits")]
        public float maxYaw = 80f;
        public float maxHeadPitch = 50f;
        public float maxNeckPitch = 25f;

        [Header("Idle behaviour")]
        public bool idleGlances = true;
        public Vector2 glanceEverySeconds = new Vector2(1.5f, 4f);

        MicroduckRig _rig;
        int _yaw, _hpitch, _npitch;
        float[] _rest = new float[3];          // angles at enable (usually STAND)
        float[] _cur = new float[3];
        Vector3 _idlePoint; float _nextGlance;

        int[] _chain = new int[3];
        float[] _lim = new float[3], _restOf = new float[3], _solved = new float[3];

        // Start (not OnEnable) so MicroduckRig has already bound and posed the duck.
        void Start()
        {
            _rig = GetComponent<MicroduckRig>();
            _yaw = MicroduckRig.IndexOf("head_yaw");
            _hpitch = MicroduckRig.IndexOf("head_pitch");
            _npitch = MicroduckRig.IndexOf("neck_pitch");
            _rest[0] = _rig.GetJoint(_npitch); _rest[1] = _rig.GetJoint(_hpitch); _rest[2] = _rig.GetJoint(_yaw);
            System.Array.Copy(_rest, _cur, 3);
        }

        void LateUpdate()
        {
            var head = _rig.HeadForward;
            if (head == null) return;
            var tgt = target != null ? target : (Camera.main != null ? Camera.main.transform : null);

            Vector3 goal; bool attending = false;
            if (tgt != null && Vector3.Distance(tgt.position, head.position) < attentionRadius)
            { goal = tgt.position; attending = true; }
            else if (idleGlances) goal = IdlePoint(head);
            else goal = head.position + transform.forward;

            // 1) solve: start from rest, run a few CCD sweeps from the head joint outward to the neck
            var chain = _chain; var lim = _lim; var restOf = _restOf;
            chain[0] = _yaw; chain[1] = _hpitch; chain[2] = _npitch;
            lim[0] = maxYaw; lim[1] = maxHeadPitch; lim[2] = maxNeckPitch;
            restOf[0] = _rest[2]; restOf[1] = _rest[1]; restOf[2] = _rest[0];
            for (int k = 0; k < 3; k++) _rig.SetJoint(chain[k], restOf[k]);
            for (int it = 0; it < 4; it++)
            {
                for (int k = 0; k < 3; k++)
                {
                    int j = chain[k];
                    Transform body = JointBody(j);
                    if (body == null) continue;
                    Vector3 axis = body.rotation * Vector3.back;        // joint axis in world
                    Vector3 from = Vector3.ProjectOnPlane(head.forward, axis);
                    Vector3 to = Vector3.ProjectOnPlane(goal - head.position, axis);
                    if (from.sqrMagnitude < 1e-8f || to.sqrMagnitude < 1e-8f) continue;
                    float delta = Vector3.SignedAngle(from, to, axis) * Mathf.Deg2Rad;
                    float a = _rig.GetJoint(j) + delta * (k == 2 ? 0.35f : 1f);   // neck helps a little
                    float l = lim[k] * Mathf.Deg2Rad;
                    a = Mathf.Clamp(a, restOf[k] - l, restOf[k] + l);
                    _rig.SetJoint(j, a);
                }
            }
            var solved = _solved;
            solved[0] = _rig.GetJoint(_npitch); solved[1] = _rig.GetJoint(_hpitch); solved[2] = _rig.GetJoint(_yaw);

            // 2) smooth toward the solution so it feels alive rather than robotic
            float s = 1f - Mathf.Exp(-(attending ? responsiveness : responsiveness * 0.5f) * Time.deltaTime);
            for (int k = 0; k < 3; k++) _cur[k] = Mathf.Lerp(_cur[k], solved[k], s);
            _rig.SetJoint(_npitch, _cur[0]); _rig.SetJoint(_hpitch, _cur[1]); _rig.SetJoint(_yaw, _cur[2]);
        }

        readonly System.Collections.Generic.Dictionary<int, Transform> _bodyCache =
            new System.Collections.Generic.Dictionary<int, Transform>();

        Transform JointBody(int j)
        {
            // the joint rotates its own body (the GameObject named after the MuJoCo body)
            if (!_bodyCache.TryGetValue(j, out var t) || t == null)
                _bodyCache[j] = t = FindDeep(transform, MicroduckModel.Joints[j].Body);
            return t;
        }

        Vector3 IdlePoint(Transform head)
        {
            if (Time.time >= _nextGlance)
            {
                _nextGlance = Time.time + Random.Range(glanceEverySeconds.x, glanceEverySeconds.y);
                var dir = Quaternion.Euler(Random.Range(-15f, 20f), Random.Range(-60f, 60f), 0) * transform.forward;
                _idlePoint = head.position + dir * 1.0f;
            }
            return _idlePoint;
        }

        static Transform FindDeep(Transform t, string n)
        {
            if (t.name == n) return t;
            foreach (Transform c in t) { var r = FindDeep(c, n); if (r != null) return r; }
            return null;
        }
    }
}
