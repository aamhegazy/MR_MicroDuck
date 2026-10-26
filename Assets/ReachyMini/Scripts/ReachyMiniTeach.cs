using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace ReachyMini
{
    /// <summary>One sample, same fields as the SDK's recorded moves ("set_target_data").</summary>
    [Serializable]
    public class TargetFrame
    {
        public float[] head;       // 4x4 row-major SDK head pose (flat 16)
        public float[] antennas;   // [right, left] rad
        public float body_yaw;     // rad
    }

    /// <summary>Same layout as the Hugging Face move datasets the SDK plays (RecordedMove).</summary>
    [Serializable]
    public class RecordedMove
    {
        public string description;
        public List<float> time = new List<float>();
        public List<TargetFrame> set_target_data = new List<TargetFrame>();
        public float Duration => time.Count > 0 ? time[time.Count - 1] : 0f;
    }

    /// <summary>
    /// Teaching by demonstration in MR: grab Reachy Mini's head with your hand and move it; the twin
    /// follows within what the real robot can reach. Record what it does (whatever is driving it: your
    /// hand, the look-at, or the real robot mirrored through ReachyMiniDaemonLink), play it back, and save
    /// it as JSON in the SDK's recorded-move format so tools/play_move.py can replay it on the robot.
    ///
    /// Grab wiring (Meta Interaction SDK): put the "Grab Interaction" building block on the TeachHandle
    /// object this component creates, add a PointableUnityEventWrapper to it and connect
    /// When Select -> BeginGrab and When Unselect -> EndGrab. In the editor, tick "grabbed" and drag the
    /// handle in the Scene view.
    /// </summary>
    public class ReachyMiniTeach : MonoBehaviour
    {
        public ReachyMiniRig rig;

        [Tooltip("What your hand grabs. Created around the head at start if empty. Keep it outside the robot's hierarchy.")]
        public Transform handle;
        public float handleRadius = 0.07f;

        [Tooltip("True while the handle is held (set by BeginGrab/EndGrab, or tick it in the editor).")]
        public bool grabbed;

        [Header("Recording")]
        public float sampleRate = 50f;
        public string moveName = "my_move";
        public string description = "Taught in mixed reality";
        public bool loopPlayback;

        [Header("Status (read only)")]
        [SerializeField] string status = "idle";
        [SerializeField] float recordedSeconds;

        public RecordedMove LastMove { get; private set; }
        public bool IsRecording => _recording;
        public bool IsPlaying => _play != null;
        public string SaveFolder => Path.Combine(Application.persistentDataPath, "reachy_moves");

        ReachyMiniLookAt _lookAt;
        bool _lookAtWasOn, _wasGrabbed, _recording;
        Vector3 _homePos; Quaternion _homeRot;     // handle pose in head-local space
        Matrix4x4 _handle0, _head0;                // at grab start (SDK frame)
        float _recStart, _nextSample;
        Coroutine _play;

        void Start()
        {
            if (rig == null) rig = GetComponent<ReachyMiniRig>();
            if (rig == null || rig.Head == null) { Debug.LogError("[ReachyMiniTeach] needs a ReachyMiniRig", this); enabled = false; return; }
            _lookAt = rig.GetComponent<ReachyMiniLookAt>();

            // handle centered on the head's visible bounds
            var b = new Bounds(rig.Head.position, Vector3.zero);
            foreach (var r in rig.Head.GetComponentsInChildren<Renderer>())
                if (r.transform.parent == rig.Head) b.Encapsulate(r.bounds);   // head shell only, not the antennas
            if (handle == null)
            {
                var go = new GameObject("TeachHandle");
                go.transform.SetPositionAndRotation(b.center, rig.Head.rotation);
                var sc = go.AddComponent<SphereCollider>(); sc.radius = handleRadius;
                var rb = go.AddComponent<Rigidbody>(); rb.isKinematic = true; rb.useGravity = false;
                handle = go.transform;
            }
            _homePos = rig.Head.InverseTransformPoint(handle.position);
            _homeRot = Quaternion.Inverse(rig.Head.rotation) * handle.rotation;
        }

        // ------------------------------------------------------------------ grab
        public void BeginGrab() => grabbed = true;
        public void EndGrab() => grabbed = false;

        void LateUpdate()
        {
            if (handle == null) return;
            if (grabbed && !_wasGrabbed) OnGrabStart();
            if (!grabbed && _wasGrabbed) OnGrabEnd();
            _wasGrabbed = grabbed;

            if (grabbed && _play == null)
            {
                // the head moves rigidly with the hand: target = handle(t) * handle(0)^-1 * head(0)
                var h = rig.WorldPoseToSdk(handle.position, handle.forward, handle.up);
                var target = h * _handle0.inverse * _head0;
                var cur = rig.HeadPose;
                if (!rig.SetTarget(target, rig.BodyYaw, rig.RightAntenna, rig.LeftAntenna))
                {
                    float lo = 0f, hi = 1f;   // as close as the robot can get
                    for (int i = 0; i < 8; i++)
                    {
                        float mid = 0.5f * (lo + hi);
                        if (rig.IsPoseReachable(ReachyMiniRig.InterpolatePose(cur, target, mid), rig.BodyYaw)) lo = mid; else hi = mid;
                    }
                    rig.SetTarget(ReachyMiniRig.InterpolatePose(cur, target, lo), rig.BodyYaw, rig.RightAntenna, rig.LeftAntenna);
                }
            }
            else if (!grabbed)
            {
                handle.SetPositionAndRotation(rig.Head.TransformPoint(_homePos), rig.Head.rotation * _homeRot);
            }

            if (_recording && Time.time >= _nextSample)
            {
                _nextSample = Mathf.Max(_nextSample + 1f / sampleRate, Time.time);
                LastMove.time.Add(Time.time - _recStart);
                LastMove.set_target_data.Add(Snapshot());
                recordedSeconds = LastMove.Duration;
            }
        }

        void OnGrabStart()
        {
            PauseLookAt();
            _handle0 = rig.WorldPoseToSdk(handle.position, handle.forward, handle.up);
            _head0 = rig.HeadPose;
            status = _recording ? "recording (grabbed)" : "grabbed";
        }

        void OnGrabEnd()
        {
            if (_play == null) ResumeLookAt();
            status = _recording ? "recording" : "idle";
        }

        void PauseLookAt()
        {
            if (_lookAt != null && _lookAt.enabled) { _lookAtWasOn = true; _lookAt.enabled = false; }
        }

        void ResumeLookAt()
        {
            if (_lookAt != null && _lookAtWasOn) { _lookAt.enabled = true; _lookAtWasOn = false; }
        }

        // ------------------------------------------------------------------ record
        [ContextMenu("Teach/Start recording")]
        public void StartRecording()
        {
            LastMove = new RecordedMove { description = description };
            _recStart = Time.time; _nextSample = Time.time; _recording = true;
            status = grabbed ? "recording (grabbed)" : "recording";
        }

        [ContextMenu("Teach/Stop recording")]
        public RecordedMove StopRecording()
        {
            _recording = false;
            status = $"recorded {LastMove?.set_target_data.Count ?? 0} frames";
            return LastMove;
        }

        TargetFrame Snapshot()
        {
            var m = rig.HeadPose; var head = new float[16];
            for (int r = 0; r < 4; r++) for (int c = 0; c < 4; c++) head[r * 4 + c] = m[r, c];
            return new TargetFrame { head = head, antennas = new[] { rig.RightAntenna, rig.LeftAntenna }, body_yaw = rig.BodyYaw };
        }

        // ------------------------------------------------------------------ play
        [ContextMenu("Teach/Play last move")]
        public void PlayLast() { if (LastMove != null) Play(LastMove); }

        public void Play(RecordedMove move)
        {
            if (move == null || move.time.Count < 2) return;
            if (_recording) StopRecording();
            if (_play != null) StopCoroutine(_play);
            _play = StartCoroutine(PlayRoutine(move));
        }

        [ContextMenu("Teach/Stop playback")]
        public void StopPlayback()
        {
            if (_play != null) StopCoroutine(_play);
            _play = null; status = "idle"; ResumeLookAt();
        }

        IEnumerator PlayRoutine(RecordedMove move)
        {
            PauseLookAt();
            // get to the first frame smoothly, like play_move(initial_goto_duration=...)
            var f0 = move.set_target_data[0];
            yield return rig.GotoTarget(ToMatrix(f0.head), f0.body_yaw, f0.antennas[0], f0.antennas[1], 0.8f);
            do
            {
                status = "playing";
                float t0 = Time.time, dur = move.Duration; int i = 0;
                while (Time.time - t0 < dur)
                {
                    float t = Time.time - t0;
                    while (i < move.time.Count - 2 && move.time[i + 1] <= t) i++;
                    float a = Mathf.InverseLerp(move.time[i], move.time[i + 1], t);
                    var A = move.set_target_data[i]; var B = move.set_target_data[i + 1];
                    rig.SetTarget(ReachyMiniRig.InterpolatePose(ToMatrix(A.head), ToMatrix(B.head), a),
                                  Mathf.Lerp(A.body_yaw, B.body_yaw, a),
                                  Mathf.Lerp(A.antennas[0], B.antennas[0], a), Mathf.Lerp(A.antennas[1], B.antennas[1], a));
                    yield return null;
                }
            } while (loopPlayback);
            _play = null; status = "idle"; ResumeLookAt();
        }

        static Matrix4x4 ToMatrix(float[] h)
        {
            var m = new Matrix4x4();
            for (int r = 0; r < 4; r++) for (int c = 0; c < 4; c++) m[r, c] = h[r * 4 + c];
            return m;
        }

        // ------------------------------------------------------------------ files
        [ContextMenu("Teach/Save last move")]
        public string SaveLast() => LastMove == null ? null : Save(LastMove, moveName);

        public string Save(RecordedMove move, string name)
        {
            Directory.CreateDirectory(SaveFolder);
            var path = Path.Combine(SaveFolder, name + ".json");
            File.WriteAllText(path, JsonUtility.ToJson(move, true));
            status = "saved " + path; Debug.Log("[ReachyMiniTeach] saved " + path);
            return path;
        }

        public RecordedMove Load(string name)
        {
            var path = Path.Combine(SaveFolder, name + ".json");
            if (!File.Exists(path)) return null;
            return LastMove = JsonUtility.FromJson<RecordedMove>(File.ReadAllText(path));
        }

        public string[] SavedMoves()
        {
            if (!Directory.Exists(SaveFolder)) return new string[0];
            var files = Directory.GetFiles(SaveFolder, "*.json");
            for (int i = 0; i < files.Length; i++) files[i] = Path.GetFileNameWithoutExtension(files[i]);
            return files;
        }
    }
}
