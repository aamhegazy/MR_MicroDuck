using System;
using System.Globalization;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

namespace ReachyMini
{
    /// <summary>
    /// Live link to a Reachy Mini daemon (the real robot, or `reachy-mini-daemon --sim` on your PC)
    /// over its WebSocket API:
    ///   Mirror: the twin follows the robot  (ws://HOST:8000/api/state/ws/full)
    ///   Drive:  the robot follows the twin  (ws://HOST:8000/api/move/ws/set_target)
    /// Poses travel as x, y, z, roll, pitch, yaw (m, rad, extrinsic xyz) like the daemon's XYZRPYPose.
    /// Drive only ever sends poses the twin's IK accepted, so the robot gets nothing it would refuse.
    /// </summary>
    public class ReachyMiniDaemonLink : MonoBehaviour
    {
        public enum Mode { MirrorRobot, DriveRobot }

        public ReachyMiniRig rig;
        [Tooltip("Robot IP (Quest often can't resolve reachy-mini.local), or your PC's IP for the simulator.")]
        public string host = "192.168.1.42";
        public int port = 8000;
        public Mode mode = Mode.MirrorRobot;
        [Range(1f, 60f)] public float rateHz = 30f;
        public bool connectOnStart;

        [Header("Status (read only)")]
        [SerializeField] string status = "disconnected";
        [SerializeField] int messages;

        [Serializable] class Pose6 { public float x, y, z, roll, pitch, yaw; }
        [Serializable] class StateMsg { public Pose6 head_pose; public float body_yaw; public float[] antennas_position; }

        ClientWebSocket _ws;
        CancellationTokenSource _cts;
        readonly object _lock = new object();
        StateMsg _latest; bool _fresh;
        float _nextSend;
        ReachyMiniLookAt _lookAt; bool _lookAtWasOn;
        volatile bool _sending;

        public bool IsConnected => _ws != null && _ws.State == WebSocketState.Open;

        void Start()
        {
            if (rig == null) rig = GetComponent<ReachyMiniRig>();
            _lookAt = rig != null ? rig.GetComponent<ReachyMiniLookAt>() : null;
            if (connectOnStart) Connect();
        }

        [ContextMenu("Connect")]
        public async void Connect()
        {
            Disconnect();
            var path = mode == Mode.MirrorRobot
                ? "/api/state/ws/full?frequency=" + rateHz.ToString(CultureInfo.InvariantCulture) +
                  "&with_head_pose=true&with_body_yaw=true&with_antenna_positions=true&use_pose_matrix=false"
                : "/api/move/ws/set_target";
            var uri = new Uri($"ws://{host}:{port}{path}");
            _cts = new CancellationTokenSource();
            _ws = new ClientWebSocket();
            status = "connecting " + uri;
            try
            {
                await _ws.ConnectAsync(uri, _cts.Token);
                status = (mode == Mode.MirrorRobot ? "mirroring " : "driving ") + host;
                if (mode == Mode.MirrorRobot && _lookAt != null && _lookAt.enabled) { _lookAtWasOn = true; _lookAt.enabled = false; }
                _ = ReceiveLoop(_ws, _cts.Token);
            }
            catch (Exception e) { status = "failed: " + e.Message; Debug.LogWarning("[ReachyMiniDaemonLink] " + e.Message, this); }
        }

        [ContextMenu("Disconnect")]
        public void Disconnect()
        {
            try { _cts?.Cancel(); } catch { }
            try { _ws?.Dispose(); } catch { }
            _ws = null; _cts = null;
            if (_lookAt != null && _lookAtWasOn) { _lookAt.enabled = true; _lookAtWasOn = false; }
            status = "disconnected";
        }

        void OnDisable() => Disconnect();

        async Task ReceiveLoop(ClientWebSocket ws, CancellationToken ct)
        {
            var buf = new byte[16384]; var sb = new StringBuilder();
            try
            {
                while (!ct.IsCancellationRequested && ws.State == WebSocketState.Open)
                {
                    var r = await ws.ReceiveAsync(new ArraySegment<byte>(buf), ct);
                    if (r.MessageType == WebSocketMessageType.Close) break;
                    sb.Append(Encoding.UTF8.GetString(buf, 0, r.Count));
                    if (!r.EndOfMessage) continue;
                    var text = sb.ToString(); sb.Clear();
                    if (mode == Mode.MirrorRobot)
                    {
                        var msg = JsonUtility.FromJson<StateMsg>(text);
                        lock (_lock) { _latest = msg; _fresh = true; }
                    }
                    else if (text.Contains("error")) Debug.LogWarning("[ReachyMiniDaemonLink] daemon: " + text);
                    Interlocked.Increment(ref messages);
                }
            }
            catch (OperationCanceledException) { }
            catch (Exception e) { Debug.LogWarning("[ReachyMiniDaemonLink] " + e.Message); }
        }

        void Update()
        {
            if (!IsConnected || rig == null) return;
            if (mode == Mode.MirrorRobot)
            {
                StateMsg s = null;
                lock (_lock) { if (_fresh) { s = _latest; _fresh = false; } }
                if (s == null || s.head_pose == null) return;
                var p = s.head_pose;
                var pose = ReachyMiniRig.CreateHeadPose(p.x, p.y, p.z, p.roll, p.pitch, p.yaw);
                float r = s.antennas_position != null && s.antennas_position.Length == 2 ? s.antennas_position[0] : rig.RightAntenna;
                float l = s.antennas_position != null && s.antennas_position.Length == 2 ? s.antennas_position[1] : rig.LeftAntenna;
                rig.SetTarget(pose, s.body_yaw, r, l);
            }
            else if (Time.time >= _nextSend && !_sending)
            {
                _nextSend = Time.time + 1f / rateHz;
                ReachyMiniRig.ToXYZRPY(rig.HeadPose, out var pos, out var rpy);
                var ci = CultureInfo.InvariantCulture;
                string F(float v) => v.ToString("R", ci);
                var json = "{\"target_head_pose\":{\"x\":" + F(pos.x) + ",\"y\":" + F(pos.y) + ",\"z\":" + F(pos.z) +
                           ",\"roll\":" + F(rpy.x) + ",\"pitch\":" + F(rpy.y) + ",\"yaw\":" + F(rpy.z) + "}," +
                           "\"target_antennas\":[" + F(rig.RightAntenna) + "," + F(rig.LeftAntenna) + "]," +
                           "\"target_body_yaw\":" + F(rig.BodyYaw) + "}";
                _ = Send(json);
            }
        }

        async Task Send(string json)
        {
            _sending = true;
            try { await _ws.SendAsync(new ArraySegment<byte>(Encoding.UTF8.GetBytes(json)), WebSocketMessageType.Text, true, _cts.Token); }
            catch (Exception e) { status = "send failed: " + e.Message; }
            finally { _sending = false; }
        }
    }
}
