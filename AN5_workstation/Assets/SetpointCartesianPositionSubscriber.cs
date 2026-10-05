using System.Collections;
using System.Globalization;
using UnityEngine;
using RosSharp.RosBridgeClient;
using RosString = RosSharp.RosBridgeClient.MessageTypes.Std.String;

/// Suscriptor para /setpoint_cartesian_position (x,y,z,rx,ry,rz), el target
/// del movimiento en curso segun el mock (o la posicion actual si no hay
/// ninguno activo -- ver mock_cmd_server.py, mismo timer que
/// current_cartesian_position). Mirror casi exacto de
/// CartesianPositionSubscriber.cs; existe por separado para no mezclar
/// "real" y "setpoint" en el mismo array y para poder graficar ambos lado a
/// lado en SecTrendGraphController.
public class SetpointCartesianPositionSubscriber : UnitySubscriber<RosString>
{
    private float[] lastSetpoint = new float[6];
    private bool _hasReceivedSetpoint;

    private RosConnector rosConnectorRef;
    private RosSocket lastSeenSocket;
    private bool hasSeenFirstSocket = false;

    protected override void Start()
    {
        Topic = "/setpoint_cartesian_position";
        base.Start();

        rosConnectorRef = GetComponent<RosConnector>();
        StartCoroutine(WatchForReconnect());
    }

    private IEnumerator WatchForReconnect()
    {
        var wait = new WaitForSeconds(0.5f);
        while (true)
        {
            yield return wait;

            if (rosConnectorRef == null) continue;
            RosSocket currentSocket = rosConnectorRef.RosSocket;
            if (currentSocket == null) continue;

            if (!hasSeenFirstSocket)
            {
                hasSeenFirstSocket = true;
                lastSeenSocket = currentSocket;
                continue;
            }

            if (currentSocket != lastSeenSocket)
            {
                lastSeenSocket = currentSocket;
                currentSocket.Subscribe<RosString>(Topic, ReceiveMessage, (int)(TimeStep * 1000));
                Debug.Log("[SetpointCartesianPositionSubscriber] RosSocket reconectado, re-suscrito a " + Topic);
            }
        }
    }

    protected override void ReceiveMessage(RosString message)
    {
        string[] parts = message.data.Split(',');
        if (parts.Length != 6) return;

        float[] positions = new float[6];
        for (int i = 0; i < 6; i++)
        {
            if (!float.TryParse(parts[i], NumberStyles.Float, CultureInfo.InvariantCulture, out float val))
                return;
            positions[i] = val;
        }
        lastSetpoint = positions;
        _hasReceivedSetpoint = true;
    }

    /// Modo Simulacion (LocalRobotSimulator): mismo camino que un mensaje de ROS.
    public void InjectLocal(string data)
    {
        ReceiveMessage(new RosString { data = data });
    }

    public float[] GetLastKnownSetpoint()
    {
        return (float[])lastSetpoint.Clone();
    }

    // Distinguishes "never received a message" (lastSetpoint still its float[6]
    // zero default) from a legitimately-published all-zero setpoint -- needed
    // because the real robot's fr_ros2 driver never publishes this topic at all
    // (only an5_mock_sim does), so on real hardware this stays false forever and
    // callers (SecTrendGraphController) know to fall back to something else
    // instead of graphing a permanently-frozen zero as if it were real data.
    public bool HasReceivedSetpoint => _hasReceivedSetpoint;
}
