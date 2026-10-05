/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)
*******************/
using System.Collections;
using System.Globalization;
using UnityEngine;
using RosSharp.RosBridgeClient;
using RosString = RosSharp.RosBridgeClient.MessageTypes.Std.String;

/// Suscriptor para /current_cartesian_position, que recibe x,y,z,rx,ry,rz
/// y guarda la última posición en un array float[6]. Además, activa o desactiva
/// la señal de interpolación según se pause o reanude la lectura del tópico.
public class CartesianPositionSubscriber : UnitySubscriber<RosString>
{
    // Para pausar/reanudar la lectura de mensajes.
    private bool isUpdating = true;
    /// Indica si la interpolación debe estar activada. Cuando isUpdating es false,
    /// se activa la interpolación; cuando se reanuda, se desactiva.
    public bool InterpolationEnabled { get; private set; } = false;

    // Última posición [x, y, z, rx, ry, rz].
    private float[] lastCartesianPositions = new float[6];

    // Cada reconexión de RosConnector crea un RosSocket nuevo, y la suscripción
    // original (hecha en UnitySubscriber<T>.Start() contra el socket viejo) deja de
    // recibir mensajes para siempre: current_cartesian_position se queda congelado
    // (o en 0 si nunca llegó nada desde la última reconexión). JointPositionSubscriber
    // ya tiene este mismo workaround; aquí replicamos la misma vigilancia.
    private RosConnector rosConnectorRef;
    private RosSocket lastSeenSocket;
    private bool hasSeenFirstSocket = false;

    protected override void Start()
    {
        // Debe fijarse antes de base.Start(), que lanza en un hilo aparte la
        // suscripción inicial usando este mismo campo Topic.
        Topic = "/current_cartesian_position";
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
                Debug.Log("[CartesianPositionSubscriber] RosSocket reconectado, re-suscrito a " + Topic);
            }
        }
    }

    /// Se ejecuta cada vez que llega un mensaje del tópico.
    /// Se parsean los valores y se actualiza la posición interna.
    protected override void ReceiveMessage(RosString message)
    {
        if (!isUpdating)
            return;

        string[] parts = message.data.Split(',');
        if (parts.Length != 6)
            return;

        float[] positions = new float[6];
        for (int i = 0; i < 6; i++)
        {
            if (float.TryParse(parts[i], NumberStyles.Float, CultureInfo.InvariantCulture, out float val))
            {
                positions[i] = Mathf.Round(val * 100f) / 100f;
            }
            else
            {
                return;
            }
        }
        lastCartesianPositions = positions;
    }
    /// Modo Simulacion (LocalRobotSimulator): mismo camino que un mensaje de ROS.
    public void InjectLocal(string data)
    {
        ReceiveMessage(new RosString { data = data });
    }

    /// Devuelve una copia de la última posición conocida.
    public float[] GetLastKnownCartesianPositions()
    {
        return (float[])lastCartesianPositions.Clone();
    }

    /// Detiene la actualización de mensajes y activa la interpolación.
    public void StopUpdating()
    {
        isUpdating = false;
        InterpolationEnabled = true;
    }

    /// Reanuda la actualización de mensajes y desactiva la interpolación.
    public void StartUpdating()
    {
        isUpdating = true;
        InterpolationEnabled = false;
    }
}
