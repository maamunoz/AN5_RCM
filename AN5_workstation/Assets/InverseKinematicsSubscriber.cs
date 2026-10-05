/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)                   
*******************/
using System.Collections;
using UnityEngine;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Std;
using System;

// Alias para diferenciar entre RosSharp.RosBridgeClient.MessageTypes.Std.String y System.String
using RosString = RosSharp.RosBridgeClient.MessageTypes.Std.String;

// Clase para suscribirse a resultados de cinemática inversa y procesarlos.
public class InverseKinematicsSubscriber : UnitySubscriber<RosString>
{
    // Referencia al script CartesianStateWriterNew para actualizar las posiciones articulares
    public CartesianStateWriterNew cartesianStateWriter;

    // Evento para notificar a otros scripts cuando se recibe un resultado de cinemática inversa
    public System.Action<string> OnInverseKinematicsResultReceived;

    // Detección propia de reconexión (mismo patrón que JointPositionSubscriber /
    // CartesianPositionSubscriber / SetpointCartesianPositionSubscriber): cada
    // reconexión de RosConnector crea un RosSocket nuevo, y la suscripción original
    // (hecha una sola vez en UnitySubscriber<T>.Start()) queda apuntando al socket
    // viejo -- sin esto, un ciclo de reconexión (ej. el nodo ROS/rosbridge
    // reiniciado) deja a Unity sordo en output_joint_position para siempre: los
    // requests de SecTrajController/CartesianStateWriterNew se siguen publicando
    // bien en input_cartesian_position, pero ninguna respuesta vuelve a llegar,
    // así que todo punto termina en timeout aunque MATLAB/el mock esté contestando.
    private RosConnector rosConnectorRef;
    private RosSocket lastSeenSocket;
    private bool hasSeenFirstSocket = false;

    // Inicializa la suscripción al tópico ROS correspondiente.
    protected override void Start()
    {
        base.Start();
        // Configurar el tópico al que se suscribirá
        Topic = "output_joint_position"; // Tópico que recibe el resultado de la inversa en posiciones articulares

        rosConnectorRef = GetComponent<RosConnector>();
        StartCoroutine(WatchForReconnect());
    }

    // Vigila el RosSocket vigente; si cambia (reconexión), vuelve a suscribirse a
    // output_joint_position sobre el socket nuevo. La primera conexión la maneja
    // la suscripción original de UnitySubscriber<T>, así que aquí solo se actúa
    // ante cambios posteriores.
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
                Debug.Log("[InverseKinematicsSubscriber] RosSocket reconectado, re-suscrito a " + Topic);
            }
        }
    }

    /// Modo Simulacion (LocalRobotSimulator): IK resuelta local, mismo camino que ROS.
    public void InjectLocal(string data)
    {
        ReceiveMessage(new RosString { data = data });
    }

    // Método que se llama al recibir un mensaje del tópico suscrito.
 protected override void ReceiveMessage(RosString message)
{
    Debug.Log("Recibido en output_joint_position: " + message.data);

    if (cartesianStateWriter != null)
    {
        cartesianStateWriter.ReceiveJointPositions(message.data);
    }
    else
    {
        Debug.LogError("CartesianStateWriterNew no está asignado en el Inspector.");
    }

    OnInverseKinematicsResultReceived?.Invoke(message.data);
}
}