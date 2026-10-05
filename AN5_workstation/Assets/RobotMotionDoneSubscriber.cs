using System.Collections;
using UnityEngine;
using RosSharp.RosBridgeClient;
using FRStateMsg = RosSharp.RosBridgeClient.MessageTypes.Frhal.FRState;

/// Suscriptor a nonrt_state_data (frhal_msgs/FRState), publicado tanto por el
/// driver real (fr_ros2/ros2_cmd_server, leído directo del socket de estado del
/// controlador -- ver fr_ros2/src/state_feedback.cpp) como por el emulador
/// (mock_cmd_server.py, que simula el mismo campo). Expone únicamente
/// robot_motion_done: 1 cuando el controlador reporta el movimiento en curso
/// como terminado, 0 mientras se sigue moviendo.
///
/// A diferencia de inferir "llegada" mirando si la posición YA ENTRÓ en una
/// tolerancia, esto es una señal explícita del propio controlador de que el
/// movimiento terminó de asentar -- un margen adicional útil sobre todo
/// contra el robot físico, cuya dinámica de asentamiento no es la del
/// emulador.
///
/// Igual que JointPositionSubscriber/CartesianPositionSubscriber/
/// SetpointCartesianPositionSubscriber: se apoya en el mismo RosConnector
/// compartido (requiere el componente en el mismo GameObject), procesa el
/// mensaje en Update() (hilo principal) en vez del hilo de red de RosSharp, y
/// se vuelve a suscribir si el RosSocket cambia por una reconexión.
public class RobotMotionDoneSubscriber : UnitySubscriber<FRStateMsg>
{
    private readonly object _pendingLock = new object();
    private int _pendingValue;
    private bool _hasPendingData;

    private bool _motionDone = true;
    private bool _hasReceivedMessage;

    private RosConnector rosConnectorRef;
    private RosSocket lastSeenSocket;
    private bool hasSeenFirstSocket = false;

    protected override void Start()
    {
        Topic = "nonrt_state_data";
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
                currentSocket.Subscribe<FRStateMsg>(Topic, ReceiveMessage, (int)(TimeStep * 1000));
                Debug.Log("[RobotMotionDoneSubscriber] RosSocket reconectado, re-suscrito a " + Topic);
            }
        }
    }

    // Invocado por RosSharp desde el hilo de red -- solo encola el valor crudo,
    // mismo patrón que JointPositionSubscriber para no tocar estado de Unity
    // fuera del hilo principal.
    protected override void ReceiveMessage(FRStateMsg message)
    {
        lock (_pendingLock)
        {
            _pendingValue = message.robot_motion_done;
            _hasPendingData = true;
        }
    }

    private void Update()
    {
        int value;
        lock (_pendingLock)
        {
            if (!_hasPendingData) return;
            value = _pendingValue;
            _hasPendingData = false;
        }
        _motionDone = value != 0;
        _hasReceivedMessage = true;
    }

    /// true si el último estado recibido reporta el movimiento como terminado.
    /// Antes del primer mensaje vale true (no bloquea) -- usar HasReceivedMessage
    /// para distinguir ese caso de una confirmación real.
    public bool IsMotionDone => _motionDone;

    /// false hasta que llegue el primer mensaje de nonrt_state_data (p.ej. si el
    /// nodo del driver no está corriendo, o la escena no tiene este componente
    /// cableado). Quien consuma IsMotionDone debe chequear esto primero para no
    /// confundir "sin dato todavía" con "movimiento confirmado".
    public bool HasReceivedMessage => _hasReceivedMessage;
}
