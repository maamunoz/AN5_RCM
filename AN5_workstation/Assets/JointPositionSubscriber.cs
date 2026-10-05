/*******************
Autores:    Angel Garzon Sarzosa (ahgarzon@unicauca.edu.co)
            Jhoan Simei Sarria (simei@unicauca.edu.co)
*******************/

using System;
using System.Collections;
using System.Globalization;
using UnityEngine;
using RosSharp.RosBridgeClient;
// Alias para el tipo de mensaje ROS estándar String
using RosString = RosSharp.RosBridgeClient.MessageTypes.Std.String;

public class JointPositionSubscriber : UnitySubscriber<RosString>
{
    // Array de objetos encargados de escribir y actualizar el estado de las articulaciones en la representación visual.
    public JointStateWriter[] jointStateWriters;

    // Flag que determina si se deben procesar y actualizar las posiciones recibidas.
    private bool isUpdating = true;

    /// When false, joint angles are computed and the event fires but fr5v6 is not driven.
    public bool driveRobotModel = true;

    // Evento que notifica a otros componentes cuando se actualizan las posiciones de las articulaciones.
    public event Action<float[]> OnJointPositionsUpdated;

    // Almacena la última posición conocida de cada articulación.
    private float[] lastPositions;

    // Indica si ya se aplicó la pose inicial del robot al modelo (una sola vez al arrancar).
    private bool hasAppliedInitialPose = false;

    // Detección propia de reconexión: cada reconexión de RosConnector crea un RosSocket
    // nuevo, y la suscripción original (en UnitySubscriber<T>) queda apuntando al socket
    // viejo. Esto NO toca RosConnector ni el envío de comandos: solo vuelve a suscribir
    // la lectura de current_joint_position cuando detecta que el socket cambió.
    private RosConnector rosConnectorRef;
    private RosSocket lastSeenSocket;
    private bool hasSeenFirstSocket = false;

    // RosSharp invoca ReceiveMessage() desde el hilo de red de WebSocketSharp, no desde
    // el hilo principal de Unity. Escribir Transform/UI (jointStateWriters, sliders vía
    // OnJointPositionsUpdated) desde ese hilo es inseguro y provocaba que el modelo se
    // quedara congelado en la primera posición: el evento de Unity UI queda diferido al
    // siguiente frame de EventSystem.Update(), momento en el cual guardas como
    // _programmaticSliderUpdate (en ControlArticular) ya se revirtieron, dejando pasar
    // el callback como si fuera una edición manual del usuario. Por eso el mensaje crudo
    // solo se encola aquí, y el procesamiento real ocurre en Update() (hilo principal).
    private readonly object _pendingLock = new object();
    private string _pendingData;
    private bool _hasPendingData;

    // Configuración para el modo "freeze" que ignora cambios pequeños.
    [Header("Modo Freeze (para ignorar pequeños cambios)")]
    [Tooltip("Si está activo, solo actualiza el URDF cuando la diferencia > freezeThresholdDeg en alguna articulación.")]
    public bool freezeMode = false;

    [Tooltip("Umbral de diferencia en grados para salir del freeze en una articulación (ej. 0.5)")]
    public float freezeThresholdDeg = 0.5f;

    // Método de inicialización del componente.
    protected override void Start()
    {
        // Se invoca el método Start de la clase base para inicializar la suscripción.
        base.Start();

        // Se inicializa el arreglo lastPositions según la cantidad de jointStateWriters asignados.
        if (jointStateWriters != null && jointStateWriters.Length > 0)
            lastPositions = new float[jointStateWriters.Length];
        else
            // Si no se han asignado escritores, se asume un arreglo de 6 articulaciones por defecto.
            lastPositions = new float[6];

        // Se define el tópico de ROS al que se suscribirá este componente.
        Topic = "current_joint_position";

        rosConnectorRef = GetComponent<RosConnector>();
        StartCoroutine(WatchForReconnect());
    }

    // Vigila el RosSocket vigente; si cambia (reconexión), vuelve a suscribirse a
    // current_joint_position sobre el socket nuevo. La primera conexión la maneja
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
                Debug.Log("[JointPositionSubscriber] RosSocket reconectado, re-suscrito a " + Topic);
            }
        }
    }

    // Método invocado por RosSharp desde el hilo de red. Solo guarda el payload crudo;
    // el procesamiento real (que toca jointStateWriters/UI) se hace en Update().
    protected override void ReceiveMessage(RosString message)
    {
        lock (_pendingLock)
        {
            _pendingData = message.data;
            _hasPendingData = true;
        }
    }

    /// Modo Simulacion (LocalRobotSimulator): entrega el mismo CSV que publicaria
    /// current_joint_position, por el mismo camino que un mensaje de ROS.
    public void InjectLocal(string data)
    {
        ReceiveMessage(new RosString { data = data });
    }

    private void Update()
    {
        string data;
        lock (_pendingLock)
        {
            if (!_hasPendingData)
                return;
            data = _pendingData;
            _hasPendingData = false;
        }
        ProcessMessage(data);
    }

    // Procesa el mensaje en el hilo principal de Unity.
    private void ProcessMessage(string data)
    {
        // Si la actualización está deshabilitada, se ignora el mensaje.
        if (!isUpdating)
            return;

        // Se separa el mensaje recibido utilizando la coma como delimitador.
        string[] parts = data.Split(',');
        // Se verifica que la cantidad de valores coincida con el número de articulaciones a actualizar.
        if (parts.Length != jointStateWriters.Length)
            return;

        // Se parsean y almacenan las nuevas posiciones de las articulaciones.
        float[] newPositions = new float[jointStateWriters.Length];
        for (int i = 0; i < jointStateWriters.Length; i++)
        {
            // Si no se puede convertir el valor, se abandona la actualización.
            if (!float.TryParse(parts[i], NumberStyles.Float, CultureInfo.InvariantCulture, out float degValue))
                return;
            // Se redondea el valor a dos decimales para evitar actualizaciones con fluctuaciones insignificantes.
            newPositions[i] = Mathf.Round(degValue * 100f) / 100f;
        }

        if (lastPositions == null || lastPositions.Length != jointStateWriters.Length)
            lastPositions = new float[jointStateWriters.Length];

        // Se calcula la diferencia máxima entre las nuevas posiciones y las posiciones almacenadas previamente.
        float computedMaxDiff = 0f;
        for (int i = 0; i < newPositions.Length; i++)
        {
            float diff = Mathf.Abs(newPositions[i] - lastPositions[i]);
            if (diff > computedMaxDiff)
                computedMaxDiff = diff;
        }

        // Al recibir el primer mensaje tras arrancar, se sincroniza el modelo fr5 con la pose real
        // del robot inmediatamente, sin importar el estado de driveRobotModel (que depende de qué
        // panel esté activo). Así el modelo siempre arranca mostrando la pose actual del robot.
        if (!hasAppliedInitialPose)
        {
            hasAppliedInitialPose = true;
            foreach (var writer in jointStateWriters)
            {
                writer.InterpolationEnabled = false;
                writer.UnlockWriting();
            }
            for (int i = 0; i < jointStateWriters.Length; i++)
            {
                jointStateWriters[i].Write(newPositions[i] * Mathf.Deg2Rad);
            }
            lastPositions = newPositions;
            OnJointPositionsUpdated?.Invoke(newPositions);
            return;
        }

        // While a trajectory is executing, StartLiveTracking() polls ApplyToModel()
        // on its own timer (every ~0.1s, always InterpolationEnabled=false) regardless
        // of tab/driveRobotModel -- that's what keeps fr5 in sync on the main panel,
        // where driveRobotModel is false and nothing else writes to jointStateWriters.
        // On the monitoring tab driveRobotModel is true, so the block below ALSO ran
        // on every incoming message: for small (<0.5deg) diffs it started a smooth
        // 1s lerp (InterpolationEnabled=true), but the next ApplyToModel() tick (up to
        // ~0.1s later) forced InterpolationEnabled=false again, aborting that lerp
        // mid-flight and snapping to whatever lastPositions was at that instant --
        // the two writers fighting over the same joints is what looked like jitter.
        // Skip this method's own write while live-tracking owns the model; just keep
        // lastPositions in sync so GetLastKnownPositions()/ApplyToModel() stay correct.
        if (driveRobotModel && _liveTrackCoroutine == null)
        {
            // Si la diferencia máxima es mayor o igual a 0.5 grados, se actualizan los escritores sin interpolación.
            if (computedMaxDiff >= 0.5f)
            {
                foreach (var writer in jointStateWriters)
                {
                    writer.InterpolationEnabled = false;
                    writer.UnlockWriting();
                }
                for (int i = 0; i < jointStateWriters.Length; i++)
                {
                    float jointRad = newPositions[i] * Mathf.Deg2Rad;
                    jointStateWriters[i].Write(jointRad);
                }
                lastPositions = newPositions;
            }
            else if (freezeMode && computedMaxDiff < freezeThresholdDeg)
            {
                // FIX: esta rama existe para no reescribir el URDF ante cambios
                // minúsculos (ver el tooltip de freezeMode: "solo actualiza el
                // URDF cuando la diferencia > freezeThresholdDeg") -- es una
                // decisión sobre el MODELO VISUAL, no sobre qué posición se
                // considera conocida. Antes retornaba sin actualizar
                // lastPositions, así que GetLastKnownPositions() se quedaba
                // con un valor de mitad de movimiento, desfasado hasta
                // freezeThresholdDeg (0.5° por defecto) del real -- causaba
                // que cualquier espera de llegada que dependa de
                // GetLastKnownPositions() (como
                // SecTrajController/SecCoordQueueController) leyera un
                // "alcanzado" que nunca circuló como estado final real.
                lastPositions = newPositions;
                OnJointPositionsUpdated?.Invoke(newPositions);
                return;
            }
            else
            {
                // Separate pre-existing bug, found while investigating the jitter
                // above: this branch set InterpolationEnabled=true but never
                // actually called Write() with the new angle, so sub-0.5deg
                // updates (fine manual jogging while on the monitoring tab, with
                // no trajectory executing) never reached the model at all until
                // enough of them accumulated to cross the 0.5deg threshold above.
                foreach (var writer in jointStateWriters)
                {
                    writer.InterpolationEnabled = true;
                    writer.UnlockWriting();
                }
                for (int i = 0; i < jointStateWriters.Length; i++)
                {
                    float jointRad = newPositions[i] * Mathf.Deg2Rad;
                    jointStateWriters[i].Write(jointRad);
                }
                lastPositions = newPositions;
            }
        }
        else
        {
            lastPositions = newPositions;
        }
        // Se invoca el evento para notificar a los suscriptores de la actualización de posiciones.
        OnJointPositionsUpdated?.Invoke(newPositions);
    }

    // Retorna una copia de las últimas posiciones conocidas de las articulaciones.
    public float[] GetLastKnownPositions()
    {
        return (float[])lastPositions.Clone();
    }

    /// Force-writes lastPositions to all JointStateWriters immediately, ignoring the diff threshold.
    public void ApplyToModel()
    {
        if (jointStateWriters == null) return;
        if (lastPositions == null || lastPositions.Length != jointStateWriters.Length)
            lastPositions = new float[jointStateWriters.Length];
        foreach (var writer in jointStateWriters)
        {
            writer.InterpolationEnabled = false;
            writer.UnlockWriting();
        }
        for (int i = 0; i < jointStateWriters.Length; i++)
            jointStateWriters[i].Write(lastPositions[i] * Mathf.Deg2Rad);
    }

    private Coroutine _liveTrackCoroutine;

    /// Starts polling ApplyToModel() at a fixed interval, so fr5 tracks the robot
    /// while a queued trajectory is executing. Independent of driveRobotModel/tab state.
    public void StartLiveTracking(float intervalSeconds = 0.1f)
    {
        StopLiveTracking();
        _liveTrackCoroutine = StartCoroutine(LiveTrackLoop(intervalSeconds));
    }

    public void StopLiveTracking()
    {
        if (_liveTrackCoroutine != null)
        {
            StopCoroutine(_liveTrackCoroutine);
            _liveTrackCoroutine = null;
        }
    }

    private IEnumerator LiveTrackLoop(float intervalSeconds)
    {
        var wait = new WaitForSeconds(intervalSeconds);
        while (true)
        {
            ApplyToModel();
            yield return wait;
        }
    }

    // Desactiva la actualización de posiciones, ignorando futuros mensajes.
    public void StopUpdating()
    {
        isUpdating = false;
    }

    // Activa la actualización de posiciones para procesar nuevos mensajes.
    public void StartUpdating()
    {
        isUpdating = true;
    }
}
