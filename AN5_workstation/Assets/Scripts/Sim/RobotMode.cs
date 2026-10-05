using System;
using UnityEngine;

/// Modo de operacion de la app: Ejec. Real (comandos y estado via ROS2 hacia el
/// controlador fisico) o Simulacion (todo local en Unity, sin ROS -- ver
/// LocalRobotSimulator). Lo fija ModeToggleController; lo consultan
/// Ros2CommandSender (a donde mandar cada comando), RosConnector (si conectarse)
/// y LocalRobotSimulator (si correr).
public static class RobotMode
{
    // Simulacion por defecto: es el modo con que arranca ModeToggleController.
    public static bool IsSimulation { get; private set; } = true;

    public static event Action<bool> Changed;

    public static void Set(bool simulation)
    {
        if (IsSimulation == simulation) return;
        IsSimulation = simulation;
        Changed?.Invoke(simulation);
    }

    // Con "Enter Play Mode Options" sin domain reload, los estaticos sobreviven
    // entre sesiones de Play: se vuelve al default en cada arranque.
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    static void ResetOnLoad()
    {
        IsSimulation = true;
        Changed = null;
    }
}
