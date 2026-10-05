using System.Collections.Generic;
using System.Globalization;
using System.Text.RegularExpressions;
using UnityEngine;

/// Robot simulado 100% local para el modo Simulacion (RobotMode.IsSimulation):
/// reemplaza al par rosbridge + an5_mock_sim/mock_cmd_server.py sin ROS de por
/// medio. Ros2CommandSender le entrega cada comando en vez de publicarlo en
/// /api_command, y este componente le inyecta el estado resultante a los mismos
/// suscriptores que en modo real reciben current_joint_position,
/// current_cartesian_position, setpoint_cartesian_position y nonrt_state_data --
/// asi todos los paneles (sliders, cola, trayectorias, graficos, fulcro) funcionan
/// sin cambios y sin saber en que modo estan.
///
/// Port de mock_cmd_server.py: misma gramatica de comandos (JNTPoint, CARTPoint,
/// MoveJ/MoveL, SplineStart/SplinePTP/SplineEnd, StopMotion y el mismo conjunto de
/// comandos aceptados como no-op), misma interpolacion articular (duracion segun el
/// joint mas lento a speed%, acotada a [0.2s, 8s]; ease in/out con empalme continuo
/// entre segmentos de una cadena de SplinePTP) y la misma pose inicial que
/// sim.launch.py. Dos diferencias deliberadas:
///   - Los puntos CART se resuelven con FR5AnalyticIK (IK local de la app) en vez del
///     solver numerico del mock.
///   - La pose cartesiana publicada sale de LocalForwardKinematics (cadena DH), la
///     misma que usa FR5AnalyticIK -- el mock usaba la cadena del URDF, que difiere
///     unos cm en el ultimo eslabon; con DH un punto de trayectoria cargado en
///     cartesianas se ve exactamente en la posicion pedida.
public class LocalRobotSimulator : MonoBehaviour
{
    public static LocalRobotSimulator Instance { get; private set; }

    [Tooltip("Pose articular inicial (grados), igual que initial_joint_positions_deg de sim.launch.py.")]
    public float[] initialJointsDeg = { 0f, -90f, 90f, -90f, 90f, 0f };

    [Tooltip("Frecuencia de publicacion del estado hacia los suscriptores (Hz), como joint_states_rate_hz del mock.")]
    public float publishRateHz = 50f;

    const float MinMoveDuration = 0.2f;
    const float MaxMoveDuration = 8f;

    // Velocidad maxima por joint (deg/s): JOINT_LIMITS[i][2] del mock (3.15/3.20 rad/s).
    static readonly float[] MaxJointSpeedDegPerSec =
        { 180.48f, 180.48f, 180.48f, 183.35f, 183.35f, 183.35f };

    static readonly HashSet<string> NoOpCommands = new HashSet<string>
    {
        "DragTeachSwitch", "RobotEnable", "SetSpeed", "Mode", "SetToolCoord",
        "SetToolList", "SetExToolCoord", "SetExToolList", "SetWObjCoord",
        "SetWObjList", "SetLoadWeight", "SetLoadCoord", "SetRobotInstallPos",
        "SetRobotInstallAngle", "SetAnticollision", "SetCollisionStrategy",
        "SetLimitPositive", "SetLimitNegative", "ResetAllError",
        "FrictionCompensationOnOff", "SetFrictionValue_level",
        "SetFrictionValue_wall", "SetFrictionValue_ceiling",
        "SetFrictionValue_freedom", "ActGripper", "MoveGripper", "SetDO",
        "SetToolDO", "SetAO", "SetToolAO", "StartJOG", "StopJOG", "ImmStopJOG",
        "MoveC", "Circle", "ServoJTStart", "ServoJT", "ServoJTEnd",
        "NewSplineStart", "NewSplinePoint", "NewSplineEnd",
        "PointsOffsetEnable", "PointsOffsetDisable", "ProgramRun",
    };

    static readonly Regex CommandRegex = new Regex(@"^([A-Za-z_]+)\((.*)\)$");
    static readonly Regex PointRefRegex = new Regex(@"^(JNT|CART)(\d+)$");

    // --- Estado de movimiento (grados), igual que el mock ---
    float[] _currentJnt = new float[6];
    float[] _moveFrom = new float[6];
    float[] _moveTarget = new float[6];
    bool _moveActive;
    float _moveStart;
    float _moveDuration;
    bool _moveEaseIn = true;
    bool _moveEaseOut = true;
    bool _splineChainActive;

    readonly List<float[]> _jntPoints = new List<float[]>();
    readonly List<float[]> _cartPoints = new List<float[]>();
    readonly List<(float[] targetDeg, float speedPct)> _splineQueue = new List<(float[], float)>();

    bool _initialized;
    float _nextPublishTime;

    JointPositionSubscriber _jointSub;
    CartesianPositionSubscriber _cartSub;
    SetpointCartesianPositionSubscriber _setpointSub;
    RobotMotionDoneSubscriber _motionDoneSub;
    MGD_Subscriber _fkSub;
    InverseKinematicsSubscriber _ikSub;

    void Awake()
    {
        Instance = this;
    }

    void OnEnable()
    {
        RobotMode.Changed += OnModeChanged;
    }

    void OnDisable()
    {
        RobotMode.Changed -= OnModeChanged;
    }

    void OnDestroy()
    {
        if (Instance == this) Instance = null;
    }

    // Al entrar en Simulacion despues de haber estado en Ejec. Real, el robot
    // simulado arranca donde estaba el real (ultima posicion conocida), no en la
    // pose inicial, para que el cambio de modo no haga saltar el modelo.
    void OnModeChanged(bool simulation)
    {
        if (!simulation || !_initialized) return;
        ResolveSubscribers();
        float[] last = _jointSub != null ? _jointSub.GetLastKnownPositions() : null;
        if (last != null && last.Length == 6)
            ResetTo(last);
    }

    void ResetTo(float[] jointsDeg)
    {
        _currentJnt = (float[])jointsDeg.Clone();
        _moveFrom = (float[])jointsDeg.Clone();
        _moveTarget = (float[])jointsDeg.Clone();
        _moveActive = false;
        _splineChainActive = false;
        _splineQueue.Clear();
    }

    void Update()
    {
        if (!RobotMode.IsSimulation) return;

        if (!_initialized)
        {
            _initialized = true;
            ResetTo(initialJointsDeg);
        }

        Tick();

        if (Time.time >= _nextPublishTime)
        {
            _nextPublishTime = Time.time + 1f / Mathf.Max(1f, publishRateHz);
            Publish();
        }
    }

    // ------------------------------------------------------------------ //
    //  Comandos (/api_command)
    // ------------------------------------------------------------------ //
    public bool ProcessCommand(string cmd)
    {
        if (string.IsNullOrEmpty(cmd)) return false;
        var m = CommandRegex.Match(cmd.Trim());
        if (!m.Success)
        {
            Debug.LogWarning($"[LocalRobotSimulator] Formato de comando invalido: '{cmd}'");
            return false;
        }

        string func = m.Groups[1].Value;
        string para = m.Groups[2].Value;

        switch (func)
        {
            case "JNTPoint":    return DefinePoint(_jntPoints, para, "JNTPoint");
            case "CARTPoint":   return DefinePoint(_cartPoints, para, "CARTPoint");
            case "MoveJ":
            case "MoveL":       return Move(para, func);
            case "SplineStart": _splineQueue.Clear(); return true;
            case "SplinePTP":   return SplinePtp(para);
            case "SplineEnd":   return true; // Tick() arranca la cola sola
            case "StopMotion":  StopMotion(); return true;
            case "GET":         return true;
        }

        if (NoOpCommands.Contains(func)) return true;

        Debug.LogWarning($"[LocalRobotSimulator] Comando no reconocido: '{func}'");
        return false;
    }

    bool DefinePoint(List<float[]> points, string para, string name)
    {
        var parts = para.Split(',');
        if (parts.Length != 7)
        {
            Debug.LogWarning($"[LocalRobotSimulator] {name} requiere 7 parametros, se recibieron {parts.Length}.");
            return false;
        }
        if (!int.TryParse(parts[0], NumberStyles.Integer, CultureInfo.InvariantCulture, out int idx))
            return false;
        var values = new float[6];
        for (int i = 0; i < 6; i++)
            if (!float.TryParse(parts[i + 1], NumberStyles.Float, CultureInfo.InvariantCulture, out values[i]))
                return false;

        if (idx <= 0 || idx > points.Count + 1)
        {
            Debug.LogWarning($"[LocalRobotSimulator] {name}: indice {idx} fuera de rango.");
            return false;
        }
        if (idx <= points.Count) points[idx - 1] = values;
        else points.Add(values);
        return true;
    }

    // Resuelve "JNT<n>" / "CART<n>" a un objetivo articular (grados). Los CART se
    // resuelven con IK local partiendo de seedDeg.
    bool ResolvePointRef(string head, float[] seedDeg, string context, out float[] targetDeg)
    {
        targetDeg = null;
        var m = PointRefRegex.Match(head);
        if (!m.Success)
        {
            Debug.LogWarning($"[LocalRobotSimulator] {context}: punto invalido '{head}'.");
            return false;
        }
        bool isCart = m.Groups[1].Value == "CART";
        int idx = int.Parse(m.Groups[2].Value, CultureInfo.InvariantCulture);
        var points = isCart ? _cartPoints : _jntPoints;
        if (idx <= 0 || idx > points.Count)
        {
            Debug.LogWarning($"[LocalRobotSimulator] {context}: indice {head} fuera de rango.");
            return false;
        }

        float[] p = points[idx - 1];
        if (!isCart)
        {
            targetDeg = (float[])p.Clone();
            return true;
        }

        var result = FR5AnalyticIK.Solve(p[0], p[1], p[2], p[3], p[4], p[5], seedDeg);
        if (!result.Success)
        {
            Debug.LogWarning($"[LocalRobotSimulator] {context}({head}): {result.FailureReason}; no se mueve.");
            return false;
        }
        targetDeg = result.JointsDeg;
        return true;
    }

    static float ParseSpeed(string s)
    {
        return float.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out float v) ? v : 50f;
    }

    bool Move(string para, string func)
    {
        var parts = para.Split(',');
        if (parts.Length < 2)
        {
            Debug.LogWarning($"[LocalRobotSimulator] {func}: se esperaba \"JNT<idx>,speed\".");
            return false;
        }
        if (!ResolvePointRef(parts[0], _currentJnt, func, out float[] target)) return false;
        StartMoveTo(target, ParseSpeed(parts[1]), true, true);
        return true;
    }

    bool SplinePtp(string para)
    {
        var parts = para.Split(',');
        if (parts.Length < 2)
        {
            Debug.LogWarning("[LocalRobotSimulator] SplinePTP: se esperaba \"JNT<idx>,speed\".");
            return false;
        }
        // Igual que el mock: los CART se resuelven desde el ultimo punto ya
        // encolado, para que toda la cadena quede en una misma rama del brazo.
        // Se guarda una COPIA del objetivo: Unity reusa JNTPoint(1..5) por lote.
        float[] seed = _splineQueue.Count > 0 ? _splineQueue[_splineQueue.Count - 1].targetDeg : _currentJnt;
        if (!ResolvePointRef(parts[0], seed, "SplinePTP", out float[] target)) return false;
        _splineQueue.Add((target, ParseSpeed(parts[1])));
        return true;
    }

    void StopMotion()
    {
        _moveTarget = (float[])_currentJnt.Clone();
        _moveActive = false;
        _splineChainActive = false;
        _splineQueue.Clear();
    }

    void StartMoveTo(float[] targetDeg, float speedPct, bool easeIn, bool easeOut)
    {
        var clamped = new float[6];
        for (int i = 0; i < 6; i++)
            clamped[i] = Mathf.Clamp(targetDeg[i], Fr5DhTable.JointMinDeg[i], Fr5DhTable.JointMaxDeg[i]);

        _moveFrom = (float[])_currentJnt.Clone();
        _moveTarget = clamped;
        _moveStart = Time.time;
        _moveDuration = EstimateDuration(_currentJnt, clamped, speedPct);
        _moveEaseIn = easeIn;
        _moveEaseOut = easeOut;
        _moveActive = true;
        _splineChainActive = !easeOut;
    }

    static float EstimateDuration(float[] from, float[] to, float speedPct)
    {
        speedPct = Mathf.Clamp(speedPct, 1f, 100f);
        float worst = 0f;
        for (int i = 0; i < 6; i++)
        {
            float vmax = MaxJointSpeedDegPerSec[i] * (speedPct / 100f);
            if (vmax <= 1e-6f) continue;
            worst = Mathf.Max(worst, Mathf.Abs(to[i] - from[i]) / vmax);
        }
        return Mathf.Clamp(worst, MinMoveDuration, MaxMoveDuration);
    }

    // Mismo esquema que mock._tick_joint_states: avanza el movimiento activo y,
    // cuando termina, arranca el siguiente punto de la cola de spline.
    void Tick()
    {
        if (_moveActive)
        {
            float t = _moveDuration <= 0f ? 1f : Mathf.Min(1f, (Time.time - _moveStart) / _moveDuration);
            float eased;
            if (_moveEaseIn && _moveEaseOut) eased = t * t * (3f - 2f * t);
            else if (_moveEaseIn)            eased = t * t;
            else if (_moveEaseOut)           eased = 1f - (1f - t) * (1f - t);
            else                             eased = t;

            for (int i = 0; i < 6; i++)
                _currentJnt[i] = _moveFrom[i] + (_moveTarget[i] - _moveFrom[i]) * eased;

            if (t >= 1f) _moveActive = false;
        }

        if (!_moveActive && _splineQueue.Count > 0)
        {
            var next = _splineQueue[0];
            _splineQueue.RemoveAt(0);
            bool easeIn = !_splineChainActive;
            bool easeOut = _splineQueue.Count == 0;
            StartMoveTo(next.targetDeg, next.speedPct, easeIn, easeOut);
        }
    }

    // ------------------------------------------------------------------ //
    //  Estado hacia los suscriptores (lo que en modo real llega por ROS)
    // ------------------------------------------------------------------ //
    void ResolveSubscribers()
    {
        if (_jointSub == null)      _jointSub      = FindAnyObjectByType<JointPositionSubscriber>(FindObjectsInactive.Include);
        if (_cartSub == null)       _cartSub       = FindAnyObjectByType<CartesianPositionSubscriber>(FindObjectsInactive.Include);
        if (_setpointSub == null)   _setpointSub   = FindAnyObjectByType<SetpointCartesianPositionSubscriber>(FindObjectsInactive.Include);
        if (_motionDoneSub == null) _motionDoneSub = FindAnyObjectByType<RobotMotionDoneSubscriber>(FindObjectsInactive.Include);
        if (_fkSub == null)         _fkSub         = FindAnyObjectByType<MGD_Subscriber>(FindObjectsInactive.Include);
        if (_ikSub == null)         _ikSub         = FindAnyObjectByType<InverseKinematicsSubscriber>(FindObjectsInactive.Include);
    }

    void Publish()
    {
        ResolveSubscribers();

        float[] setpoint = _moveActive ? _moveTarget : _currentJnt;

        _jointSub?.InjectLocal(Csv(_currentJnt));
        _cartSub?.InjectLocal(Csv(LocalForwardKinematics.CartesianFromJointsDeg(_currentJnt)));
        _setpointSub?.InjectLocal(Csv(LocalForwardKinematics.CartesianFromJointsDeg(setpoint)));
        _motionDoneSub?.InjectLocalMotionDone(!_moveActive && _splineQueue.Count == 0);
    }

    static string Csv(float[] v)
    {
        var sb = new System.Text.StringBuilder();
        for (int i = 0; i < v.Length; i++)
        {
            if (i > 0) sb.Append(',');
            sb.Append(v[i].ToString("F2", CultureInfo.InvariantCulture));
        }
        return sb.ToString();
    }

    // ------------------------------------------------------------------ //
    //  Topicos de cinematica (input_joint_position / input_cartesian_position)
    //  -- en modo real los atienden MGD_Node y MATLAB; aca se resuelven local
    //  y se entregan a los mismos suscriptores de respuesta.
    // ------------------------------------------------------------------ //
    public void ProcessTopic(string topic, string data, string directaTopic, string inversaTopic)
    {
        string t = topic.TrimStart('/');
        ResolveSubscribers();

        if (t == directaTopic.TrimStart('/'))
        {
            if (!TryParse6(data, out float[] joints)) return;
            _fkSub?.InjectLocal(Csv(LocalForwardKinematics.CartesianFromJointsDeg(joints)));
            return;
        }

        if (t == inversaTopic.TrimStart('/'))
        {
            if (!TryParse6(data, out float[] cart)) return;
            var result = FR5AnalyticIK.Solve(cart[0], cart[1], cart[2], cart[3], cart[4], cart[5], _currentJnt);
            _ikSub?.InjectLocal(result.Success ? Csv(result.JointsDeg) : "ERROR:" + result.FailureReason);
            return;
        }

        Debug.Log($"[LocalRobotSimulator] Topico '{topic}' sin equivalente local, ignorado: {data}");
    }

    static bool TryParse6(string data, out float[] values)
    {
        values = new float[6];
        var parts = data.Split(',');
        if (parts.Length != 6) return false;
        for (int i = 0; i < 6; i++)
            if (!float.TryParse(parts[i], NumberStyles.Float, CultureInfo.InvariantCulture, out values[i]))
                return false;
        return true;
    }

    // Para inspeccion/pruebas.
    public float[] CurrentJointsDeg => (float[])_currentJnt.Clone();
    public bool IsMoving => _moveActive || _splineQueue.Count > 0;
}
