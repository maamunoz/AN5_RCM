using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

/// Jog con restriccion de punto de fulcro/trocar (RCM) -- porta el flujo de
/// github.com/Juan-Sebastian-Silva/Robot-Quir-rgico-AN5- (IKController.cs +
/// FulcroCommandSender.cs + FulcroPanel.cs): activar modo fulcro, mover la punta
/// de la herramienta con sliders XYZ mientras el eje de insercion pasa siempre
/// por el punto de trocar, modo J6 independiente para rotar solo la camara, y un
/// boton "Enviar" que manda la solucion actual al robot real.
///
/// Diferencias respecto al original:
///   - La geometria RCM y la IK se resuelven en el mismo marco cartesiano
///     (x,y,z en mm, rx,ry,rz en grados) que ya usan el jog panel y
///     LocalForwardKinematics -- no en Transforms de la escena. Ver el
///     comentario de FR5AnalyticIK para por que: evita tener que calibrar una
///     conversion mundo-Unity <-> marco-DH, y el fr5v6 visual ya diverge un
///     poco de la cadena DH (divergencia preexistente de este proyecto,
///     cuantificada en su momento por P9KinematicConsistency). trocarXYZ vive
///     en ese marco: o se registra con FIJAR TROCAR (como un RCM real), o sale
///     de trocarObject, un cilindro de la escena convertido via base_link (el
///     modelo y la cadena DH comparten base y eje J6, verificado a 0.1 mm).
///   - Con trocarObject, activar el fulcro acopla el instrumento al puerto si no
///     lo atraviesa ya: aproximacion por fuera del puerto + insercion recta por
///     su eje (ver PlanDocking), todo previsualizado y enviado como un camino.
///   - IK_calc.cs recalculaba cada Update() (60Hz) leyendo Transforms; aca se
///     recalcula solo cuando cambia un slider (OnValueChanged), que es
///     suficiente para jog y evita trabajo todos los cuadros.
///   - El robot real se mueve solo al presionar "Enviar" (igual que el
///     original) -- mover los sliders solo previsualiza en el modelo virtual
///     (mismos sliders SecJoints que ControlArticular ya escucha). A diferencia del
///     original, "Enviar" manda el camino restringido recorrido (spline), no un MoveJ
///     a la pose final, que se saldria del trocar a mitad de movimiento.
///   - El jog avanza en pasos de 2 mm y se detiene ante un cambio de rama de la IK
///     (ver MaxJointStepDeg) en vez de voltear el brazo con el instrumento insertado.
public class FulcroController : MonoBehaviour
{
    [Header("Robot")]
    public Ros2CommandSender ros2CommandSender;
    public JointPositionSubscriber jointPositionSubscriber;
    public CartesianPositionSubscriber cartesianPositionSubscriber;
    public SecCoordQueueController secCoordQueueController;

    [Header("Punto de trocar (mm, marco cartesiano del robot)")]
    [Tooltip("Posicion del punto de fulcro/trocar, medida en el mismo marco que current_cartesian_position.")]
    public float trocarX, trocarY, trocarZ;
    [Tooltip("True cuando trocarXYZ es un punto registrado (boton FIJAR TROCAR o al activar).")]
    public bool trocarRegistered;
    public Button btnFijarTrocar;

    [Header("Trocar en la escena (opcional)")]
    [Tooltip("Objeto que hace de trocar: su posicion es el punto de fulcro y su eje -up (abajo) la direccion de insercion. Con el fulcro activo queda bloqueado.")]
    public Transform trocarObject;
    [Tooltip("Acoplamiento: distancia de la punta al fulcro, por fuera del puerto, antes de insertar (mm). Debe superar la mitad del largo del cilindro.")]
    public float approachClearanceMm = 70f;
    const float RollSampleDeg = 30f;

    [Header("Herramienta")]
    [Tooltip("Longitud total desde el TCP (marco final de la cadena DH) hasta la punta del instrumento, mm.")]
    public float toolLengthMm = 300f;
    [Tooltip("Al registrar el trocar o al acoplar: cuanto del instrumento queda adentro, de la punta al trocar, mm.")]
    public float insertionDepthMm = 80f;

    [Header("Visualizacion 3D (opcional)")]
    [Tooltip("base_link del modelo: origen del marco del robot en la escena.")]
    public Transform robotBase;
    [Tooltip("Brida del modelo (tool_Link): donde nace el instrumento dibujado. El modelo y la cadena DH comparten el eje J6 pero no los offsets sobre el (el TCP DH queda ~200 mm mas alla de tool_Link), asi que la linea arranca aca para verse montada en el robot.")]
    public Transform instrumentMount;

    [Header("Sliders XYZ -- desplazamiento (mm) de la punta desde la referencia")]
    public Slider sliderX, sliderY, sliderZ;
    public float rangeXMm = 100f, rangeYMm = 100f, rangeZMm = 100f;

    [Header("Slider J6 -- rotacion de camara independiente del fulcro")]
    public Slider sliderJ6;
    [Range(10f, 180f)] public float rangeJ6Deg = 90f;
    public Button btnModoJ6;
    public Text labelModoJ6;

    [Header("Activacion / envio")]
    public Toggle modeToggle;
    public Button sendButton;
    public Slider speedSlider;
    public Text statusLabel;

    [Header("Lectura del valor de cada slider (opcional)")]
    public Text valueX, valueY, valueZ, valueJ6, valueSpeed;

    public bool IsActive => _fulcroActive;
    public float LastRcmErrorMm { get; private set; }

    bool _fulcroActive;
    bool _modoJ6Active;
    bool _updatingSliders;
    Vector3 _referenceTip;           // (x,y,z) mm capturada al activar el fulcro
    float[] _lastSolvedDeg;          // ultima solucion valida -- lo que manda "Enviar"
    float[] _baseAnglesJ6 = new float[6]; // J1-J5 congelados mientras dura el modo J6

    void Start()
    {
        if (ros2CommandSender == null) ros2CommandSender = FindObjectOfType<Ros2CommandSender>();
        if (jointPositionSubscriber == null) jointPositionSubscriber = FindObjectOfType<JointPositionSubscriber>();
        if (cartesianPositionSubscriber == null) cartesianPositionSubscriber = FindObjectOfType<CartesianPositionSubscriber>();
        if (secCoordQueueController == null) secCoordQueueController = FindObjectOfType<SecCoordQueueController>();

        if (modeToggle != null) { modeToggle.isOn = false; modeToggle.onValueChanged.AddListener(OnToggleChanged); }
        if (sliderX != null) { ConfigureSlider(sliderX, rangeXMm); sliderX.onValueChanged.AddListener(_ => RecomputePreview()); }
        if (sliderY != null) { ConfigureSlider(sliderY, rangeYMm); sliderY.onValueChanged.AddListener(_ => RecomputePreview()); }
        if (sliderZ != null) { ConfigureSlider(sliderZ, rangeZMm); sliderZ.onValueChanged.AddListener(_ => RecomputePreview()); }
        if (sliderJ6 != null)
        {
            sliderJ6.minValue = -rangeJ6Deg; sliderJ6.maxValue = rangeJ6Deg; sliderJ6.value = 0f;
            sliderJ6.interactable = false;
            sliderJ6.onValueChanged.AddListener(OnSliderJ6Changed);
        }
        if (btnModoJ6 != null) btnModoJ6.onClick.AddListener(ToggleModoJ6);
        if (btnFijarTrocar != null) btnFijarTrocar.onClick.AddListener(FijarTrocar);
        if (sendButton != null) { sendButton.onClick.AddListener(() => StartCoroutine(SendToRobot())); sendButton.interactable = false; }
        if (speedSlider != null) { speedSlider.minValue = 1f; speedSlider.maxValue = 100f; speedSlider.value = 10f; }

        SetStatus("FULCRO");
    }

    void Update()
    {
        SetValue(valueX, sliderX, "{0:+0;-0;0} mm");
        SetValue(valueY, sliderY, "{0:+0;-0;0} mm");
        SetValue(valueZ, sliderZ, "{0:+0;-0;0} mm");
        SetValue(valueJ6, sliderJ6, "{0:+0;-0;0}°");
        SetValue(valueSpeed, speedSlider, "{0:0}%");
        UpdateTrocarObject();
        UpdateVisuals();
    }

    static void SetValue(Text label, Slider slider, string format)
    {
        if (label == null || slider == null) return;
        string text = string.Format(System.Globalization.CultureInfo.InvariantCulture, format, slider.value);
        if (label.text != text) label.text = text;
    }

    static void ConfigureSlider(Slider s, float range)
    {
        s.minValue = -range; s.maxValue = range; s.wholeNumbers = false; s.value = 0f;
    }

    // ------------------------------------------------------------------ //
    //  Activacion
    // ------------------------------------------------------------------ //
    void OnToggleChanged(bool isOn)
    {
        if (isOn) StartCoroutine(ActivateFulcro());
        else Deactivate();
    }

    IEnumerator ActivateFulcro()
    {
        _fulcroActive = false;
        _modoJ6Active = false;
        if (sendButton != null) sendButton.interactable = false;
        if (sliderJ6 != null) sliderJ6.interactable = false;
        SetStatus("SINCRONIZANDO...");

        _path.Clear();
        _path.Add(ReadSliderDeg());
        string docked = null;

        if (trocarObject != null)
        {
            // El trocar de la escena manda. Si el instrumento no lo atraviesa, se
            // previsualiza el acoplamiento (aproximacion + insercion) y queda en _path
            // para que "Enviar" lo ejecute junto con el jog posterior.
            SyncTrocarFromObject();
            if (!InstrumentThroughTrocar(ReadSliderDeg()))
            {
                if (!PlanDocking(ReadSliderDeg(), out List<float[]> dockPath, out string failure))
                {
                    if (modeToggle != null) modeToggle.SetIsOnWithoutNotify(false);
                    SetStatus("TROCAR INALCANZABLE: " + failure);
                    yield break;
                }
                _path.AddRange(dockPath);
                ApplyToSliders(dockPath[dockPath.Count - 1]);
                _lastSolvedDeg = dockPath[dockPath.Count - 1];
                docked = $"ACOPLADO · {ThinPath(_path, PathResolutionDeg).Count - 1} puntos · ENVIAR";
            }
        }
        // Sin trocar registrado se fija sobre el eje actual del instrumento: asi el
        // primer calculo reproduce la pose actual y activar el fulcro no mueve nada.
        else if (!trocarRegistered) FijarTrocar();

        _lockedTrocarPos = trocarObject != null ? trocarObject.position : Vector3.zero;
        _lockedTrocarRot = trocarObject != null ? trocarObject.rotation : Quaternion.identity;

        // Referencia = punta del instrumento en la pose actual del modelo (TCP + Z *
        // longitud), no el TCP: los sliders XYZ desplazan la punta, no la brida.
        CurrentInstrument(out _, out _, out Vector3 tip);
        _referenceTip = tip;

        yield return null;

        _updatingSliders = true;
        if (sliderX != null) sliderX.value = 0f;
        if (sliderY != null) sliderY.value = 0f;
        if (sliderZ != null) sliderZ.value = 0f;
        _updatingSliders = false;
        _lastOffset = Vector3.zero;

        _fulcroActive = true;
        if (sendButton != null) sendButton.interactable = true;
        RecomputePreview();
        if (docked != null) SetStatus(docked);
    }

    void Deactivate()
    {
        _fulcroActive = false;
        _modoJ6Active = false;
        if (sendButton != null) sendButton.interactable = false;
        if (sliderJ6 != null) { sliderJ6.value = 0f; sliderJ6.interactable = false; }
        SetStatus("FULCRO");
    }

    static bool HasNonZero(float[] v)
    {
        foreach (float x in v) if (Mathf.Abs(x) > 0.01f) return true;
        return false;
    }

    /// Registra el trocar sobre el eje actual del instrumento, insertionDepthMm por
    /// detras de la punta -- igual que en un robot quirurgico se registra el RCM con
    /// el instrumento ya pasado por el puerto. No se permite con el fulcro activo:
    /// mover el pivote a mitad de un jog cambiaria la restriccion bajo el usuario.
    public void FijarTrocar()
    {
        if (_fulcroActive) { SetStatus("Desactiva el fulcro para fijar el trocar"); return; }

        CurrentInstrument(out _, out Vector3 axis, out Vector3 tip);
        float depth = Mathf.Clamp(insertionDepthMm, 1f, toolLengthMm - 1f);
        Vector3 rcm = tip - axis * depth;
        trocarX = rcm.x; trocarY = rcm.y; trocarZ = rcm.z;
        trocarRegistered = true;

        // Con trocar en escena, el punto registrado (p. ej. tocando el puerto real con el
        // robot) mueve el cilindro: el punto fisico manda sobre la escena.
        if (trocarObject != null && robotBase != null)
        {
            trocarObject.position = RobotToWorld(rcm);
            trocarObject.rotation = Quaternion.FromToRotation(Vector3.up, -RobotDirToWorld(axis));
        }
        SetStatus($"TROCAR ({rcm.x:F0}, {rcm.y:F0}, {rcm.z:F0})");
    }

    // ------------------------------------------------------------------ //
    //  Trocar en la escena: sincronizacion, bloqueo y acoplamiento
    // ------------------------------------------------------------------ //
    Vector3 _lockedTrocarPos;
    Quaternion _lockedTrocarRot;
    Vector3 _checkedTrocarPos;
    Quaternion _checkedTrocarRot;
    float[] _checkedFromDeg;
    float _nextReachCheck;
    bool _trocarReachable = true;
    MaterialPropertyBlock _trocarMpb;

    static readonly Color TrocarOk      = new Color(0.976f, 0.451f, 0.086f, 1f);
    static readonly Color TrocarBlocked = new Color(0.910f, 0.251f, 0.251f, 1f);

    /// Eje de insercion del puerto en el marco del robot: -up del cilindro.
    Vector3 PortAxis() => WorldDirToRobot(-trocarObject.up).normalized;

    void SyncTrocarFromObject()
    {
        Vector3 p = WorldToRobot(trocarObject.position);
        trocarX = p.x; trocarY = p.y; trocarZ = p.z;
        trocarRegistered = true;
    }

    void UpdateTrocarObject()
    {
        if (trocarObject == null || robotBase == null) return;

        if (_fulcroActive)
        {
            // Bloqueado: mover el pivote con el instrumento insertado no tiene sentido
            // fisico. Para moverlo: desactivar, mover y volver a acoplar.
            if (trocarObject.position != _lockedTrocarPos || trocarObject.rotation != _lockedTrocarRot)
            {
                trocarObject.SetPositionAndRotation(_lockedTrocarPos, _lockedTrocarRot);
                SetStatus("Desactiva el fulcro para mover el trocar");
            }
            SetTrocarColor(TrocarOk);
            return;
        }

        SyncTrocarFromObject();

        // Indicador de alcance: se replanifica el acoplamiento solo si cambio el trocar o
        // la pose de partida, y como mucho 4 veces por segundo (IK analitica, barata).
        float[] from = ReadSliderDeg();
        bool changed = trocarObject.position != _checkedTrocarPos || trocarObject.rotation != _checkedTrocarRot
                       || _checkedFromDeg == null || MaxAbsDelta(from, _checkedFromDeg) > 0.5f;
        if (changed && Time.unscaledTime >= _nextReachCheck)
        {
            _nextReachCheck = Time.unscaledTime + 0.25f;
            _checkedTrocarPos = trocarObject.position;
            _checkedTrocarRot = trocarObject.rotation;
            _checkedFromDeg = from;
            _trocarReachable = InstrumentThroughTrocar(from) || PlanDocking(from, out _, out _);
        }
        SetTrocarColor(_trocarReachable ? TrocarOk : TrocarBlocked);
    }

    void SetTrocarColor(Color c)
    {
        var r = trocarObject.GetComponent<Renderer>();
        if (r == null) return;
        if (_trocarMpb == null) _trocarMpb = new MaterialPropertyBlock();
        r.GetPropertyBlock(_trocarMpb);
        _trocarMpb.SetColor("_Color", c);
        r.SetPropertyBlock(_trocarMpb);
    }

    /// El instrumento ya atraviesa el trocar: la recta pasa a <1 mm del fulcro y el
    /// fulcro queda entre el TCP y la punta.
    bool InstrumentThroughTrocar(float[] deg)
    {
        InstrumentFromJoints(deg, out Vector3 tcp, out Vector3 axis, out _);
        Vector3 d = new Vector3(trocarX, trocarY, trocarZ) - tcp;
        float along = Vector3.Dot(d, axis);
        return (d - along * axis).magnitude < 1f && along > 0f && along < toolLengthMm;
    }

    /// Acoplamiento al trocar en dos fases, desde `fromDeg`:
    ///   1. Aproximacion: la punta a approachClearanceMm por fuera del puerto, alineada
    ///      con su eje. Movimiento libre (interpolacion articular) -- no hay modelo de
    ///      colisiones del entorno, por eso se previsualiza antes de enviar.
    ///   2. Insercion: la punta avanza en linea recta por el eje del puerto hasta
    ///      insertionDepthMm mas alla del fulcro, en pasos de JogStepMm y con el mismo
    ///      freno ante cambios de rama que el jog.
    /// `path` no incluye la pose de partida.
    bool PlanDocking(float[] fromDeg, out List<float[]> path, out string failure)
    {
        path = null;
        Vector3 rcm = new Vector3(trocarX, trocarY, trocarZ);
        Vector3 axis = PortAxis();
        Vector3 preTip = rcm - axis * approachClearanceMm;
        Vector3 inTip = rcm + axis * Mathf.Clamp(insertionDepthMm, 1f, toolLengthMm - 1f);

        // La rama "mas cercana" de la aproximacion no sirve sola: puede dejar el codo casi
        // estirado y la insercion lo haria pasar por la singularidad (J3=0) y saltar de
        // rama. Se prueba la insercion completa desde CADA rama candidata y se elige, de
        // las que la completan sin saltos, la de menor costo desde la pose de partida.
        // El roll del instrumento sobre su eje tampoco se hereda de la pose de partida:
        // si la herramienta apuntaba a otro lado, ese roll puede dejar fuera de limites a
        // las ramas naturales y forzar una media vuelta de J1. Para acoplar es libre, asi
        // que se prueban rolls cada RollSampleDeg y entran todas sus ramas candidatas.
        SliderLimits(out float[] sliderMin, out float[] sliderMax);
        Vector3 refX = Vector3.Cross(Mathf.Abs(axis.z) < 0.9f ? Vector3.forward : Vector3.right, axis).normalized;
        var candidates = new List<float[]>();
        string reason = null;
        for (float roll = 0f; roll < 360f; roll += RollSampleDeg)
        {
            Vector3 prefX = Quaternion.AngleAxis(roll, axis) * refX;
            candidates.AddRange(FR5AnalyticIK.SolveAll(InstrumentTargetMatrix(preTip, axis, prefX), fromDeg,
                                                       out string r, sliderMin, sliderMax));
            reason ??= r;
        }
        if (candidates.Count == 0)
        {
            failure = "aproximacion: " + reason;
            return false;
        }
        // Las de menor costo primero: la primera rama que completa la insercion gana, y
        // no hace falta simular las demas.
        candidates.Sort((a, b) => FR5AnalyticIK.SolutionCost(a, fromDeg).CompareTo(FR5AnalyticIK.SolutionCost(b, fromDeg)));

        failure = "insercion: cambio de configuracion en todas las ramas";
        foreach (float[] pre in candidates)
        {
            if (!SimulateInsertion(pre, preTip, inTip, axis, out List<float[]> insertion, out string insFailure))
            {
                failure = "insercion: " + insFailure;
                continue;
            }

            // Aproximacion: interpolacion articular, un punto cada PathResolutionDeg.
            path = new List<float[]>();
            int n = Mathf.Max(1, Mathf.CeilToInt(MaxAbsDelta(pre, fromDeg) / PathResolutionDeg));
            for (int k = 1; k <= n; k++)
            {
                float t = (float)k / n;
                var q = new float[6];
                for (int i = 0; i < 6; i++) q[i] = Mathf.Lerp(fromDeg[i], pre[i], t);
                path.Add(q);
            }
            path.AddRange(insertion);
            failure = null;
            return true;
        }
        return false;
    }

    /// Avanza la punta de preTip a inTip por el eje, en pasos de JogStepMm, partiendo de
    /// la rama `pre`. Falla si algun paso no tiene solucion o exige un giro de rama.
    bool SimulateInsertion(float[] pre, Vector3 preTip, Vector3 inTip, Vector3 axis,
                           out List<float[]> insertion, out string failure)
    {
        insertion = new List<float[]>();
        failure = null;
        float[] seed = pre;
        int steps = Mathf.Max(1, Mathf.CeilToInt((inTip - preTip).magnitude / JogStepMm));
        for (int k = 1; k <= steps; k++)
        {
            Vector3 tip = Vector3.Lerp(preTip, inTip, (float)k / steps);
            if (!SolveInstrumentPose(tip, axis, seed, out float[] sol, out failure)) return false;
            if (MaxAbsDelta(sol, seed) > MaxJointStepDeg) { failure = "cambio de configuracion"; return false; }
            insertion.Add(sol);
            seed = sol;
        }
        return true;
    }

    static float MaxAbsDelta(float[] a, float[] b)
    {
        float m = 0f;
        for (int i = 0; i < 6; i++) m = Mathf.Max(m, Mathf.Abs(a[i] - b[i]));
        return m;
    }

    /// TCP, eje de insercion (Z del TCP) y punta del instrumento en la pose que
    /// muestran ahora los sliders SecJoints, en mm del marco del robot.
    void CurrentInstrument(out Vector3 tcp, out Vector3 axis, out Vector3 tip)
    {
        InstrumentFromJoints(ReadSliderDeg(), out tcp, out axis, out tip);
    }

    void InstrumentFromJoints(float[] deg, out Vector3 tcp, out Vector3 axis, out Vector3 tip)
    {
        float[] fk = LocalForwardKinematics.CartesianFromJointsDeg(deg);
        Matrix4x4 m = FR5AnalyticIK.BuildTargetMatrix(fk[0], fk[1], fk[2], fk[3], fk[4], fk[5]);
        tcp = new Vector3(fk[0], fk[1], fk[2]);
        axis = new Vector3(m[0, 2], m[1, 2], m[2, 2]).normalized;
        tip = tcp + axis * toolLengthMm;
    }

    /// Distancia (mm) del trocar a la recta del instrumento en la pose `deg`: lo que la
    /// restriccion RCM promete mantener en ~0.
    float RcmErrorMm(float[] deg)
    {
        InstrumentFromJoints(deg, out Vector3 tcp, out Vector3 axis, out _);
        Vector3 d = new Vector3(trocarX, trocarY, trocarZ) - tcp;
        return (d - Vector3.Dot(d, axis) * axis).magnitude;
    }

    // ------------------------------------------------------------------ //
    //  Restriccion RCM -- geometria vectorial (igual que IK_calc.Update(),
    //  pero en el marco cartesiano del robot en vez de Transforms de escena).
    // ------------------------------------------------------------------ //
    /// Paso de interpolacion de la punta (mm) y giro maximo de una articulacion por
    /// paso (grados). Un paso que exige mas giro es un cambio de rama de la IK -- el
    /// brazo pasaria por otra configuracion arrastrando el instrumento por el trocar --
    /// asi que el jog se detiene ahi en vez de saltar.
    const float JogStepMm = 2f;
    const float MaxJointStepDeg = 10f;

    Vector3 _lastOffset; // ultimo desplazamiento XYZ alcanzado -- de donde parte la interpolacion

    /// Poses articulares recorridas (restringidas) desde el ultimo envio; "Enviar" manda
    /// este camino. Resolucion del envio: un punto cada PathResolutionDeg.
    readonly List<float[]> _path = new List<float[]>();
    const float PathResolutionDeg = 2f;

    void RecomputePreview()
    {
        if (_updatingSliders || !_fulcroActive || _modoJ6Active) return;

        Vector3 target = new Vector3(
            sliderX != null ? sliderX.value : 0f,
            sliderY != null ? sliderY.value : 0f,
            sliderZ != null ? sliderZ.value : 0f);
        Vector3 from = _lastOffset;
        int steps = Mathf.Max(1, Mathf.CeilToInt((target - from).magnitude / JogStepMm));

        float[] seed = ReadSliderDeg();
        float[] good = null;
        Vector3 goodOffset = from;
        string failure = null;
        for (int k = 1; k <= steps; k++)
        {
            Vector3 offset = Vector3.Lerp(from, target, (float)k / steps);
            if (!TrySolveRcm(offset, seed, out float[] sol, out failure)) break;

            int worst = 0; float worstDelta = 0f;
            for (int i = 0; i < 6; i++)
            {
                float d = Mathf.Abs(sol[i] - seed[i]);
                if (d > worstDelta) { worstDelta = d; worst = i; }
            }
            if (worstDelta > MaxJointStepDeg)
            {
                failure = $"limite de configuracion (J{worst + 1} giraria {worstDelta:F0}°)";
                break;
            }
            seed = sol; good = sol; goodOffset = offset;
            _path.Add(sol);
        }

        if (good != null)
        {
            _lastSolvedDeg = good;
            _lastOffset = goodOffset;
            ApplyToSliders(good);
            LastRcmErrorMm = RcmErrorMm(good);
        }

        if (failure != null)
        {
            // Los sliders XYZ vuelven al ultimo punto alcanzado: la UI siempre muestra
            // lo que "Enviar" va a mandar, no un objetivo al que el robot no llego.
            _updatingSliders = true;
            if (sliderX != null) sliderX.value = _lastOffset.x;
            if (sliderY != null) sliderY.value = _lastOffset.y;
            if (sliderZ != null) sliderZ.value = _lastOffset.z;
            _updatingSliders = false;
            SetStatus("FULCRO: " + failure);
            return;
        }
        SetStatus($"FULCRO ACTIVO · err RCM {LastRcmErrorMm:F2} mm");
    }

    /// IK de la pose que lleva la punta a _referenceTip + offset con el eje del
    /// instrumento pasando por el trocar. `seedDeg` es la semilla de la IK y de el sale
    /// tambien el roll preferido.
    bool TrySolveRcm(Vector3 offset, float[] seedDeg, out float[] jointsDeg, out string failure)
    {
        jointsDeg = null;
        failure = null;
        Vector3 tipTarget = _referenceTip + offset;
        Vector3 rcm = new Vector3(trocarX, trocarY, trocarZ);

        Vector3 toTip = tipTarget - rcm;
        float pivotOffset = toTip.magnitude;
        if (pivotOffset < 1f) // <1mm -- la punta coincide con el trocar, direccion indefinida
        {
            failure = "punta demasiado cerca del trocar";
            return false;
        }
        return SolveInstrumentPose(tipTarget, toTip / pivotOffset, seedDeg, out jointsDeg, out failure);
    }

    /// IK de la pose con la punta del instrumento en `tipTarget` (mm) y el eje de
    /// insercion `zAxis` (unitario, de la brida hacia la punta).
    bool SolveInstrumentPose(Vector3 tipTarget, Vector3 zAxis, float[] seedDeg, out float[] jointsDeg, out string failure)
    {
        jointsDeg = null;
        failure = null;
        SliderLimits(out float[] sliderMin, out float[] sliderMax);
        var result = FR5AnalyticIK.Solve(InstrumentTargetMatrix(tipTarget, zAxis, seedDeg), seedDeg, sliderMin, sliderMax);
        if (!result.Success)
        {
            failure = result.FailureReason;
            return false;
        }
        jointsDeg = result.JointsDeg;
        return true;
    }

    /// Rango de los sliders SecJoints como limite extra de la IK: la solucion se escribe
    /// ahi y Slider.value la recortaria en silencio si cayera fuera.
    void SliderLimits(out float[] sliderMin, out float[] sliderMax)
    {
        var s = Sliders();
        sliderMin = new float[6]; sliderMax = new float[6];
        for (int i = 0; i < 6; i++)
        {
            sliderMin[i] = s[i] != null ? s[i].minValue : float.MinValue;
            sliderMax[i] = s[i] != null ? s[i].maxValue : float.MaxValue;
        }
    }

    /// Pose objetivo del TCP (matriz en metros) para la punta en `tipTarget` con el eje
    /// `zAxis`; el roll sale de `seedDeg`.
    Matrix4x4 InstrumentTargetMatrix(Vector3 tipTarget, Vector3 zAxis, float[] seedDeg)
    {
        // "Roll" preferido: mantener la orientacion actual del TCP, ortogonalizada
        // contra el nuevo eje de insercion (Gram-Schmidt) -- el modo J6 es quien
        // realmente controla el roll de camara; este solo evita un salto brusco
        // de orientacion cada vez que cambia el eje.
        float[] curFk = LocalForwardKinematics.CartesianFromJointsDeg(seedDeg);
        Matrix4x4 curMatrix = FR5AnalyticIK.BuildTargetMatrix(curFk[0], curFk[1], curFk[2], curFk[3], curFk[4], curFk[5]);
        return InstrumentTargetMatrix(tipTarget, zAxis, new Vector3(curMatrix[0, 0], curMatrix[1, 0], curMatrix[2, 0]));
    }

    /// Igual, con el eje X del TCP preferido explicito (se ortogonaliza contra zAxis).
    Matrix4x4 InstrumentTargetMatrix(Vector3 tipTarget, Vector3 zAxis, Vector3 preferredX)
    {
        Vector3 tcpTarget = tipTarget - zAxis * toolLengthMm;

        Vector3 xAxis = preferredX - Vector3.Project(preferredX, zAxis);
        if (xAxis.sqrMagnitude < 1e-6f) xAxis = Vector3.Cross(Vector3.up, zAxis);
        xAxis.Normalize();
        Vector3 yAxis = Vector3.Cross(zAxis, xAxis).normalized;

        var target = Matrix4x4.identity;
        target.SetColumn(0, new Vector4(xAxis.x, xAxis.y, xAxis.z, 0));
        target.SetColumn(1, new Vector4(yAxis.x, yAxis.y, yAxis.z, 0));
        target.SetColumn(2, new Vector4(zAxis.x, zAxis.y, zAxis.z, 0));
        target.SetColumn(3, new Vector4(tcpTarget.x / 1000f, tcpTarget.y / 1000f, tcpTarget.z / 1000f, 1));

        // Ida y vuelta por rx/ry/rz: misma matriz que construye la ruta cartesiana del
        // resto de la app (BuildTargetMatrix), asi la verificacion por FK de la IK compara
        // contra exactamente la misma convencion.
        FR5AnalyticIK.DecodeRxRyRz(target, out float rx, out float ry, out float rz);
        return FR5AnalyticIK.BuildTargetMatrix(tcpTarget.x, tcpTarget.y, tcpTarget.z, rx, ry, rz);
    }

    // ------------------------------------------------------------------ //
    //  Modo J6 -- rotacion de camara independiente, congela J1-J5
    // ------------------------------------------------------------------ //
    public void ToggleModoJ6()
    {
        if (!_fulcroActive) return;
        _modoJ6Active = !_modoJ6Active;

        if (_modoJ6Active)
        {
            // Base = la previsualizacion (sliders), no el estado del robot: el jog del
            // fulcro todavia no se envio, y partir del robot devolvia el brazo de golpe
            // a la ultima pose enviada, fuera del trocar.
            _baseAnglesJ6 = ReadSliderDeg();

            _updatingSliders = true;
            if (sliderJ6 != null) { sliderJ6.value = 0f; sliderJ6.interactable = true; }
            _updatingSliders = false;

            if (labelModoJ6 != null) labelModoJ6.text = "MODO J6 ON";
        }
        else
        {
            if (sliderJ6 != null) { sliderJ6.value = 0f; sliderJ6.interactable = false; }
            if (labelModoJ6 != null) labelModoJ6.text = "MODO J6";
            RecomputePreview();
        }
    }

    void OnSliderJ6Changed(float offsetDeg)
    {
        if (_updatingSliders || !_modoJ6Active) return;

        float[] deg = (float[])_baseAnglesJ6.Clone();
        deg[5] += offsetDeg;
        _lastSolvedDeg = deg;
        _path.Add(deg); // girar solo J6 no mueve el eje del instrumento: cualquier tramo es seguro
        ApplyToSliders(deg);
    }

    // ------------------------------------------------------------------ //
    //  Sliders SecJoints compartidos -- mismo mecanismo de previsualizacion
    //  que SecCartInputController: estos son los sliders que ControlArticular
    //  ya escucha, asi que escribirlos actualiza el modelo 3D sin codigo propio.
    // ------------------------------------------------------------------ //
    float[] ReadSliderDeg()
    {
        var s = Sliders();
        float[] deg = new float[6];
        for (int i = 0; i < 6; i++) deg[i] = s[i] != null ? s[i].value : 0f;
        return deg;
    }

    void ApplyToSliders(float[] deg)
    {
        var s = Sliders();
        for (int i = 0; i < 6; i++)
            if (s[i] != null) s[i].value = deg[i];
    }

    Slider[] Sliders() => secCoordQueueController == null ? new Slider[6] : new[]
    {
        secCoordQueueController.j1Slider, secCoordQueueController.j2Slider, secCoordQueueController.j3Slider,
        secCoordQueueController.j4Slider, secCoordQueueController.j5Slider, secCoordQueueController.j6Slider,
    };

    // ------------------------------------------------------------------ //
    //  Enviar -- manda la ultima solucion previsualizada al robot real.
    //  Misma secuencia de comandos que ControlArticular.SendSplineCommandsWithDelay
    //  y FulcroCommandSender.SendFulcroCommand del proyecto de referencia.
    // ------------------------------------------------------------------ //
    /// Manda el CAMINO restringido recorrido desde el ultimo envio (no solo la pose
    /// final): un MoveJ directo interpola en espacio articular y a mitad de camino el
    /// instrumento se sale del trocar. Puntos cada PathResolutionDeg, encolados como
    /// spline en lotes de 5 -- mismo mecanismo que SecTrajController.
    IEnumerator SendToRobot()
    {
        if (_lastSolvedDeg == null || ros2CommandSender == null) yield break;

        List<float[]> points = ThinPath(_path, PathResolutionDeg);
        if (points.Count < 2) { SetStatus("FULCRO: nada que enviar"); yield break; }
        points.RemoveAt(0); // la pose de partida es donde ya esta el robot

        float speed = speedSlider != null ? speedSlider.value : 10f;
        var inv = System.Globalization.CultureInfo.InvariantCulture;
        if (sendButton != null) sendButton.interactable = false;

        ros2CommandSender.SendCommand("DragTeachSwitch(0)");  yield return new WaitForSeconds(0.05f);
        ros2CommandSender.SendCommand("StopMotion()");        yield return new WaitForSeconds(0.05f);
        ros2CommandSender.SendCommand("ResetAllError()");     yield return new WaitForSeconds(0.05f);
        ros2CommandSender.SendCommand("StartJOG(0,6,0,100)"); yield return new WaitForSeconds(0.5f);
        ros2CommandSender.SendCommand("StartJOG(0,6,1,100)"); yield return new WaitForSeconds(0.5f);

        ros2CommandSender.SendCommand("SplineStart()"); yield return new WaitForSeconds(0.05f);
        const int batchSize = 5;
        for (int start = 0; start < points.Count; start += batchSize)
        {
            int end = Mathf.Min(start + batchSize, points.Count);
            for (int i = start; i < end; i++)
            {
                float[] q = points[i];
                ros2CommandSender.SendCommand(string.Format(inv, "JNTPoint({0},{1:F4},{2:F4},{3:F4},{4:F4},{5:F4},{6:F4})",
                    i - start + 1, q[0], q[1], q[2], q[3], q[4], q[5]));
                yield return new WaitForSeconds(0.05f);
            }
            for (int i = start; i < end; i++)
            {
                ros2CommandSender.SendCommand(string.Format(inv, "SplinePTP(JNT{0},{1:F0})", i - start + 1, speed));
                yield return new WaitForSeconds(0.05f);
            }
        }
        ros2CommandSender.SendCommand("SplineEnd()");

        // Lo enviado pasa a ser el nuevo punto de partida del camino.
        _path.Clear();
        _path.Add((float[])_lastSolvedDeg.Clone());
        if (sendButton != null) sendButton.interactable = _fulcroActive;
        Debug.Log($"[FulcroController] Enviado camino RCM: {points.Count} puntos, vel {speed:F0}%");
        SetStatus($"ENVIADO · {points.Count} puntos");
    }

    /// Conserva un punto cada vez que alguna articulacion se movio >= resolutionDeg
    /// desde el ultimo conservado; el primero y el ultimo siempre quedan.
    static List<float[]> ThinPath(List<float[]> path, float resolutionDeg)
    {
        var kept = new List<float[]>();
        if (path.Count == 0) return kept;
        kept.Add(path[0]);
        for (int k = 1; k < path.Count; k++)
        {
            float[] last = kept[kept.Count - 1];
            float maxDelta = 0f;
            for (int i = 0; i < 6; i++) maxDelta = Mathf.Max(maxDelta, Mathf.Abs(path[k][i] - last[i]));
            if (maxDelta >= resolutionDeg || k == path.Count - 1) kept.Add(path[k]);
        }
        return kept;
    }

    void SetStatus(string text)
    {
        if (statusLabel != null) statusLabel.text = text;
    }

    // ------------------------------------------------------------------ //
    //  Visualizacion: instrumento (TCP -> punta), punta y trocar en el visor.
    //  Se arma en runtime; solo aparece una vez registrado el trocar.
    // ------------------------------------------------------------------ //
    static readonly Color VisAccent     = new Color(0.976f, 0.451f, 0.086f, 1f);
    static readonly Color VisInstrument = new Color(0.85f, 0.85f, 0.85f, 1f);

    LineRenderer _instrumentLine;
    Transform _trocarMarker, _tipMarker;

    /// mm del marco del robot -> mundo. X robot = forward de base_link, Y = -right,
    /// Z = up (pose home: x=-497 mm cae en +0.497 m del X de Unity con la base a 270°).
    Vector3 RobotToWorld(Vector3 mm)
    {
        return robotBase.position + RobotDirToWorld(mm) / 1000f;
    }

    Vector3 RobotDirToWorld(Vector3 v)
    {
        return robotBase.forward * v.x - robotBase.right * v.y + robotBase.up * v.z;
    }

    Vector3 WorldToRobot(Vector3 world)
    {
        return WorldDirToRobot(world - robotBase.position) * 1000f;
    }

    Vector3 WorldDirToRobot(Vector3 v)
    {
        return new Vector3(Vector3.Dot(v, robotBase.forward), -Vector3.Dot(v, robotBase.right), Vector3.Dot(v, robotBase.up));
    }

    void UpdateVisuals()
    {
        if (robotBase == null) return;
        bool show = trocarRegistered;
        if (_instrumentLine == null)
        {
            if (!show) return;
            BuildVisuals();
        }
        _instrumentLine.gameObject.SetActive(show);
        _trocarMarker.gameObject.SetActive(show && trocarObject == null); // el cilindro ya marca el fulcro
        _tipMarker.gameObject.SetActive(show);
        if (!show) return;

        CurrentInstrument(out Vector3 tcp, out _, out Vector3 tip);
        _instrumentLine.SetPosition(0, instrumentMount != null ? instrumentMount.position : RobotToWorld(tcp));
        _instrumentLine.SetPosition(1, RobotToWorld(tip));
        _tipMarker.position = RobotToWorld(tip);
        _trocarMarker.position = RobotToWorld(new Vector3(trocarX, trocarY, trocarZ));
    }

    void BuildVisuals()
    {
        // Sprites/Default: color plano sin luz, y esta en "Always Included Shaders" por
        // defecto -- un Shader.Find de otro shader podria no llegar al build.
        var unlit = Shader.Find("Sprites/Default");
        var root = new GameObject("FulcroVisuals").transform;

        var lineGo = new GameObject("Instrumento");
        lineGo.transform.SetParent(root, false);
        _instrumentLine = lineGo.AddComponent<LineRenderer>();
        _instrumentLine.positionCount = 2;
        _instrumentLine.useWorldSpace = true;
        _instrumentLine.widthMultiplier = 0.006f;
        _instrumentLine.numCapVertices = 4;
        _instrumentLine.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        _instrumentLine.receiveShadows = false;
        _instrumentLine.sharedMaterial = new Material(unlit) { color = VisInstrument };

        _trocarMarker = MakeMarker(root, "Trocar", 0.03f, new Material(unlit) { color = VisAccent });
        _tipMarker    = MakeMarker(root, "Punta", 0.012f, new Material(unlit) { color = VisAccent });
    }

    static Transform MakeMarker(Transform parent, string name, float diameter, Material mat)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        go.name = name;
        Destroy(go.GetComponent<Collider>());
        go.transform.SetParent(parent, false);
        go.transform.localScale = Vector3.one * diameter;
        var r = go.GetComponent<MeshRenderer>();
        r.sharedMaterial = mat;
        r.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        r.receiveShadows = false;
        return go.transform;
    }
}
