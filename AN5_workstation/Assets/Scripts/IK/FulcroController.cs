using System.Collections;
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
///     cuantificada en su momento por P9KinematicConsistency). trocarXYZ es
///     entonces una posicion medida/registrada en ese marco -- igual que se
///     registra un RCM real en un robot quirurgico -- no un gizmo de escena.
///   - IK_calc.cs recalculaba cada Update() (60Hz) leyendo Transforms; aca se
///     recalcula solo cuando cambia un slider (OnValueChanged), que es
///     suficiente para jog y evita trabajo todos los cuadros.
///   - El robot real se mueve solo al presionar "Enviar" (igual que el
///     original) -- mover los sliders solo previsualiza en el modelo virtual
///     (mismos sliders SecJoints que ControlArticular ya escucha).
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

    [Header("Herramienta")]
    [Tooltip("Longitud total desde la brida (muneca/J6) hasta la punta de la herramienta, mm.")]
    public float toolLengthMm = 300f;

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

        // Referencia = posicion cartesiana actual (del robot real si hay dato,
        // si no la del modelo virtual) -- igual que IKController.OnEnable() en el
        // proyecto de referencia, para que activar el fulcro no mueva nada de golpe.
        float[] cart = cartesianPositionSubscriber != null ? cartesianPositionSubscriber.GetLastKnownCartesianPositions() : null;
        if (cart != null && cart.Length == 6 && HasNonZero(cart))
        {
            _referenceTip = new Vector3(cart[0], cart[1], cart[2]);
        }
        else
        {
            float[] curDeg = ReadSliderDeg();
            float[] fk = LocalForwardKinematics.CartesianFromJointsDeg(curDeg);
            _referenceTip = new Vector3(fk[0], fk[1], fk[2]);
        }

        yield return null;

        _updatingSliders = true;
        if (sliderX != null) sliderX.value = 0f;
        if (sliderY != null) sliderY.value = 0f;
        if (sliderZ != null) sliderZ.value = 0f;
        _updatingSliders = false;

        _fulcroActive = true;
        if (sendButton != null) sendButton.interactable = true;
        RecomputePreview();
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

    // ------------------------------------------------------------------ //
    //  Restriccion RCM -- geometria vectorial (igual que IK_calc.Update(),
    //  pero en el marco cartesiano del robot en vez de Transforms de escena).
    // ------------------------------------------------------------------ //
    void RecomputePreview()
    {
        if (_updatingSliders || !_fulcroActive || _modoJ6Active) return;

        Vector3 offset = new Vector3(
            sliderX != null ? sliderX.value : 0f,
            sliderY != null ? sliderY.value : 0f,
            sliderZ != null ? sliderZ.value : 0f);
        Vector3 tipTarget = _referenceTip + offset;
        Vector3 rcm = new Vector3(trocarX, trocarY, trocarZ);

        Vector3 toTip = tipTarget - rcm;
        float pivotOffset = toTip.magnitude;
        if (pivotOffset < 1f) // <1mm -- la punta coincide con el trocar, direccion indefinida
        {
            SetStatus("FULCRO: punta demasiado cerca del trocar");
            return;
        }
        Vector3 zAxis = toTip / pivotOffset;
        float tcpDistance = toolLengthMm - pivotOffset;
        Vector3 tcpTarget = rcm - zAxis * tcpDistance;

        float[] currentDeg = ReadSliderDeg();

        // "Roll" preferido: mantener la orientacion actual del TCP, ortogonalizada
        // contra el nuevo eje de insercion (Gram-Schmidt) -- el modo J6 es quien
        // realmente controla el roll de camara; este solo evita un salto brusco
        // de orientacion cada vez que cambia el eje.
        float[] curFk = LocalForwardKinematics.CartesianFromJointsDeg(currentDeg);
        Matrix4x4 curMatrix = FR5AnalyticIK.BuildTargetMatrix(curFk[0], curFk[1], curFk[2], curFk[3], curFk[4], curFk[5]);
        Vector3 preferredX = new Vector3(curMatrix[0, 0], curMatrix[1, 0], curMatrix[2, 0]);

        Vector3 xAxis = preferredX - Vector3.Project(preferredX, zAxis);
        if (xAxis.sqrMagnitude < 1e-6f) xAxis = Vector3.Cross(Vector3.up, zAxis);
        xAxis.Normalize();
        Vector3 yAxis = Vector3.Cross(zAxis, xAxis).normalized;

        var target = Matrix4x4.identity;
        target.SetColumn(0, new Vector4(xAxis.x, xAxis.y, xAxis.z, 0));
        target.SetColumn(1, new Vector4(yAxis.x, yAxis.y, yAxis.z, 0));
        target.SetColumn(2, new Vector4(zAxis.x, zAxis.y, zAxis.z, 0));
        target.SetColumn(3, new Vector4(tcpTarget.x / 1000f, tcpTarget.y / 1000f, tcpTarget.z / 1000f, 1));

        FR5AnalyticIK.DecodeRxRyRz(target, out float rx, out float ry, out float rz);
        // Rango de los sliders SecJoints como limite extra: la solucion se escribe ahi
        // y Slider.value la recortaria en silencio si cayera fuera.
        var s = Sliders();
        float[] sliderMin = new float[6], sliderMax = new float[6];
        for (int i = 0; i < 6; i++)
        {
            sliderMin[i] = s[i] != null ? s[i].minValue : float.MinValue;
            sliderMax[i] = s[i] != null ? s[i].maxValue : float.MaxValue;
        }
        var result = FR5AnalyticIK.Solve(tcpTarget.x, tcpTarget.y, tcpTarget.z, rx, ry, rz, currentDeg, sliderMin, sliderMax);
        if (!result.Success)
        {
            SetStatus("FULCRO: " + result.FailureReason);
            return;
        }

        _lastSolvedDeg = result.JointsDeg;
        ApplyToSliders(result.JointsDeg);
        SetStatus("FULCRO ACTIVO");
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
            float[] real = jointPositionSubscriber != null ? jointPositionSubscriber.GetLastKnownPositions() : null;
            _baseAnglesJ6 = (real != null && real.Length == 6 && HasNonZero(real)) ? (float[])real.Clone() : ReadSliderDeg();

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
    IEnumerator SendToRobot()
    {
        if (_lastSolvedDeg == null || ros2CommandSender == null) yield break;

        float speed = speedSlider != null ? speedSlider.value : 10f;
        float[] deg = _lastSolvedDeg;

        ros2CommandSender.SendCommand("DragTeachSwitch(0)");  yield return new WaitForSeconds(0.05f);
        ros2CommandSender.SendCommand("StopMotion()");        yield return new WaitForSeconds(0.05f);
        ros2CommandSender.SendCommand("ResetAllError()");     yield return new WaitForSeconds(0.05f);
        ros2CommandSender.SendCommand("StartJOG(0,6,0,100)"); yield return new WaitForSeconds(0.5f);
        ros2CommandSender.SendCommand("StartJOG(0,6,1,100)"); yield return new WaitForSeconds(0.5f);

        string jntCommand = $"JNTPoint(1,{deg[0]:F4},{deg[1]:F4},{deg[2]:F4},{deg[3]:F4},{deg[4]:F4},{deg[5]:F4})";
        ros2CommandSender.SendCommand(jntCommand); yield return new WaitForSeconds(0.05f);
        ros2CommandSender.SendCommand($"MoveJ(JNT1,{speed:F0})");

        Debug.Log($"[FulcroController] Enviado: {jntCommand}");
    }

    void SetStatus(string text)
    {
        if (statusLabel != null) statusLabel.text = text;
    }
}
