using System.Collections;
using UnityEngine;
using UnityEngine.UI;
using RosSharp.RosBridgeClient;

/// Attached to ModeToggle in PersistentLayer/Header.
/// Eje. Real: RosConnector conectado al controlador fisico (como siempre).
/// Simulacion: sin ROS -- RosConnector desconectado y todo el robot corre local
/// en LocalRobotSimulator (comandos, interpolacion, estado e IK/FK). Ver RobotMode.
public class ModeToggleController : MonoBehaviour
{
    const string RosUrlReal = "ws://192.168.58.3:9090";

    static readonly Color ActiveBg          = new Color(0.000f, 0.831f, 0.667f, 1.000f);
    static readonly Color ActiveHighlighted = new Color(0.000f, 0.900f, 0.720f, 1.000f);
    static readonly Color ActivePressed     = new Color(0.000f, 0.650f, 0.520f, 1.000f);
    static readonly Color ActiveText        = new Color(0.000f, 0.000f, 0.000f, 1.000f);

    static readonly Color IdleBg            = new Color(0.000f, 0.000f, 0.000f, 0.000f);
    static readonly Color IdleHighlighted   = new Color(0.000f, 0.831f, 0.667f, 0.300f);
    static readonly Color IdlePressed       = new Color(0.000f, 0.831f, 0.667f, 0.500f);
    static readonly Color IdleText          = new Color(0.353f, 0.388f, 0.439f, 1.000f);

    Button _btnReal, _btnSim;
    Image  _bgReal,  _bgSim;
    Text   _txtReal, _txtSim;

    bool _realModeActive;

    IEnumerator Start()
    {
        _btnReal = transform.Find("BtnReal")?.GetComponent<Button>();
        _btnSim  = transform.Find("BtnSim")?.GetComponent<Button>();
        _bgReal  = transform.Find("BtnReal")?.GetComponent<Image>();
        _bgSim   = transform.Find("BtnSim")?.GetComponent<Image>();
        _txtReal = transform.Find("BtnReal/T")?.GetComponent<Text>();
        _txtSim  = transform.Find("BtnSim/T")?.GetComponent<Text>();

        if (_btnReal) _btnReal.onClick.AddListener(SetRealMode);
        if (_btnSim)  _btnSim.onClick.AddListener(SetSimMode);

        // Wait one frame so the Canvas finishes its first layout pass
        // before we apply colors — prevents Button.CrossFadeColor from
        // overwriting our state on the very first frame.
        yield return null;

        // Simulacion por defecto al arrancar (RobotMode ya arranca asi y
        // RosConnector.Awake ya no se conecto).
        _realModeActive = false;
        SetHighlight(_realModeActive);
        ApplyMode(_realModeActive);
    }

    public void SetRealMode()
    {
        _realModeActive = true;
        SetHighlight(_realModeActive);
        ApplyMode(_realModeActive);
    }

    public void SetSimMode()
    {
        _realModeActive = false;
        SetHighlight(_realModeActive);
        ApplyMode(_realModeActive);
    }

    void ApplyMode(bool realIsActive)
    {
        bool wasSimulation = RobotMode.IsSimulation;
        RobotMode.Set(!realIsActive);

        var rosConnector = FindObjectOfType<RosConnector>();
        if (rosConnector == null) return;

        if (!realIsActive)
        {
            if (!rosConnector.IsSuspended)
                rosConnector.Disconnect();
            Debug.Log("[ModeToggleController] Simulacion: sin ROS, robot simulado local.");
            return;
        }

        // Ejec. Real: reconectar si veniamos de Simulacion (conexion suspendida)
        // o si la URL no era la del controlador fisico.
        if (wasSimulation || rosConnector.IsSuspended || rosConnector.RosBridgeServerUrl != RosUrlReal)
        {
            Debug.Log($"[ModeToggleController] Eje. Real: conectando a {RosUrlReal}");
            rosConnector.RosBridgeServerUrl = RosUrlReal;
            rosConnector.ReconnectNow();
        }

        // Keep the Configuracion panel's IP/Puerto fields in sync so they
        // don't keep showing whatever was there before this mode switch.
        FindObjectOfType<SecConfigController>()?.RefreshFromRosConnector();
    }

    void SetHighlight(bool realIsActive)
    {
        ApplyState(_bgReal, _btnReal, _txtReal, realIsActive);
        ApplyState(_bgSim,  _btnSim,  _txtSim,  !realIsActive);
    }

    void ApplyState(Image bg, Button btn, Text txt, bool active)
    {
        var bgColor = active ? ActiveBg : IdleBg;
        if (bg != null) bg.color = bgColor;
        if (btn != null)
        {
            var cb = btn.colors;
            cb.normalColor      = bgColor;
            cb.highlightedColor = active ? ActiveHighlighted : IdleHighlighted;
            cb.pressedColor     = active ? ActivePressed     : IdlePressed;
            cb.selectedColor    = bgColor;
            cb.fadeDuration     = 0.05f;
            btn.colors = cb;
            // Force the button to immediately snap to Normal state
            btn.targetGraphic.CrossFadeColor(bgColor, 0f, true, true);
        }
        if (txt != null) txt.color = active ? ActiveText : IdleText;
    }
}
