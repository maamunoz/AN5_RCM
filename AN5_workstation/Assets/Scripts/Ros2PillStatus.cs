using UnityEngine;
using UnityEngine.UI;
using RosSharp.RosBridgeClient;

/// Drives Pill_ROS2 color and label based on RosConnector.IsOnline.
/// Attach directly to the Pill_ROS2 GameObject.
public class Ros2PillStatus : MonoBehaviour
{
    [Header("ROS (auto-resolved if blank)")]
    public RosConnector rosConnector;

    [Header("Pill children (auto-resolved)")]
    public Image   pillBackground;
    public Outline pillOutline;
    public Image   dot;
    public Text    label;

    static readonly Color GreenSolid  = new Color(0.00f, 0.83f, 0.50f, 1.00f);
    static readonly Color GreenFaint  = new Color(0.00f, 0.83f, 0.50f, 0.15f);
    static readonly Color GreenBorder = new Color(0.00f, 0.83f, 0.50f, 0.40f);

    // Advertencia/desconectado en amarillo: el naranja es el acento de la interfaz.
    static readonly Color WarnSolid  = new Color(0.961f, 0.773f, 0.094f, 1.00f);
    static readonly Color WarnFaint  = new Color(0.961f, 0.773f, 0.094f, 0.15f);
    static readonly Color WarnBorder = new Color(0.961f, 0.773f, 0.094f, 0.40f);

    bool _lastState    = false;
    bool _initialized  = false;

    void Start()
    {
        if (rosConnector   == null) rosConnector   = FindObjectOfType<RosConnector>();
        if (pillBackground == null) pillBackground = GetComponent<Image>();
        if (pillOutline    == null) pillOutline    = GetComponent<Outline>();
        if (dot            == null) dot            = transform.Find("Dot")?.GetComponent<Image>();
        if (label          == null) label          = transform.Find("Label")?.GetComponent<Text>();

        Apply(false);
    }

    void Update()
    {
        if (rosConnector == null) return;
        bool online = rosConnector.IsOnline;
        if (online == _lastState && _initialized) return;
        _lastState   = online;
        _initialized = true;
        Apply(online);
    }

    void Apply(bool online)
    {
        if (pillBackground != null) pillBackground.color    = online ? GreenFaint  : WarnFaint;
        if (pillOutline    != null) pillOutline.effectColor = online ? GreenBorder : WarnBorder;
        if (dot            != null) dot.color               = online ? GreenSolid  : WarnSolid;
        if (label          != null) label.text              = online ? "ROS2 ACTIVO" : "ROS2 INACTIVO";
    }
}
