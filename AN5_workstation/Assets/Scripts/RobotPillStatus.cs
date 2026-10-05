using UnityEngine;
using UnityEngine.UI;

/// Drives Pill_ROBOT color and label by watching JointPositionSubscriber message activity.
/// If no joint state arrives within `timeoutSeconds`, the robot is considered disconnected.
/// Attach directly to the Pill_ROBOT GameObject.
public class RobotPillStatus : MonoBehaviour
{
    [Header("Data source (auto-resolved if blank)")]
    public JointPositionSubscriber jointSubscriber;

    [Header("Timeout")]
    [Tooltip("Seconds without a joint-state message before the robot is considered offline.")]
    public float timeoutSeconds = 2f;

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

    float _lastMessageTime = -999f;
    bool  _lastState       = false;

    void Start()
    {
        if (jointSubscriber   == null) jointSubscriber   = FindObjectOfType<JointPositionSubscriber>();
        if (pillBackground    == null) pillBackground    = GetComponent<Image>();
        if (pillOutline       == null) pillOutline       = GetComponent<Outline>();
        if (dot               == null) dot               = transform.Find("Dot")?.GetComponent<Image>();
        if (label             == null) label             = transform.Find("Label")?.GetComponent<Text>();

        if (jointSubscriber != null)
            jointSubscriber.OnJointPositionsUpdated += OnMessage;

        Apply(false);
    }

    void OnDestroy()
    {
        if (jointSubscriber != null)
            jointSubscriber.OnJointPositionsUpdated -= OnMessage;
    }

    void OnMessage(float[] _) => _lastMessageTime = Time.time;

    void Update()
    {
        bool connected = (Time.time - _lastMessageTime) <= timeoutSeconds;
        if (connected == _lastState) return;
        _lastState = connected;
        Apply(connected);
    }

    void Apply(bool connected)
    {
        if (pillBackground != null) pillBackground.color    = connected ? GreenFaint  : WarnFaint;
        if (pillOutline    != null) pillOutline.effectColor = connected ? GreenBorder : WarnBorder;
        if (dot            != null) dot.color               = connected ? GreenSolid  : WarnSolid;
        if (label          != null) label.text              = connected ? "ROBOT CONECTADO" : "ROBOT INACTIVO";
    }
}
