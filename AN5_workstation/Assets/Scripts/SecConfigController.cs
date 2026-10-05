using UnityEngine;
using UnityEngine.UI;
using RosSharp.RosBridgeClient;

/// Attached to SecConfig in PersistentLayer/LeftPanel/ScrollView/Content.
/// Mirrors the active RosConnector's endpoint into the IP/Puerto fields and
/// lets Btn_Reconnect push an edited value back, forcing a reconnect.
public class SecConfigController : MonoBehaviour
{
    InputField _inputIp;
    InputField _inputPort;
    Button _btnReconnect;

    void Start()
    {
        _inputIp       = transform.Find("Body/Row_IP ROS2/Input_IP ROS 2")?.GetComponent<InputField>();
        _inputPort     = transform.Find("Body/Row_Puerto/Input_Puerto ROSBridge")?.GetComponent<InputField>();
        _btnReconnect  = transform.Find("Body/Btn_Reconnect")?.GetComponent<Button>();

        if (_btnReconnect != null)
            _btnReconnect.onClick.AddListener(ApplyFromFields);

        RefreshFromRosConnector();
    }

    /// Repopulates the IP/Puerto fields from the current RosConnector state.
    /// Called on Start, and by ModeToggleController after Eje. Real/Simulacion
    /// changes the endpoint, so this panel never shows a stale value.
    public void RefreshFromRosConnector()
    {
        var rosConnector = FindObjectOfType<RosConnector>();
        if (rosConnector == null) return;

        if (TryParseUrl(rosConnector.RosBridgeServerUrl, out string host, out string port))
        {
            if (_inputIp   != null) _inputIp.text   = host;
            if (_inputPort != null) _inputPort.text = port;
        }
    }

    // Btn_Reconnect's onClick target: reads the IP/Puerto fields and pushes
    // them to the RosConnector, forcing an immediate reconnect.
    void ApplyFromFields()
    {
        var rosConnector = FindObjectOfType<RosConnector>();
        if (rosConnector == null) return;

        // En Simulacion no hay ROS (todo corre en LocalRobotSimulator): reconectar
        // aca mezclaria estado del robot local con el de ROS.
        if (RobotMode.IsSimulation)
        {
            Debug.LogWarning("[SecConfigController] En modo Simulacion no hay conexion ROS; cambia a Eje. Real para conectar.");
            return;
        }

        string host = _inputIp   != null ? _inputIp.text.Trim()   : "";
        string port = _inputPort != null ? _inputPort.text.Trim() : "";
        if (string.IsNullOrEmpty(host) || string.IsNullOrEmpty(port))
        {
            Debug.LogWarning("[SecConfigController] IP/Puerto vacío, no se aplica el cambio.");
            return;
        }

        string newUrl = $"ws://{host}:{port}";
        if (rosConnector.RosBridgeServerUrl == newUrl)
            return;

        Debug.Log($"[SecConfigController] Aplicando nueva URL de RosBridge: {newUrl}");
        rosConnector.RosBridgeServerUrl = newUrl;
        rosConnector.ReconnectNow();
    }

    // Splits "ws://host:port" (or "wss://host:port") into host/port. Falls
    // back to an empty port if the URL has no ':' after the scheme.
    static bool TryParseUrl(string url, out string host, out string port)
    {
        host = port = "";
        if (string.IsNullOrEmpty(url))
            return false;

        int schemeEnd = url.IndexOf("://");
        string rest = schemeEnd >= 0 ? url.Substring(schemeEnd + 3) : url;

        int colonIndex = rest.LastIndexOf(':');
        if (colonIndex < 0)
        {
            host = rest;
            return true;
        }

        host = rest.Substring(0, colonIndex);
        port = rest.Substring(colonIndex + 1);
        return true;
    }
}
