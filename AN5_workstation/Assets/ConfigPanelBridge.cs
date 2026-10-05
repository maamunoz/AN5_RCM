using UnityEngine;
using UnityEngine.UI;
using RosSharp.RosBridgeClient;

public class ConfigPanelBridge : MonoBehaviour
{
    [Header("ROS Connector")]
    public RosConnector rosConnector;

    [Header("Config UI Fields")]
    public InputField ipInputField;
    public InputField portInputField;

    [Header("Status UI")]
    public Text connectionStatusText;
    public Text elapsedTimeText;

    private static readonly Color ColorConnected    = Color.green;
    private static readonly Color ColorDisconnected = new Color(0.961f, 0.773f, 0.094f); // amarillo: el naranja es el acento

    private bool _updating = false;

    private void Start()
    {
        PullFromConnector();
        ipInputField.onEndEdit.AddListener(_ => PushToConnector());
        portInputField.onEndEdit.AddListener(_ => PushToConnector());
    }

    private void Update()
    {
        if (connectionStatusText != null && rosConnector != null)
        {
            if (rosConnector.IsOnline)
            {
                connectionStatusText.text  = "CONECTADO";
                connectionStatusText.color = ColorConnected;
            }
            else
            {
                connectionStatusText.text  = "DESCONECTADO";
                connectionStatusText.color = ColorDisconnected;
            }
        }

        if (elapsedTimeText != null)
        {
            int total   = (int)Time.realtimeSinceStartup;
            int hours   = total / 3600;
            int minutes = (total % 3600) / 60;
            int seconds = total % 60;
            elapsedTimeText.text = string.Format("{0:D2}:{1:D2}:{2:D2}", hours, minutes, seconds);
        }
    }

    // RosBridgeServerUrl → input fields
    public void PullFromConnector()
    {
        if (rosConnector == null) return;

        // Expected format: ws://HOST:PORT
        string url = rosConnector.RosBridgeServerUrl;
        string hostPort = url.Contains("://") ? url.Substring(url.IndexOf("://") + 3) : url;
        int lastColon = hostPort.LastIndexOf(':');

        if (lastColon >= 0)
        {
            _updating = true;
            ipInputField.text = hostPort.Substring(0, lastColon);
            portInputField.text = hostPort.Substring(lastColon + 1);
            _updating = false;
        }
        else
        {
            ipInputField.text = hostPort;
            portInputField.text = "";
        }
    }

    // Input fields → RosBridgeServerUrl
    public void PushToConnector()
    {
        if (_updating || rosConnector == null) return;
        string scheme = rosConnector.RosBridgeServerUrl.Contains("://")
            ? rosConnector.RosBridgeServerUrl.Substring(0, rosConnector.RosBridgeServerUrl.IndexOf("://") + 3)
            : "ws://";
        rosConnector.RosBridgeServerUrl = scheme + ipInputField.text + ":" + portInputField.text;
    }
}
