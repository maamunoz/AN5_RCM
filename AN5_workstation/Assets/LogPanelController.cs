using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class LogPanelController : MonoBehaviour
{
    [Header("References")]
    public Transform logBox;

    [Header("Settings")]
    public int maxEntries = 10;
    public float refreshInterval = 3f;

    [Header("Controls")]
    public UnityEngine.UI.Button clearButton;

    private static readonly Color ColorInfo    = new Color(0.35f, 0.65f, 1f);
    private static readonly Color ColorWarning = new Color(0.961f, 0.773f, 0.094f); // amarillo: el naranja es el acento
    private static readonly Color ColorError   = new Color(1f,   0.22f, 0.22f);

    private GameObject _template;
    private readonly Queue<(string msg, LogType type)> _messages
        = new Queue<(string, LogType)>();

    private void Awake()
    {
        // Use first existing LogEntry as template, remove the rest
        if (logBox != null && logBox.childCount > 0)
        {
            _template = logBox.GetChild(0).gameObject;
            _template.SetActive(false);
            for (int i = logBox.childCount - 1; i >= 1; i--)
                Destroy(logBox.GetChild(i).gameObject);
        }

        Application.logMessageReceived += OnLogReceived;
    }

    private void Start()
    {
        if (clearButton != null)
            clearButton.onClick.AddListener(ClearLog);

        InvokeRepeating(nameof(RefreshDisplay), 0f, refreshInterval);
    }

    public void ClearLog()
    {
        _messages.Clear();
        RefreshDisplay();
    }

    private void OnDestroy()
    {
        Application.logMessageReceived -= OnLogReceived;
    }

    private void OnLogReceived(string message, string stackTrace, LogType type)
    {
        if (type == LogType.Assert || type == LogType.Exception)
            type = LogType.Error;

        _messages.Enqueue((message, type));
        while (_messages.Count > maxEntries)
            _messages.Dequeue();
    }

    private void RefreshDisplay()
    {
        if (logBox == null || _template == null) return;

        // Destroy all displayed entries (index 0 is the template, skip it)
        for (int i = logBox.childCount - 1; i >= 1; i--)
            Destroy(logBox.GetChild(i).gameObject);

        foreach (var entry in _messages)
        {
            var go   = Instantiate(_template, logBox);
            go.name  = "LogEntry";
            go.SetActive(true);

            var text = go.GetComponent<Text>();
            if (text == null) continue;

            text.text  = entry.msg;
            text.color = EntryColor(entry.type);
        }
    }

    private static Color EntryColor(LogType type)
    {
        if (type == LogType.Warning) return ColorWarning;
        if (type == LogType.Error)   return ColorError;
        return ColorInfo;
    }
}
