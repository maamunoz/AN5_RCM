using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit.UI;

/// Teclado virtual en World Space para el visor.
///
/// En escritorio los InputField de la aplicación se rellenan tecleando; en el visor no
/// hay teclado, y un InputField heredado bajo OpenXR pelado no levanta el del sistema de
/// Meta. Este componente cubre ese hueco: vigila el foco, y en cuanto uno de esos campos
/// se selecciona abre un teclado flotante delante del operador, al alcance del rayo y del
/// toque directo.
///
/// Se construye por código a propósito, no desde la escena: Quest_test_arnes se
/// reconstruye entera desde AN5_sim en cada pasada (ver QuestSceneBuilder), así que
/// cualquier jerarquía que se dejara montada a mano se perdería. Basta con que el
/// constructor deje un GameObject con este componente.
public class VrKeyboard : MonoBehaviour
{
    [Header("Colocación")]
    [Tooltip("m por delante del operador al abrirse.")]
    public float distance = 0.80f;

    [Tooltip("Grados por debajo de la vista. Negativo = se mira hacia abajo, como un teclado sobre la mesa.")]
    public float elevationDeg = -28f;

    [Tooltip("Ancho del teclado en metros.")]
    public float width = 0.66f;

    // Cuatro filas de diez para que la rejilla salga pareja. El punto, el guion, el guion
    // bajo y la barra están porque los campos que hay que rellenar son coordenadas con
    // decimales y signo, una IP, un puerto y un nombre de archivo .csv.
    static readonly string[] k_Rows =
    {
        "1234567890",
        "QWERTYUIOP",
        "ASDFGHJKL.",
        "ZXCVBNM-_/",
    };

    // --- Maquetación, en px; el conjunto se escala luego a `width` metros ---
    const float k_Key = 88f;
    const float k_Gap = 8f;
    const float k_Pad = 16f;
    const float k_PreviewHeight = 72f;
    const int k_Columns = 10;

    RectTransform _root;
    Text _preview;
    InputField _target;
    string _original;
    Camera _camera;

    void Start()
    {
        Build();
        _root.gameObject.SetActive(false);
    }

    void Update()
    {
        var events = EventSystem.current;
        if (events == null) return;

        var selected = events.currentSelectedGameObject;
        if (selected == null) return;

        // Pulsar una tecla mueve el foco a la propia tecla; eso no cierra nada.
        if (_root != null && selected.transform.IsChildOf(_root)) return;

        var field = selected.GetComponent<InputField>();
        if (field != null)
        {
            if (field != _target) Open(field);
            return;
        }

        // Se ha seleccionado otra cosa que no es ni un campo ni el teclado: se cierra sin
        // aplicar, que es lo que espera cualquiera que haya tocado fuera.
        if (_target != null) Cancel();
    }

    void Open(InputField field)
    {
        _target = field;
        _original = field.text;

        // El campo se suelta del EventSystem nada más abrir. Si se quedara seleccionado,
        // la primera tecla que se pulsara le robaría el foco y Unity dispararía su
        // onEndEdit a mitad de escritura — y a ese evento le cuelgan cosas como
        // SecCartInputController, que manda la pose al robot. A partir de aquí se escribe
        // en el campo a mano y el evento se dispara una sola vez, al aceptar.
        if (EventSystem.current != null && EventSystem.current.currentSelectedGameObject == field.gameObject)
            EventSystem.current.SetSelectedGameObject(null);

        // La cámara puede no existir todavía cuando se construye el teclado (el rig se
        // instancia aparte), así que se resuelve también aquí.
        var canvas = _root.GetComponent<Canvas>();
        if (canvas != null && canvas.worldCamera == null) canvas.worldCamera = ResolveCamera();

        _root.gameObject.SetActive(true);
        Reposition();
        Refresh();
    }

    /// Se coloca delante de donde mire el operador en ese momento, girando solo en yaw:
    /// como el resto de ventanas de la escena, queda perpendicular a la tapa de la mesa.
    void Reposition()
    {
        var eye = ResolveCamera();
        if (eye == null) return;

        var forward = eye.transform.forward;
        forward.y = 0f;
        if (forward.sqrMagnitude < 1e-4f) forward = Vector3.forward;
        forward.Normalize();

        var position = eye.transform.position + forward * distance;
        position.y = eye.transform.position.y + distance * Mathf.Tan(elevationDeg * Mathf.Deg2Rad);

        _root.SetPositionAndRotation(position, Quaternion.LookRotation(forward, Vector3.up));
    }

    void Type(string character)
    {
        if (_target == null) return;
        Write(_target.text + character);
    }

    void Backspace()
    {
        if (_target == null) return;
        var text = _target.text;
        if (text.Length > 0) Write(text.Substring(0, text.Length - 1));
    }

    /// Escribe sin avisar a nadie. Mientras se teclea no se dispara ni onValueChanged ni
    /// onEndEdit: lo que se ve es la vista previa del propio teclado.
    void Write(string text)
    {
        _target.SetTextWithoutNotify(text);
        Refresh();
    }

    void Accept()
    {
        var field = _target;
        var text = field != null ? field.text : null;
        Close();

        if (field == null) return;

        // Ahora sí, una sola vez y con el valor definitivo.
        field.text = text;              // onValueChanged
        field.onEndEdit?.Invoke(text);  // onEndEdit
    }

    void Cancel()
    {
        var field = _target;
        var original = _original;
        Close();

        if (field != null) field.SetTextWithoutNotify(original);
    }

    void Close()
    {
        _target = null;
        _original = null;
        if (_root != null) _root.gameObject.SetActive(false);
    }

    void Refresh()
    {
        if (_preview != null) _preview.text = _target != null ? _target.text : string.Empty;
    }

    Camera ResolveCamera()
    {
        if (_camera != null) return _camera;
        _camera = Camera.main != null ? Camera.main : FindFirstObjectByType<Camera>();
        return _camera;
    }

    // -----------------------------------------------------------------
    // Construcción
    // -----------------------------------------------------------------
    void Build()
    {
        var rowWidth = k_Columns * k_Key + (k_Columns - 1) * k_Gap;
        var panelWidth = rowWidth + 2f * k_Pad;
        var panelHeight = 2f * k_Pad + k_PreviewHeight + k_Gap
                          + k_Rows.Length * (k_Key + k_Gap)
                          + k_Key;

        var go = new GameObject("VR Keyboard", typeof(RectTransform));
        go.transform.SetParent(transform, worldPositionStays: false);

        _root = (RectTransform)go.transform;
        _root.sizeDelta = new Vector2(panelWidth, panelHeight);
        _root.localScale = Vector3.one * (width / panelWidth);

        var canvas = go.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.WorldSpace;
        canvas.worldCamera = ResolveCamera();
        canvas.sortingOrder = 500;   // por delante de las ventanas que pueda pillar detrás

        var scaler = go.AddComponent<CanvasScaler>();
        scaler.dynamicPixelsPerUnit = 2f;   // si no, el texto pequeño sale borroso en el visor

        go.AddComponent<GraphicRaycaster>();
        go.AddComponent<TrackedDeviceGraphicRaycaster>();   // sin esto el rayo del mando no lo ve

        var background = go.AddComponent<Image>();
        background.color = new Color(0.05f, 0.05f, 0.05f, 0.96f);

        var font = ResolveFont();
        var cursor = -k_Pad;

        _preview = MakeLabel(_root, new Vector2(k_Pad, cursor), new Vector2(rowWidth, k_PreviewHeight),
                             string.Empty, font, 44, TextAnchor.MiddleLeft);
        cursor -= k_PreviewHeight + k_Gap;

        foreach (var row in k_Rows)
        {
            for (var i = 0; i < row.Length; i++)
            {
                var character = row[i].ToString();
                MakeKey(_root, new Vector2(k_Pad + i * (k_Key + k_Gap), cursor),
                        new Vector2(k_Key, k_Key), character, font, 40, () => Type(character));
            }

            cursor -= k_Key + k_Gap;
        }

        // Fila de abajo: 2 + 4 + 2 + 2 columnas, que suman las diez de arriba.
        var neutral = new Color(0.15f, 0.15f, 0.15f);
        var x = k_Pad;
        x = AddWideKey(x, cursor, 2, "Cancelar", font, 30, Cancel, new Color(0.35f, 0.12f, 0.14f));
        x = AddWideKey(x, cursor, 4, "espacio", font, 30, () => Type(" "), neutral);
        x = AddWideKey(x, cursor, 2, "⌫", font, 40, Backspace, neutral);
        AddWideKey(x, cursor, 2, "OK", font, 34, Accept, new Color(0.76f, 0.25f, 0.05f));
    }

    /// Coloca una tecla de varias columnas y devuelve la x donde empieza la siguiente.
    float AddWideKey(float x, float y, int columns, string label, Font font, int fontSize,
                     UnityEngine.Events.UnityAction action, Color color)
    {
        var keyWidth = columns * k_Key + (columns - 1) * k_Gap;
        MakeKey(_root, new Vector2(x, y), new Vector2(keyWidth, k_Key), label, font, fontSize, action, color);
        return x + keyWidth + k_Gap;
    }

    static Image MakeBox(RectTransform parent, Vector2 topLeft, Vector2 size, Color color)
    {
        var go = new GameObject("Box", typeof(RectTransform));
        var rect = (RectTransform)go.transform;
        rect.SetParent(parent, worldPositionStays: false);
        rect.anchorMin = new Vector2(0f, 1f);
        rect.anchorMax = new Vector2(0f, 1f);
        rect.pivot = new Vector2(0f, 1f);
        rect.sizeDelta = size;
        rect.anchoredPosition = topLeft;

        var image = go.AddComponent<Image>();
        image.color = color;
        return image;
    }

    Text MakeLabel(RectTransform parent, Vector2 topLeft, Vector2 size, string text,
                   Font font, int fontSize, TextAnchor alignment)
    {
        var box = MakeBox(parent, topLeft, size, new Color(0.03f, 0.03f, 0.03f, 1f));
        box.gameObject.name = "Preview";

        var labelGo = new GameObject("Text", typeof(RectTransform));
        var labelRect = (RectTransform)labelGo.transform;
        labelRect.SetParent(box.transform, worldPositionStays: false);
        labelRect.anchorMin = Vector2.zero;
        labelRect.anchorMax = Vector2.one;
        labelRect.offsetMin = new Vector2(16f, 0f);
        labelRect.offsetMax = new Vector2(-16f, 0f);

        var label = labelGo.AddComponent<Text>();
        label.font = font;
        label.fontSize = fontSize;
        label.alignment = alignment;
        label.color = new Color(0.976f, 0.451f, 0.086f);
        label.text = text;
        label.horizontalOverflow = HorizontalWrapMode.Overflow;
        return label;
    }

    void MakeKey(RectTransform parent, Vector2 topLeft, Vector2 size, string label,
                 Font font, int fontSize, UnityEngine.Events.UnityAction action, Color? color = null)
    {
        var box = MakeBox(parent, topLeft, size, color ?? new Color(0.12f, 0.12f, 0.12f, 1f));
        box.gameObject.name = "Key " + label;

        var labelGo = new GameObject("Text", typeof(RectTransform));
        var labelRect = (RectTransform)labelGo.transform;
        labelRect.SetParent(box.transform, worldPositionStays: false);
        labelRect.anchorMin = Vector2.zero;
        labelRect.anchorMax = Vector2.one;
        labelRect.offsetMin = Vector2.zero;
        labelRect.offsetMax = Vector2.zero;

        var text = labelGo.AddComponent<Text>();
        text.font = font;
        text.fontSize = fontSize;
        text.alignment = TextAnchor.MiddleCenter;
        text.color = Color.white;
        text.text = label;
        text.horizontalOverflow = HorizontalWrapMode.Overflow;

        var button = box.gameObject.AddComponent<Button>();
        button.targetGraphic = box;
        button.onClick.AddListener(action);
    }

    /// Reutiliza la fuente que ya use la UI de la aplicación, para que el teclado no
    /// desentone y para no depender del nombre de los recursos internos de Unity, que ha
    /// cambiado entre versiones.
    static Font ResolveFont()
    {
        foreach (var text in FindObjectsByType<Text>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            if (text.font != null) return text.font;

        var builtin = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        return builtin != null ? builtin : Resources.GetBuiltinResource<Font>("Arial.ttf");
    }
}
