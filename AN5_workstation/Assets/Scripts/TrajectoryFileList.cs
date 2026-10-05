using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;
using UnityEngine.UI;

/// Lista desplegable en VR de los archivos de trayectorias disponibles en una carpeta.
///
/// Existe para Android/Quest, donde no hay diálogo nativo de archivos que abrir (ver el
/// comentario de SecTrajController.ShowNativeFileDialog) y encima no habría dónde
/// pintarlo dentro de la sesión inmersiva de OpenXR aunque lo hubiera. En vez de eso,
/// esto arma a mano una columna de botones -- uno por archivo -- colgando al lado de la
/// ventana de SecTraj, tocable con el mismo rayo de los mandos que el resto del panel.
///
/// Se monta a sí mismo en runtime, igual que VrKeyboard y VrWindowGrab: no depende de
/// nada puesto a mano en la escena, así que sobrevive a que QuestSceneBuilder la
/// reconstruya entera desde AN5_sim en cada pasada.
///
/// Vuelve a leer el disco (Directory.GetFiles) cada vez que se abre y cada vez que se
/// toca "Refrescar" -- un archivo copiado por adb push mientras la app ya está corriendo
/// aparece sin reinstalar nada, basta con cerrar y reabrir la lista.
public class TrajectoryFileList : MonoBehaviour
{
    const float k_PanelWidth  = 480f;
    // Tres rondas bajando este número (52 -> 32 -> 20 -> 15) sin que la fila se viera
    // más chica: el motivo real no era el valor, era que rootLayout/rowsLayout tenían
    // childControlHeight=false, así que LayoutElement.preferredHeight -- lo único que
    // se estaba tocando en cada ronda -- nunca se llegaba a aplicar. Cada fila se
    // quedaba en 100, el alto por defecto de un RectTransform nuevo de Unity, sin
    // importar qué valor tuviera preferredHeight. Con childControlHeight=true (ver
    // Create()) esto por fin manda de verdad; 15 alcanza de sobra para el texto a
    // k_RowFontSize.
    const float k_RowHeight   = 15f;
    const float k_RowSpacing  = 1f;
    const float k_PanelPadding = 5f;
    // La cabecera (título + ⟳/✕) necesita más que k_RowHeight: por el mismo bug de
    // arriba, esos botones llevaban meses con preferredWidth respetado pero preferred-
    // Height ignorado -- 15 de ancho por 100 de alto, un palito. Con el alto ya
    // controlado de verdad, un botón de icono tocable por rayo necesita su propio
    // tamaño, no el de una fila de texto.
    const float k_HeaderHeight = 26f;
    const float k_GlyphButtonWidth = 26f;
    const int   k_TitleFontSize = 10;
    const int   k_RowFontSize   = 9;
    const int   k_GlyphFontSize = 14; // Refrescar/Cerrar: iconos, más grandes que el texto de fila.
    const int   k_MoreFontSize  = 8;
    const int   k_MaxRows     = 12; // sin ScrollRect: cap razonable para no salirse de la sala.
    static readonly string[] k_Extensions = { "*.txt", "*.csv" };

    RectTransform _rows;
    Text          _titleText;
    string        _directory;
    Action<string> _onChosen;

    /// Muestra (creándola si hace falta) la lista colgando a la derecha de `anchor` y la
    /// refresca contra `directory`. `anchor` tiene que ser un RectTransform que ya cuelgue
    /// de un Canvas World Space con TrackedDeviceGraphicRaycaster -- la ventana de SecTraj
    /// ya lo es, ver QuestSceneBuilder.DetachAsWindow.
    public static TrajectoryFileList Show(RectTransform anchor, string directory, Action<string> onChosen)
    {
        var picker = anchor.GetComponentInChildren<TrajectoryFileList>(true);
        if (picker == null) picker = Create(anchor);

        picker._directory = directory;
        picker._onChosen  = onChosen;
        picker.gameObject.SetActive(true);
        picker.Refresh();
        return picker;
    }

    public static void Hide(RectTransform anchor)
    {
        var picker = anchor.GetComponentInChildren<TrajectoryFileList>(true);
        if (picker != null) picker.gameObject.SetActive(false);
    }

    public static bool IsVisible(RectTransform anchor)
    {
        var picker = anchor.GetComponentInChildren<TrajectoryFileList>(true);
        return picker != null && picker.gameObject.activeSelf;
    }

    void Refresh()
    {
        foreach (Transform child in _rows)
            Destroy(child.gameObject);

        var font = VrUiKit.DefaultFont();

        List<string> files;
        try
        {
            files = k_Extensions
                .SelectMany(pattern => Directory.GetFiles(_directory, pattern))
                .Distinct()
                .OrderByDescending(File.GetLastWriteTimeUtc) // lo recién copiado por adb push arriba de todo
                .ToList();
        }
        catch (Exception e)
        {
            files = new List<string>();
            Debug.LogWarning($"[TrajectoryFileList] No se pudo leer '{_directory}': {e.Message}");
        }

        _titleText.text = $"Archivos ({files.Count})";

        if (files.Count == 0)
        {
            var empty = VrUiKit.MakeRow(_rows, "Empty");
            VrUiKit.MakeText(empty, $"No hay archivos en\n{_directory}", font, k_RowFontSize, TextAnchor.MiddleLeft);
            var empties = empty.GetComponent<LayoutElement>();
            empties.preferredHeight = k_RowHeight * 1.5f;
        }
        else
        {
            int shown = Mathf.Min(files.Count, k_MaxRows);
            for (int i = 0; i < shown; i++)
            {
                string path = files[i];
                var button = VrUiKit.MakeButton(_rows, Path.GetFileName(path), font, k_RowFontSize);
                button.GetComponent<LayoutElement>().preferredHeight = k_RowHeight;
                button.onClick.AddListener(() =>
                {
                    _onChosen?.Invoke(path);
                    gameObject.SetActive(false);
                });
            }

            if (files.Count > shown)
            {
                var more = VrUiKit.MakeRow(_rows, "More");
                VrUiKit.MakeText(more, $"(+{files.Count - shown} más -- no se muestran)", font, k_MoreFontSize, TextAnchor.MiddleLeft);
            }
        }

        // Sin esto, el primer Text de cada fila recién creada mide su ajuste de línea
        // contra un rect que el LayoutGroup todavía no terminó de resolver (a veces
        // efectivamente 0 de ancho) -- con horizontalOverflow en Wrap eso parte cada
        // nombre de archivo letra por letra, en una columna vertical, en vez de leerse
        // de corrido. ForceRebuildLayoutImmediate por sí solo no basta: solo repasa lo
        // que ya está marcado sucio, y el propio Destroy/creación de filas de arriba deja
        // sin vaciar la cola de CanvasUpdateRegistry de este mismo fotograma. Vaciarla
        // primero con ForceUpdateCanvases dejas todo resuelto antes del rebuild explícito,
        // así cada Text mide contra su ancho real antes de que se vea nada.
        Canvas.ForceUpdateCanvases();
        LayoutRebuilder.ForceRebuildLayoutImmediate((RectTransform)transform);
    }

    static TrajectoryFileList Create(RectTransform anchor)
    {
        var font = VrUiKit.DefaultFont();

        var rootGo = new GameObject("TrajectoryFileList", typeof(RectTransform));
        var root = (RectTransform)rootGo.transform;
        root.SetParent(anchor, worldPositionStays: false);
        root.pivot     = new Vector2(0f, 1f);
        root.anchorMin = new Vector2(1f, 1f);
        root.anchorMax = new Vector2(1f, 1f);
        root.anchoredPosition = new Vector2(24f, 0f);
        root.sizeDelta = new Vector2(k_PanelWidth, 0f);

        var bg = rootGo.AddComponent<Image>();
        bg.color = new Color(0.05f, 0.05f, 0.05f, 0.96f);

        var rootLayout = rootGo.AddComponent<VerticalLayoutGroup>();
        rootLayout.padding = new RectOffset((int)k_PanelPadding, (int)k_PanelPadding, (int)k_PanelPadding, (int)k_PanelPadding);
        rootLayout.spacing = k_RowSpacing;
        rootLayout.childControlWidth = true;
        rootLayout.childForceExpandWidth = true;
        // true: sin esto, LayoutElement.preferredHeight de Header/Rows (más abajo) no
        // se aplica y ambos se quedan en el alto por defecto de un RectTransform nuevo
        // (100) -- ver el comentario largo junto a k_RowHeight.
        rootLayout.childControlHeight = true;
        rootLayout.childForceExpandHeight = false;

        var fitter = rootGo.AddComponent<ContentSizeFitter>();
        fitter.verticalFit = ContentSizeFitter.FitMode.PreferredSize;

        var picker = rootGo.AddComponent<TrajectoryFileList>();

        // Título + Refrescar + Cerrar, en una fila.
        var header = VrUiKit.MakeRow(root, "Header");
        var headerLayout = header.gameObject.AddComponent<HorizontalLayoutGroup>();
        headerLayout.spacing = k_RowSpacing;
        headerLayout.childControlWidth = true;
        headerLayout.childForceExpandWidth = false;
        // true por la misma razón que rootLayout: sin esto, refreshBtn/closeBtn
        // ignoraban preferredHeight y quedaban en 100 -- anchos (preferredWidth sí se
        // aplicaba, ese es childControlWidth) pero altísimos, un palito.
        headerLayout.childControlHeight = true;
        headerLayout.childForceExpandHeight = false;
        header.GetComponent<LayoutElement>().preferredHeight = k_HeaderHeight;

        picker._titleText = VrUiKit.MakeText(header, "Archivos", font, k_TitleFontSize, TextAnchor.MiddleLeft);
        picker._titleText.GetComponent<LayoutElement>().flexibleWidth = 1f;

        // preferredWidth Y preferredHeight los dos: el Button es una Image con el
        // texto del glifo como hijo aparte, no un Text en el propio GameObject como
        // picker._titleText de arriba -- sin su propio ILayoutElement que reporte un
        // alto de respaldo, con childControlHeight=true y solo el ancho fijado el
        // alto quedaba en 0 (invisible).
        var refreshBtn = VrUiKit.MakeButton(header, "⟳", font, k_GlyphFontSize); // ⟳
        var refreshLE = refreshBtn.GetComponent<LayoutElement>();
        refreshLE.preferredWidth = k_GlyphButtonWidth;
        refreshLE.preferredHeight = k_HeaderHeight;
        refreshBtn.onClick.AddListener(picker.Refresh);

        var closeBtn = VrUiKit.MakeButton(header, "✕", font, k_GlyphFontSize); // ✕
        var closeLE = closeBtn.GetComponent<LayoutElement>();
        closeLE.preferredWidth = k_GlyphButtonWidth;
        closeLE.preferredHeight = k_HeaderHeight;
        closeBtn.onClick.AddListener(() => picker.gameObject.SetActive(false));

        var rowsGo = VrUiKit.MakeRow(root, "Rows");
        var rowsLayout = rowsGo.gameObject.AddComponent<VerticalLayoutGroup>();
        rowsLayout.spacing = k_RowSpacing;
        rowsLayout.childControlWidth = true;
        rowsLayout.childForceExpandWidth = true;
        // true: mismo motivo -- sin esto cada fila de archivo ignoraba
        // preferredHeight=k_RowHeight (Refresh(), más abajo) y quedaba en 100 pese a
        // que el número que se le pasaba bajara cada vez.
        rowsLayout.childControlHeight = true;
        rowsLayout.childForceExpandHeight = false;
        Destroy(rowsGo.GetComponent<LayoutElement>()); // el contenedor de filas no necesita tamaño propio
        picker._rows = rowsGo;

        return picker;
    }
}
