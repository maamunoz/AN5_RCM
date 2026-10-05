using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR.Interaction.Toolkit.Interactables;

/// Deja mover una ventana World Space con los mandos del visor.
///
/// El sitio de cada ventana de Quest_test_arnes lo fija el constructor de la escena
/// (QuestSceneBuilder) y es el mismo para todo el mundo. Para lo que solo se mira eso
/// vale, pero SecTraj —cargar el .csv, ejecutar, pausar, parar— se maneja con las manos,
/// y ahí la altura y la distancia buenas dependen de quién lleve el visor y de si está
/// de pie o sentado. Este componente le pone a la ventana un asa: se agarra con el
/// gatillo (de lejos con el rayo, de cerca con la mano) y la ventana acompaña al mando
/// hasta que se suelta.
///
/// Se monta a sí mismo en runtime, igual que VrKeyboard y por el mismo motivo: la escena
/// Quest se reconstruye entera desde AN5_sim en cada pasada, así que cualquier jerarquía
/// dejada a mano se perdería. Al constructor le basta con añadir este componente a la
/// ventana (ver k_MovableWindows en QuestSceneBuilder).
///
/// Lo que se mueve es la ventana entera, o sea el GameObject donde está este componente;
/// el asa es solo el trozo por el que se la coge.
[RequireComponent(typeof(RectTransform))]
public class VrWindowGrab : MonoBehaviour
{
    [Header("Asa")]
    // La separación sí se mide en METROS y no en px del canvas, al revés que el resto de
    // la franja: cada ventana va a una escala muy distinta (SecTraj a 0.00184 m/px y
    // Panel_trayectorias a 0.0009) y lo que tiene que quedar igual de aire en todas es el
    // hueco real entre el contenido y el asa. El ancho y el alto de la franja, en cambio,
    // se sacan del propio rect en Build() -- ver k_BarWidthFraction y k_BarHeightFraction
    // -- así que no hacen falta en metros.
    [Tooltip("Separación entre el contenido de la ventana y el asa, en metros.")]
    public float barGap = 0.012f;

    [Tooltip("Grosor de la caja de agarre, en metros. Es lo que sobresale hacia el " +
             "operador para que la mano tenga volumen que tocar; el asa se dibuja plana.")]
    public float grabDepth = 0.09f;

    [Tooltip("Si se deja puesto, el asa se cuelga del borde inferior de ESTE rect en vez " +
             "de el de toda la ventana (ver ContentBounds) -- para cuando la ventana tiene, " +
             "al lado de la sección a la que de verdad quiere quedar pegada el asa, otra " +
             "pieza que crece de alto y correría el asa cuadro a cuadro. QuestSceneBuilder " +
             "lo usa para Panel_trayectorias: el asa cuelga de SecCartInput (el panel " +
             "cartesiano) y no del panel de la cola de coordenadas, que crece con cada " +
             "punto que se agrega. Solo manda en el alto de la posición: el ancho y el " +
             "centrado en X siguen saliendo de toda la ventana, ver Build().")]
    public RectTransform boundsAnchor;

    [Header("Límites de colocación")]
    [Tooltip("Altura mínima del centro de la ventana, en metros. Impide dejarla bajo el suelo.")]
    public float minHeight = 0.35f;

    [Tooltip("Altura máxima del centro de la ventana, en metros.")]
    public float maxHeight = 2.2f;

    // Abajo y centrada a lo ancho, sin rótulo: una franja azul bien visible justo debajo
    // del contenido, sin ocupar sitio a los costados (donde SecTraj y Panel_trayectorias sí
    // tienen otras ventanas cerca, ver los comentarios de QuestSceneBuilder sobre el
    // reparto angular). Un tercio del ancho de la ventana y no el ancho entero: alcanza
    // para agarrarla y dice claramente "esto es un asa", no "esto es el borde de la
    // ventana". Se saca del ancho de toda la ventana (ContentBounds), no del de
    // boundsAnchor cuando lo hay -- si no, en Panel_trayectorias el asa saldría del ancho
    // angosto de SecCartInput y quedaría diminuta.
    const float k_BarWidthFraction = 1f / 3f;

    // El alto es una fracción del propio ancho del asa y no un tamaño fijo en metros: así
    // la franja mantiene la misma proporción alargada sea cual sea la ventana, en vez de
    // quedar gruesa en las ventanas angostas y fina en las anchas. 1/15 y no 1/5: a 1/5
    // quedaba demasiado gruesa.
    const float k_BarHeightFraction = 1f / 15f;

    static readonly Color k_Idle = new Color(0.76f, 0.25f, 0.05f, 0.92f);
    static readonly Color k_Hover = new Color(0.976f, 0.451f, 0.086f, 0.96f);
    static readonly Color k_Held = new Color(0.984f, 0.573f, 0.235f, 1.00f);

    Image _bar;
    XRSimpleInteractable _interactable;

    // Pose del mando que la sostiene, null mientras no la sostenga nadie.
    Transform _attach;
    // Sitio y orientación de la ventana vistos desde ese mando, congelados al agarrar.
    Vector3 _offset;
    Quaternion _rotation;

    void Start()
    {
        Build();
    }

    void OnDestroy()
    {
        if (_interactable == null) return;

        _interactable.selectEntered.RemoveListener(OnSelectEntered);
        _interactable.selectExited.RemoveListener(OnSelectExited);
        _interactable.hoverEntered.RemoveListener(OnHoverEntered);
        _interactable.hoverExited.RemoveListener(OnHoverExited);
    }

    /// Sigue al mando en LateUpdate, después de que XRI haya procesado los interactores
    /// en su Update: hacerlo antes dejaría la ventana un fotograma por detrás de la mano,
    /// que en el visor se nota como que va "blanda".
    void LateUpdate()
    {
        if (_attach == null) return;

        var frame = YawOf(_attach.rotation);
        var position = _attach.position + frame * _offset;
        position.y = Mathf.Clamp(position.y, minHeight, maxHeight);

        transform.SetPositionAndRotation(position, frame * _rotation);
    }

    void OnSelectEntered(SelectEnterEventArgs args)
    {
        var attach = args.interactorObject.GetAttachTransform(_interactable);
        _attach = attach != null ? attach : args.interactorObject.transform;

        // Se congela la ventana en el marco del mando, y solo en yaw: así el cabeceo y el
        // alabeo de la muñeca no la vuelcan, y su plano sigue conteniendo el eje vertical
        // del mundo —perpendicular a la tapa de la mesa, como todas las demás ventanas de
        // la escena— la dejes donde la dejes.
        var frame = Quaternion.Inverse(YawOf(_attach.rotation));
        _offset = frame * (transform.position - _attach.position);
        _rotation = frame * YawOf(transform.rotation);

        SetBarColor(k_Held);
    }

    void OnSelectExited(SelectExitEventArgs args)
    {
        _attach = null;
        SetBarColor(_interactable != null && _interactable.isHovered ? k_Hover : k_Idle);
    }

    void OnHoverEntered(HoverEnterEventArgs args)
    {
        if (_attach == null) SetBarColor(k_Hover);
    }

    void OnHoverExited(HoverExitEventArgs args)
    {
        if (_attach == null && _interactable != null && !_interactable.isHovered) SetBarColor(k_Idle);
    }

    void SetBarColor(Color color)
    {
        if (_bar != null) _bar.color = color;
    }

    /// Yaw puro de una orientación: la vuelta que le da alrededor de la vertical del
    /// mundo, sin cabeceo ni alabeo.
    ///
    /// No se saca de eulerAngles.y: apuntando a plomo (que es como se coge una ventana
    /// que está por debajo de la vista, justo el caso de SecTraj) el yaw y el alabeo se
    /// confunden y ese valor pega saltos. Se proyecta el eje del mando sobre el plano
    /// horizontal, y si ese eje es justo el vertical se recurre a su "arriba", que
    /// entonces sí es horizontal.
    static Quaternion YawOf(Quaternion rotation)
    {
        var direction = rotation * Vector3.forward;
        direction.y = 0f;

        if (direction.sqrMagnitude < 1e-6f)
        {
            direction = rotation * Vector3.up;
            direction.y = 0f;
            if (direction.sqrMagnitude < 1e-6f) return Quaternion.identity;
        }

        return Quaternion.LookRotation(direction.normalized, Vector3.up);
    }

    // -----------------------------------------------------------------
    // Construcción
    // -----------------------------------------------------------------
    void Build()
    {
        var window = (RectTransform)transform;

        // Sin esto, medir aquí en Start() lee la jerarquía a medio asentar: los
        // VerticalLayoutGroup/HorizontalLayoutGroup de CenterBottom, JogRow, JogColumn
        // etc. no recalculan sus rects al vuelo, sino que Unity los encola y los resuelve
        // una vez por fotograma, DESPUÉS de que corren todos los Start() -- no durante
        // ellos. El primer Build() de la sesión llegaba antes de esa pasada y calculaba
        // el asa contra rects transitorios (a veces con el propio CenterBottom todavía
        // en su alto viejo), dando un tamaño y una posición sin relación con el panel
        // real; unos fotogramas después, con el layout ya asentado, todo medía bien --
        // pero el asa ya había nacido mal y se quedaba así. Vaciar la cola a mano antes
        // de medir dejaba a ContentBounds/CalculateRelativeRectTransformBounds leer el
        // layout ya resuelto, igual que hace TrajectoryFileList.Refresh() para el mismo
        // problema con el ajuste de línea del texto.
        Canvas.ForceUpdateCanvases();

        // Dos cajas distintas y no una: el TAMAÑO sale siempre de TODA la ventana
        // (totalBounds), pero el CENTRADO EN X Y el borde en Y salen de boundsAnchor
        // cuando lo hay. Para Panel_trayectorias eso significa un asa ancha (1/3 del
        // panel entero) pero centrada sobre SecCartInput y colgando justo debajo de él,
        // no del panel de la cola, que crece con cada punto que se agrega -- ver
        // boundsAnchor. Centrar en totalBounds.center.x en vez de en el propio
        // SecCartInput la dejaba a 126px a la derecha de donde tenía que estar: la fila
        // completa (JogColumn + la cola al lado) es más ancha que SecCartInput solo, y su
        // centro no coincide con el de SecCartInput.
        var totalBounds = ContentBounds(window);
        var anchorBounds = boundsAnchor != null
            ? RectTransformUtility.CalculateRelativeRectTransformBounds(window, boundsAnchor)
            : totalBounds;

        // Escala px -> m del canvas de esta ventana (se la pone QuestSceneBuilder con el
        // pixelToMeter de su fila). Si la ventana no estuviera escalada, sus px ya serían
        // metros y no hay nada que convertir.
        var scale = window.lossyScale.x;
        if (scale < 1e-6f) scale = 1f;
        var gapPx = barGap / scale;
        var depthPx = grabDepth / scale;

        // Un tercio del ancho de la ventana, y el alto un quinto de ese ancho: una franja
        // horizontal corta y alargada, no una tira que corra de punta a punta ni un
        // cuadrado.
        var barWidth = totalBounds.size.x * k_BarWidthFraction;
        var barHeight = barWidth * k_BarHeightFraction;

        var go = new GameObject("Grab Handle", typeof(RectTransform));
        var bar = (RectTransform)go.transform;
        bar.SetParent(window, worldPositionStays: false);
        bar.anchorMin = new Vector2(0.5f, 0.5f);
        bar.anchorMax = new Vector2(0.5f, 0.5f);
        bar.pivot = new Vector2(0.5f, 0.5f);
        bar.sizeDelta = new Vector2(barWidth, barHeight);

        // El ancla está en el centro del rect de la ventana, que no tiene por qué caer en
        // el centro de lo que la ventana dibuja: las secciones heredadas de AN5_sim traen
        // el rect de la maquetación de escritorio con zonas vacías dentro. Centrada en X
        // y pegada al borde inferior en Y, las dos sobre boundsAnchor (o toda la ventana
        // si no hay).
        var center = new Vector2(anchorBounds.center.x, anchorBounds.min.y - gapPx - barHeight * 0.5f);
        bar.anchoredPosition = center - window.rect.center;

        _bar = go.AddComponent<Image>();
        _bar.sprite = VrUiKit.RoundedRectSprite();
        _bar.type = Image.Type.Sliced;
        _bar.color = k_Idle;
        // El agarre entra por el collider, no por el raycaster de UI. Si el asa fuese
        // además blanco de UI, el rayo tendría dos cosas que golpear en el mismo sitio y
        // la de UI, que queda por delante, le ganaría al collider.
        _bar.raycastTarget = false;

        // Los interactores del rig disparan sus rayos contra las capas Default y UI. El
        // canvas de la ventana viene de AN5_sim y puede estar en cualquiera de ellas, así
        // que el asa se pone en Default explícitamente y no hereda nada.
        go.layer = 0;

        var box = go.AddComponent<BoxCollider>();
        box.size = new Vector3(barWidth, barHeight, depthPx);
        // Un canvas se ve desde su cara -Z, así que el volumen sobresale hacia ahí: hacia
        // el lado por el que llega la mano.
        box.center = new Vector3(0f, 0f, -depthPx * 0.5f);

        // Después del collider a propósito: XRBaseInteractable.Awake —que corre dentro de
        // este AddComponent— recoge los colliders que encuentre en su GameObject y sus
        // hijos y los registra en el XRInteractionManager. Uno añadido más tarde no
        // entraría en ese registro y el asa no se podría coger.
        _interactable = go.AddComponent<XRSimpleInteractable>();
        _interactable.selectEntered.AddListener(OnSelectEntered);
        _interactable.selectExited.AddListener(OnSelectExited);
        _interactable.hoverEntered.AddListener(OnHoverEntered);
        _interactable.hoverExited.AddListener(OnHoverExited);
    }

    /// Caja de lo que la ventana dibuja de verdad, en su espacio local: la unión de sus
    /// hijos activos, igual que hace QuestSceneBuilder para colocarla. Si no hay hijos
    /// que medir se cae al rect, que es lo único que queda.
    static Bounds ContentBounds(RectTransform window)
    {
        var bounds = new Bounds();
        var any = false;

        for (var i = 0; i < window.childCount; i++)
        {
            var child = window.GetChild(i);
            if (!child.gameObject.activeSelf) continue;

            var childBounds = RectTransformUtility.CalculateRelativeRectTransformBounds(window, child);
            if (!any) { bounds = childBounds; any = true; }
            else bounds.Encapsulate(childBounds);
        }

        if (!any)
            bounds = new Bounds(window.rect.center, window.rect.size);

        return bounds;
    }
}
