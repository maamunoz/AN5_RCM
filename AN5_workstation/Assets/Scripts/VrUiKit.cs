using UnityEngine;
using UnityEngine.UI;

/// Piezas mínimas de uGUI para los paneles que se auto-montan en runtime en Android/Quest
/// (p. ej. TrajectoryFileList): ninguno tiene un Canvas preparado a mano en la escena,
/// así que arman el suyo desde cero, y todos necesitan la misma terna -- fila, texto,
/// botón -- para no repetir el boilerplate de uGUI en cada uno.
public static class VrUiKit
{
    public static Font DefaultFont()
    {
        return Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
    }

    const int k_RoundedTextureSize = 48;
    const int k_RoundedCornerRadius = 14;
    static Sprite s_roundedSprite;

    /// Sprite blanco de esquinas redondeadas, 9-sliceado para poder estirarlo a
    /// cualquier tamaño sin deformar las esquinas (border = k_RoundedCornerRadius en los
    /// cuatro lados). Se genera una sola vez (Texture2D dibujada a mano con la SDF de un
    /// rectángulo redondeado) y se cachea -- no hay ningún sprite de esquinas
    /// redondeadas entre los recursos internos de Unity con nombre estable entre
    /// versiones, así que en vez de depender de uno se dibuja el propio.
    ///
    /// Blanco y sin tocar el alfa fuera del contorno: un Image que lo use tiñe el color
    /// que haga falta con su propio `color`, el sprite es solo la forma.
    public static Sprite RoundedRectSprite()
    {
        if (s_roundedSprite != null) return s_roundedSprite;

        const int size = k_RoundedTextureSize;
        const float radius = k_RoundedCornerRadius;
        const float half = size * 0.5f;

        var tex = new Texture2D(size, size, TextureFormat.RGBA32, false);
        tex.wrapMode = TextureWrapMode.Clamp;
        tex.filterMode = FilterMode.Bilinear;

        var pixels = new Color32[size * size];
        for (int y = 0; y < size; y++)
        {
            for (int x = 0; x < size; x++)
            {
                // SDF de un rectángulo redondeado (Inigo Quilez): distancia con signo del
                // punto al borde, negativa adentro. Un solo píxel de degradado alrededor
                // de 0 basta para el antialiasing del contorno.
                float px = x + 0.5f - half;
                float py = y + 0.5f - half;
                float qx = Mathf.Abs(px) - (half - radius);
                float qy = Mathf.Abs(py) - (half - radius);
                float outsideX = Mathf.Max(qx, 0f);
                float outsideY = Mathf.Max(qy, 0f);
                float outside = Mathf.Sqrt(outsideX * outsideX + outsideY * outsideY);
                float inside = Mathf.Min(Mathf.Max(qx, qy), 0f);
                float dist = outside + inside - radius;

                float alpha = Mathf.Clamp01(0.5f - dist);
                pixels[y * size + x] = new Color(1f, 1f, 1f, alpha);
            }
        }

        tex.SetPixels32(pixels);
        tex.Apply();

        var border = new Vector4(radius, radius, radius, radius);
        s_roundedSprite = Sprite.Create(tex, new Rect(0, 0, size, size), new Vector2(0.5f, 0.5f), 100f, 0,
            SpriteMeshType.FullRect, border);
        return s_roundedSprite;
    }

    /// Un RectTransform vacío con LayoutElement, listo para colgar de un
    /// VerticalLayoutGroup/HorizontalLayoutGroup del padre (que manda su tamaño) o para
    /// llevar su propio LayoutGroup como contenedor de más filas.
    public static RectTransform MakeRow(Transform parent, string name)
    {
        var go = new GameObject(name, typeof(RectTransform));
        var rect = (RectTransform)go.transform;
        rect.SetParent(parent, worldPositionStays: false);
        rect.localScale = Vector3.one;
        go.AddComponent<LayoutElement>();
        return rect;
    }

    /// El LayoutElement que se agrega acá empieza en blanco (todo -1, sin opinión) así
    /// que no cambia nada por sí solo -- está para que un llamador pueda pedirle
    /// GetComponent<LayoutElement>() y fijarle flexibleWidth/preferredHeight/etc. sin
    /// que eso explote con NullReferenceException cuando el texto cuelga directo de un
    /// LayoutGroup (en vez de ir envuelto en MakeRow).
    public static Text MakeText(Transform parent, string text, Font font, int fontSize, TextAnchor alignment)
    {
        var go = new GameObject("Text", typeof(RectTransform));
        var rect = (RectTransform)go.transform;
        rect.SetParent(parent, worldPositionStays: false);
        rect.localScale = Vector3.one;
        rect.anchorMin = Vector2.zero;
        rect.anchorMax = Vector2.one;
        rect.offsetMin = Vector2.zero;
        rect.offsetMax = Vector2.zero;
        go.AddComponent<LayoutElement>();

        var t = go.AddComponent<Text>();
        t.text = text;
        t.font = font;
        t.fontSize = fontSize;
        t.alignment = alignment;
        t.color = Color.white;
        t.horizontalOverflow = HorizontalWrapMode.Wrap;
        t.verticalOverflow = VerticalWrapMode.Overflow;
        return t;
    }

    /// Fila completa clickeable: fondo + Button + una Text hija que llena el resto de la
    /// fila. El GameObject del Button ES la fila (mismo que devuelve MakeRow), así que
    /// `button.GetComponent<LayoutElement>()` es válido para fijarle ancho/alto en el
    /// LayoutGroup del padre.
    public static Button MakeButton(Transform parent, string label, Font font, int fontSize)
    {
        var row = MakeRow(parent, "Button");

        var image = row.gameObject.AddComponent<Image>();
        image.color = new Color(0.17f, 0.17f, 0.17f, 1f);

        var button = row.gameObject.AddComponent<Button>();
        button.targetGraphic = image;
        var colors = button.colors;
        colors.highlightedColor = new Color(0.27f, 0.27f, 0.27f, 1f);
        colors.pressedColor     = new Color(0.976f, 0.451f, 0.086f, 1f);
        colors.selectedColor    = colors.highlightedColor;
        button.colors = colors;

        var text = MakeText(row, label, font, fontSize, TextAnchor.MiddleLeft);
        var textRect = (RectTransform)text.transform;
        textRect.offsetMin = new Vector2(12f, 0f);
        textRect.offsetMax = new Vector2(-12f, 0f);

        return button;
    }
}
