using UnityEngine;

/// Modelo visual de un laparoscopio rigido, armado con primitivas: acople a la brida,
/// cabezal de camara con anillo, ocular con conector de luz lateral, vaina y punta con
/// lente. Eje local +Y = eje del instrumento, origen = brida. FulcroController lo
/// muestra en lugar de la pinza mientras el fulcro esta activo y lo alinea con el eje
/// de la cadena DH; SetLength estira la vaina para que la punta caiga justo en la punta
/// que usa la IK.
///
/// Va en la raiz de la escena, no colgado de la cadena del robot: j6_Link tiene escala
/// no uniforme (1.2 en Y) y deformaria el modelo.
public class EndoscopeVisual : MonoBehaviour
{
    [Header("Materiales")]
    public Material bodyMaterial;
    public Material metalMaterial;
    public Material accentMaterial;
    public Material lensMaterial;

    // Cotas en metros, desde la brida a lo largo de +Y.
    const float CouplerDiam = 0.063f, CouplerLen = 0.012f;
    const float HeadDiam = 0.040f, HeadLen = 0.085f;
    const float RingDiam = 0.0412f, RingLen = 0.008f, RingAt = 0.024f;
    const float EyeDiam = 0.024f, EyeLen = 0.020f;
    const float LightPostDiam = 0.009f, LightPostLen = 0.026f, LightCapDiam = 0.013f, LightCapLen = 0.006f;
    const float ShaftDiam = 0.010f, TipDiam = 0.0104f, TipLen = 0.006f, LensDiam = 0.006f;

    const float HeadStart = CouplerLen;
    const float EyeStart = HeadStart + HeadLen;
    const float ShaftStart = EyeStart + EyeLen;

    Transform _shaft, _tip, _lens;
    float _length = -1f;

    /// Largo total brida -> punta (m) minimo para que la vaina tenga sentido.
    public static float MinLength => ShaftStart + TipLen + 0.01f;

    void Awake()
    {
        if (transform.Find("Vaina") == null) Build();
        CacheParts();
    }

    /// Arma (o rearma) el modelo con un largo inicial de 0.5 m.
    [ContextMenu("Rebuild")]
    public void Build()
    {
        for (int i = transform.childCount - 1; i >= 0; i--)
        {
            var c = transform.GetChild(i).gameObject;
            if (Application.isPlaying) Destroy(c); else DestroyImmediate(c);
        }

        Part("Acople",   CouplerDiam, CouplerLen, CouplerLen / 2f, bodyMaterial);
        Part("Cabezal",  HeadDiam, HeadLen, HeadStart + HeadLen / 2f, bodyMaterial);
        Part("Anillo",   RingDiam, RingLen, HeadStart + RingAt, accentMaterial);
        Part("Ocular",   EyeDiam, EyeLen, EyeStart + EyeLen / 2f, metalMaterial);

        // Conector de luz: sale de costado (+X local) del ocular.
        var post = Part("PostLuz", LightPostDiam, LightPostLen, EyeStart + EyeLen / 2f, metalMaterial);
        post.localRotation = Quaternion.Euler(0f, 0f, 90f);
        post.localPosition = new Vector3(EyeDiam / 2f + LightPostLen / 2f, post.localPosition.y, 0f);
        var cap = Part("TapaLuz", LightCapDiam, LightCapLen, EyeStart + EyeLen / 2f, bodyMaterial);
        cap.localRotation = Quaternion.Euler(0f, 0f, 90f);
        cap.localPosition = new Vector3(EyeDiam / 2f + LightPostLen + LightCapLen / 2f, cap.localPosition.y, 0f);

        Part("Vaina", ShaftDiam, 0.1f, ShaftStart + 0.05f, metalMaterial);
        Part("Punta", TipDiam, TipLen, 0.5f, bodyMaterial);
        var lens = Part("Lente", LensDiam, LensDiam, 0.5f, lensMaterial, PrimitiveType.Sphere);
        lens.localScale = Vector3.one * LensDiam;

        CacheParts();
        _length = -1f;
        SetLength(0.5f);
    }

    /// Largo brida -> punta del lente, en metros.
    public void SetLength(float totalM)
    {
        if (_shaft == null) CacheParts();
        if (_shaft == null) return;
        totalM = Mathf.Max(totalM, MinLength);
        if (Mathf.Abs(totalM - _length) < 1e-5f) return;
        _length = totalM;

        float shaftEnd = totalM - TipLen;
        float shaftLen = shaftEnd - ShaftStart;
        _shaft.localPosition = new Vector3(0f, ShaftStart + shaftLen / 2f, 0f);
        _shaft.localScale = new Vector3(ShaftDiam, shaftLen / 2f, ShaftDiam); // cilindro primitivo: 2 m de alto
        _tip.localPosition = new Vector3(0f, shaftEnd + TipLen / 2f, 0f);
        _lens.localPosition = new Vector3(0f, totalM - LensDiam / 2f, 0f);
    }

    void CacheParts()
    {
        _shaft = transform.Find("Vaina");
        _tip = transform.Find("Punta");
        _lens = transform.Find("Lente");
    }

    Transform Part(string name, float diam, float len, float centerY, Material mat,
                   PrimitiveType type = PrimitiveType.Cylinder)
    {
        var go = GameObject.CreatePrimitive(type);
        go.name = name;
        var col = go.GetComponent<Collider>();
        if (Application.isPlaying) Destroy(col); else DestroyImmediate(col);
        var t = go.transform;
        t.SetParent(transform, false);
        t.localPosition = new Vector3(0f, centerY, 0f);
        t.localScale = new Vector3(diam, len / 2f, diam);
        var r = go.GetComponent<MeshRenderer>();
        if (mat != null) r.sharedMaterial = mat;
        r.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        return t;
    }
}
