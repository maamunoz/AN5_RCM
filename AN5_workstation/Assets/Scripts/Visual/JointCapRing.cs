using UnityEngine;

/// Anillo de color sobre la tapa de una articulacion (como los anillos naranjas del
/// FAIRINO FR5): banda plana en el borde de la cara de la tapa (de innerFraction*radio
/// a radio) mas un borde lateral de wallDepth hacia adentro de la tapa. Eje = +Y
/// local, cara en y = 0 mirando hacia +Y. Se genera como malla al habilitarse; va
/// colgado del eslabon dueno de la tapa, asi que gira con el.
[ExecuteAlways]
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class JointCapRing : MonoBehaviour
{
    [Tooltip("Radio exterior (m): el de la tapa.")]
    public float radius = 0.06f;
    [Range(0.3f, 0.98f)] public float innerFraction = 0.80f;
    [Tooltip("Borde lateral hacia adentro de la tapa (m).")]
    public float wallDepth = 0.004f;
    [Tooltip("Separacion sobre la cara para que no parpadee con la malla del eslabon (m).")]
    public float lift = 0.0006f;
    [Range(24, 192)] public int segments = 96;
    [Tooltip("Cuanto baja el borde exterior respecto del interior (m): banda conica que sigue un canto redondeado en vez de quedar flotando sobre el.")]
    public float outerDrop = 0f;

    float _builtRadius, _builtInner, _builtWall, _builtLift, _builtDrop; int _builtSegments;

    void OnEnable() => Rebuild();

    void OnValidate() => Rebuild();

    void Update()
    {
        if (_builtRadius != radius || _builtInner != innerFraction || _builtWall != wallDepth ||
            _builtLift != lift || _builtSegments != segments || _builtDrop != outerDrop)
            Rebuild();
    }

    public void Rebuild()
    {
        var mf = GetComponent<MeshFilter>();
        if (mf == null) return;
        float rOut = radius, rIn = radius * innerFraction, y0 = lift, yOut = lift - outerDrop, y1 = yOut - wallDepth;
        int n = Mathf.Max(8, segments);
        // normal de la cara: inclinada hacia afuera si la banda es conica
        Vector3 FaceNormal(Vector3 dir) => (Vector3.up * (rOut - rIn) + dir * outerDrop).normalized;

        var v = new Vector3[n * 4];
        var nrm = new Vector3[n * 4];
        var tri = new int[n * 12];
        for (int i = 0; i < n; i++)
        {
            float a = i * Mathf.PI * 2f / n;
            var dir = new Vector3(Mathf.Cos(a), 0f, Mathf.Sin(a));
            // cara (normal +Y): anillo interior y exterior
            v[i] = dir * rIn + Vector3.up * y0;           nrm[i] = FaceNormal(dir);
            v[n + i] = dir * rOut + Vector3.up * yOut;    nrm[n + i] = FaceNormal(dir);
            // borde lateral (normal radial)
            v[2 * n + i] = dir * rOut + Vector3.up * yOut; nrm[2 * n + i] = dir;
            v[3 * n + i] = dir * rOut + Vector3.up * y1;  nrm[3 * n + i] = dir;
        }
        int t = 0;
        for (int i = 0; i < n; i++)
        {
            int j = (i + 1) % n;
            // cara
            tri[t++] = i; tri[t++] = n + j; tri[t++] = n + i;
            tri[t++] = i; tri[t++] = j;     tri[t++] = n + j;
            // borde
            tri[t++] = 2 * n + i; tri[t++] = 2 * n + j; tri[t++] = 3 * n + i;
            tri[t++] = 3 * n + i; tri[t++] = 2 * n + j; tri[t++] = 3 * n + j;
        }

        var mesh = mf.sharedMesh;
        if (mesh == null || mesh.name != "JointCapRing")
        {
            mesh = new Mesh { name = "JointCapRing", hideFlags = HideFlags.DontSave };
            mf.sharedMesh = mesh;
        }
        mesh.Clear();
        mesh.vertices = v;
        mesh.normals = nrm;
        mesh.triangles = tri;
        mesh.RecalculateBounds();

        _builtRadius = radius; _builtInner = innerFraction; _builtWall = wallDepth; _builtLift = lift; _builtSegments = segments; _builtDrop = outerDrop;
    }
}
