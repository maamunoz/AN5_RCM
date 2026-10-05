// Piso del visor: plano XY del robot (Z = 0 en la base) dibujado como papel milimetrado.
// Procedural y con antialiasing por fwidth, para que las lineas no parpadeen a distancia.
// Distancias en metros, medidas en el marco del robot (_Origin, _AxisX, _AxisY en mundo).
Shader "AN5/CoordinateGrid"
{
    Properties
    {
        _Origin      ("Origen (mundo)", Vector) = (0, 0, 0, 0)
        _AxisX       ("Eje X del robot (mundo)", Vector) = (-1, 0, 0, 0)
        _AxisY       ("Eje Y del robot (mundo)", Vector) = (0, 0, -1, 0)

        _FillColor   ("Relleno", Color) = (0.04, 0.04, 0.04, 0.35)
        _MinorColor  ("Lineas menores", Color) = (0.55, 0.55, 0.55, 0.22)
        _MajorColor  ("Lineas mayores", Color) = (0.75, 0.75, 0.75, 0.45)
        _AxisXColor  ("Eje X", Color) = (0.976, 0.451, 0.086, 0.95)
        _AxisYColor  ("Eje Y", Color) = (0.290, 0.620, 1.000, 0.95)
        _ReachColor  ("Circulo de alcance", Color) = (0.976, 0.451, 0.086, 0.45)

        _MinorStep   ("Paso menor (m)", Float) = 0.1
        _MajorStep   ("Paso mayor (m)", Float) = 0.5
        _MinorWidth  ("Ancho menor (px)", Float) = 1.0
        _MajorWidth  ("Ancho mayor (px)", Float) = 1.5
        _AxisWidth   ("Ancho ejes (px)", Float) = 2.5
        _ReachRadius ("Radio de alcance (m)", Float) = 0.886
        _FadeStart   ("Inicio desvanecido (m)", Float) = 0.7
        _FadeEnd     ("Fin desvanecido (m)", Float) = 1.4
        // El proyecto mezcla en espacio lineal: un alfa bajo se ve bastante mas claro de
        // lo que su numero sugiere, asi que la intensidad general se ajusta aqui.
        _Opacity     ("Opacidad general", Range(0, 1)) = 0.35
    }

    SubShader
    {
        Tags { "Queue" = "Transparent" "RenderType" = "Transparent" "IgnoreProjector" = "True" }
        Blend SrcAlpha OneMinusSrcAlpha
        ZWrite Off
        Cull Off

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            float4 _Origin, _AxisX, _AxisY;
            float4 _FillColor, _MinorColor, _MajorColor, _AxisXColor, _AxisYColor, _ReachColor;
            float _MinorStep, _MajorStep, _MinorWidth, _MajorWidth, _AxisWidth;
            float _ReachRadius, _FadeStart, _FadeEnd, _Opacity;

            struct v2f
            {
                float4 pos   : SV_POSITION;
                float3 world : TEXCOORD0;
            };

            v2f vert(appdata_base v)
            {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);
                o.world = mul(unity_ObjectToWorld, v.vertex).xyz;
                return o;
            }

            // Cobertura [0,1] de lineas repetidas cada `step`, con ancho en pixeles de pantalla.
            float GridLines(float2 p, float step, float widthPx)
            {
                float2 c = p / step;
                float2 d = fwidth(c);
                float2 g = abs(frac(c - 0.5) - 0.5) / max(d, 1e-5);
                return 1.0 - saturate(min(g.x, g.y) - (widthPx - 1.0) * 0.5);
            }

            // Cobertura de una sola linea en v = 0.
            float Line(float v, float widthPx)
            {
                float d = abs(v) / max(fwidth(v), 1e-5);
                return 1.0 - saturate(d - (widthPx - 1.0) * 0.5);
            }

            // Pinta `c` encima de `dst` con cobertura `k` (alfa premultiplicado a mano).
            float4 Over(float4 dst, float4 c, float k)
            {
                float a = c.a * k;
                float outA = a + dst.a * (1.0 - a);
                float3 rgb = (c.rgb * a + dst.rgb * dst.a * (1.0 - a)) / max(outA, 1e-5);
                return float4(rgb, outA);
            }

            float4 frag(v2f i) : SV_Target
            {
                float3 rel = i.world - _Origin.xyz;
                float2 p = float2(dot(rel, _AxisX.xyz), dot(rel, _AxisY.xyz));
                float r = length(p);

                float4 col = _FillColor;
                col = Over(col, _MinorColor, GridLines(p, _MinorStep, _MinorWidth));
                col = Over(col, _MajorColor, GridLines(p, _MajorStep, _MajorWidth));
                col = Over(col, _ReachColor, Line(r - _ReachRadius, _MajorWidth));
                // El semieje positivo se pinta entero y el negativo a media intensidad,
                // asi se lee hacia donde crece cada coordenada.
                float4 ax = _AxisXColor; ax.a *= (p.x >= 0) ? 1.0 : 0.35;
                float4 ay = _AxisYColor; ay.a *= (p.y >= 0) ? 1.0 : 0.35;
                col = Over(col, ax, Line(p.y, _AxisWidth));
                col = Over(col, ay, Line(p.x, _AxisWidth));

                col.a *= (1.0 - smoothstep(_FadeStart, _FadeEnd, r)) * _Opacity;
                return col;
            }
            ENDCG
        }
    }
}
