// An unlit colour drawn over everything: no depth test and no depth write, in the overlay queue, so what the player must see or
// click (PointVsVolumetric's beacons and its two answers) is never hidden behind a wall. _Shade darkens the faces that turn away
// from the eye, which keeps a volume such as the route tube reading as round; 0 leaves a flat colour for lines and markers
Shader "Nav3D/Demo/Overlay Unlit"
{
    Properties
    {
        _Color ("Color", Color) = (1, 1, 1, 1)
        _Shade ("Shade", Range(0, 1)) = 0
    }

    SubShader
    {
        Tags { "Queue" = "Overlay" "RenderType" = "Transparent" "IgnoreProjector" = "True" }

        ZTest Always
        ZWrite Off
        Cull Back
        Blend SrcAlpha OneMinusSrcAlpha

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            fixed4 _Color;
            half   _Shade;

            struct appdata
            {
                float4 vertex : POSITION;
                float3 normal : NORMAL;
                fixed4 color  : COLOR;
            };

            struct v2f
            {
                float4 pos   : SV_POSITION;
                fixed4 color : COLOR;
            };

            v2f vert(appdata v)
            {
                v2f o;

                o.pos = UnityObjectToClipPos(v.vertex);

                float3 normal  = normalize(UnityObjectToWorldNormal(v.normal));
                float3 toEye   = normalize(WorldSpaceViewDir(v.vertex));
                half   facing  = saturate(dot(normal, toEye));
                half   shading = lerp(1, 0.45 + 0.55 * facing, _Shade);

                o.color      = v.color * _Color;
                o.color.rgb *= shading;

                return o;
            }

            fixed4 frag(v2f i) : SV_Target
            {
                return i.color;
            }
            ENDCG
        }
    }
}
