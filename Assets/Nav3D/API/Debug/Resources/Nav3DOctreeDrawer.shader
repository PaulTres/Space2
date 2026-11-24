//Instanced shader for Nav3DOctreeDrawer. The update pulse decays here, by time, so a quiet frame costs the CPU
//nothing; the NAV3D_OCCUPANCY_LINES variant draws wireframe cells and graph edges, with the pulse per vertex.
Shader "Nav3D/OctreeDrawer"
{
    Properties
    {
        _CellColor    ("Cell Color", Color) = (0.9, 0.3, 0.25, 0.08)
        _PulseColor   ("Pulse Color", Color) = (0.9, 0.3, 0.25, 1)
        _PulseAlpha   ("Pulse Alpha", Float) = 0.3
        _PulseSeconds ("Pulse Seconds", Float) = 1.2
        //the pulse clock, set by the drawer: real time, so a pulse fades on schedule in the paused editor too
        _PulseNow     ("Pulse Now", Float) = 0
        _FresnelPower ("Fresnel Power", Float) = 2.0
        //LEqual; the interior's X-ray material sets Always
        [Enum(UnityEngine.Rendering.CompareFunction)] _ZTest ("ZTest", Float) = 4
    }

    SubShader
    {
        Tags { "Queue" = "Transparent" "RenderType" = "Transparent" "IgnoreProjector" = "True" }

        Pass
        {
            Blend SrcAlpha OneMinusSrcAlpha
            ZWrite Off
            ZTest [_ZTest]
            Cull Back

            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_instancing
            #pragma multi_compile_local _ NAV3D_OCCUPANCY_LINES
            #include "UnityCG.cginc"

            float4 _CellColor;
            float4 _PulseColor;
            float  _PulseAlpha;
            float  _PulseSeconds;
            float  _PulseNow;
            float  _FresnelPower;

            UNITY_INSTANCING_BUFFER_START(Props)
                //x = pulse start time (the _PulseNow clock), y = 1 when the instance
                //was pulsed, zw unused
                UNITY_DEFINE_INSTANCED_PROP(float4, _PulseData)
            UNITY_INSTANCING_BUFFER_END(Props)

            struct appdata
            {
                float4 vertex : POSITION;
            #if NAV3D_OCCUPANCY_LINES
                //x = pulse start time, y = 1 when the vertex belongs to a pulsed cell
                float2 pulse  : TEXCOORD0;
            #else
                float3 normal : NORMAL;
            #endif
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            struct v2f
            {
                float4 pos         : SV_POSITION;
            #if NAV3D_OCCUPANCY_LINES
                float2 pulse       : TEXCOORD0;
            #else
                float3 worldNormal : TEXCOORD0;
                float3 viewDir     : TEXCOORD1;
            #endif
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            v2f vert (appdata v)
            {
                v2f o;

                UNITY_SETUP_INSTANCE_ID(v);
                UNITY_TRANSFER_INSTANCE_ID(v, o);

                o.pos = UnityObjectToClipPos(v.vertex);

            #if NAV3D_OCCUPANCY_LINES
                o.pulse = v.pulse;
            #else
                o.worldNormal = UnityObjectToWorldNormal(v.normal);
                o.viewDir     = _WorldSpaceCameraPos - mul(unity_ObjectToWorld, v.vertex).xyz;
            #endif

                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                UNITY_SETUP_INSTANCE_ID(i);

                float3 color = _CellColor.rgb;
                float  alpha = _CellColor.a;

            #if NAV3D_OCCUPANCY_LINES
                float4 pulse = float4(i.pulse, 0.0, 0.0);
            #else
                float4 pulse = UNITY_ACCESS_INSTANCED_PROP(Props, _PulseData);
            #endif

                if (pulse.y > 0.5)
                {
                    //full pulse brightness at the stamp moment, linear decay to the steady alpha (or to nothing when
                    //the steady shell is off — the CPU passes _CellColor.a = 0 then)
                    float decay = saturate(1.0 - (_PulseNow - pulse.x) / max(_PulseSeconds, 0.0001));

                    color = lerp(color, _PulseColor.rgb, decay);
                    alpha = lerp(alpha, _PulseAlpha, decay);
                }

            #if !NAV3D_OCCUPANCY_LINES
                if (_FresnelPower > 0.0)
                {
                    //edge-brightening: faces seen head-on fade, silhouette edges hold, so a cube reads as a cube at
                    //very low alpha
                    float fresnel = 1.0 - saturate(dot(normalize(i.viewDir), normalize(i.worldNormal)));

                    alpha *= lerp(0.6, 1.6, pow(fresnel, _FresnelPower));
                }
            #endif

                return fixed4(color, saturate(alpha));
            }
            ENDCG
        }
    }
}
