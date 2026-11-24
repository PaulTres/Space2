//Body shader for Nav3DAgentPathDrawer's route capsules, drawn with two materials like the line so a body stays readable where geometry
//hides it: a dimmer pass behind the depth buffer, the full one in front. Shaded by the normal against a fixed light so the
//capsule reads as a volume, tinted by _Color, and rimmed along its silhouette in a contrasting tone of that tint, like the
//line's edges, so the body reads on a bright sky and on dark geometry alike.
Shader "Nav3D/AgentPathBody"
{
    Properties
    {
        _Color        ("Tint", Color)                    = (1,1,1,1)
        _OccludedFade ("Alpha multiplier", Range(0,1))   = 1
        [Enum(UnityEngine.Rendering.CompareFunction)] _ZTest ("Depth test", Float) = 4
        _Outline      ("Outline fraction", Range(0,0.5)) = 0.125
    }

    SubShader
    {
        Tags { "Queue" = "Transparent" "RenderType" = "Transparent" "IgnoreProjector" = "True" }

        Blend SrcAlpha OneMinusSrcAlpha
        ZWrite Off
        Cull Back
        Lighting Off

        CGINCLUDE

        #include "UnityCG.cginc"

        fixed4 _Color;
        float  _OccludedFade;
        float  _Outline;

        struct appdata
        {
            float4 vertex  : POSITION;
            float3 normal  : NORMAL;
            //xyz the route's direction at the vertex, w its share of tube (1) against cap (0); PathCapsuleMesh writes it
            float4 tangent : TANGENT;
        };

        struct v2f
        {
            float4 pos      : SV_POSITION;
            float3 normalWS : TEXCOORD0;
            float3 toCamera : TEXCOORD1;
            float4 alongWS  : TEXCOORD2;
            float  shade    : TEXCOORD3;
        };

        v2f vert(appdata _In)
        {
            v2f output;

            float3 positionWS = mul(unity_ObjectToWorld, _In.vertex).xyz;

            output.pos      = UnityObjectToClipPos(_In.vertex);
            output.normalWS = UnityObjectToWorldNormal(_In.normal);
            //toward the camera: from the surface point in a perspective view, along the view axis in an orthographic one
            output.toCamera = lerp(_WorldSpaceCameraPos - positionWS, UNITY_MATRIX_V[2].xyz, unity_OrthoParams.w);
            output.alongWS  = float4(UnityObjectToWorldDir(_In.tangent.xyz), _In.tangent.w);
            output.shade    = 0.55 + 0.45 * saturate(dot(output.normalWS, normalize(float3(0.35, 1.0, 0.25))));

            return output;
        }

        //the rim tone of a colour: the colour pulled toward black when it is light, toward white when it is dark
        fixed3 RimTone(fixed3 _Rgb)
        {
            float luminance = dot(_Rgb, float3(0.2126, 0.7152, 0.0722));

            return luminance > 0.5 ? _Rgb * 0.12 : lerp(_Rgb, fixed3(1, 1, 1), 0.8);
        }

        //where the point sits across the body as the camera sees it, 0 on the middle line and 1 on the silhouette. On the tube
        //it is the normal's share along the screen-space sideways direction (the route's direction crossed with the view),
        //which holds however the tube is foreshortened; the normal's facing alone would paint a tube heading toward or away
        //from the camera rim all over. On a cap it is the sphere's measure; a tube seen almost end-on takes that too
        float Across(float3 _Normal, float3 _ToCamera, float4 _Along)
        {
            float  facing     = saturate(dot(_Normal, _ToCamera));
            float  onSphere   = sqrt(saturate(1.0 - facing * facing));
            float3 sideways   = cross(normalize(_Along.xyz), _ToCamera);
            float  sideLength = length(sideways);
            float  onTube     = abs(dot(_Normal, sideways)) / max(sideLength, 1e-4);
            float  tubeWeight = _Along.w * saturate(sideLength / 0.2);

            return lerp(onSphere, saturate(onTube), tubeWeight);
        }

        //the shaded tint, turning into the rim tone over the outer _Outline of the body's visible radius
        fixed4 Shade(v2f _In)
        {
            fixed4 result = _Color;

            result.rgb *= _In.shade;

            float across = Across(normalize(_In.normalWS), normalize(_In.toCamera), _In.alongWS);
            float aa     = fwidth(across);
            float core   = 1.0 - smoothstep(1.0 - _Outline - aa, 1.0 - _Outline + aa, across);

            result.rgb = lerp(RimTone(_Color.rgb), result.rgb, core);

            return result;
        }

        ENDCG

        Pass
        {
            ZTest [_ZTest]

            CGPROGRAM

            #pragma target 3.0
            #pragma vertex vert
            #pragma fragment frag

            fixed4 frag(v2f _In) : SV_Target
            {
                fixed4 result = Shade(_In);

                result.a *= _OccludedFade;

                return result;
            }

            ENDCG
        }
    }
}
